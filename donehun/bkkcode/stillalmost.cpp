// histequalize_host.c — decoupled appsink->appsrc with ring buffer + paced pusher @ target fps
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

typedef struct {
    GstElement   *appsrc;
    GstElement   *appsink;
    gboolean      video_info_valid;
    GstVideoInfo  video_info;

    // Ring buffer between appsink and appsrc
    GAsyncQueue  *queue;        // holds GstBuffer* (ref'ed)
    gint          ring_capacity; // max number of buffers to keep

    // Pusher thread pacing state
    GThread      *pusher_thread;
    GMainLoop    *loop;
    gint          target_fps;
    gboolean      running;
} CustomData;

static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int signum) { (void)signum; g_stop = 1; }

// ---------- Bus watch ----------
static gboolean on_bus_message(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus;
    GMainLoop *loop = (GMainLoop *)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError *err=NULL; gchar *dbg=NULL;
        gst_message_parse_error(msg, &err, &dbg);
        g_printerr("[BUS] ERROR: %s\n", err->message);
        if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
        g_clear_error(&err); g_free(dbg);
        if (loop) g_main_loop_quit(loop);
        break;
    }
    case GST_MESSAGE_EOS:
        g_printerr("[BUS] EOS\n");
        if (loop) g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_WARNING: {
        GError *err=NULL; gchar *dbg=NULL;
        gst_message_parse_warning(msg, &err, &dbg);
        g_printerr("[BUS] WARN: %s\n", err->message);
        if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
        g_clear_error(&err); g_free(dbg);
        break;
    }
    default: break;
    }
    return TRUE;
}

// ---------- Encoder sink probe (to verify actual fps at encoder input) ----------
static GstPadProbeReturn enc_sink_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    static guint64 cnt = 0;
    (void)pad; (void)user_data;
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (buf) {
            cnt++;
            if ((cnt % 120) == 0) {
                g_print("[enc sink] frames=%" G_GUINT64_FORMAT " last PTS=%" GST_TIME_FORMAT "\n",
                        cnt, GST_TIME_ARGS(GST_BUFFER_PTS(buf)));
            }
        }
    }
    return GST_PAD_PROBE_PASS;
}

// ---------- Appsink callback: push into ring (non-blocking), drop oldest if full ----------
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Negotiated: %dx%d %s @ %d/%d\n",
                    data->video_info.width, data->video_info.height,
                    gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&data->video_info)),
                    data->video_info.fps_n, data->video_info.fps_d);
        } else {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Ref the buffer and enqueue
    gst_buffer_ref(buffer);

    // If queue is full, drop oldest to avoid backpressure on capture
    while (g_async_queue_length(data->queue) >= data->ring_capacity) {
        GstBuffer *old = (GstBuffer*)g_async_queue_try_pop(data->queue);
        if (old) gst_buffer_unref(old);
        else break;
    }
    g_async_queue_push(data->queue, buffer);

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// ---------- Pusher thread: pace appsrc at target fps ----------
static gpointer pusher_main(gpointer user_data) {
    CustomData *data = (CustomData*)user_data;
    const gdouble period_ms = 1000.0 / (gdouble)data->target_fps;

    // Use monotonic time for pacing
    gint64 next_ts_us = g_get_monotonic_time();
    guint64 pushed = 0;

    while (G_LIKELY(data->running && !g_stop)) {
        // Sleep until next tick
        next_ts_us += (gint64)(period_ms * 1000.0);
        gint64 now = g_get_monotonic_time();
        if (now < next_ts_us) {
            g_usleep((guint)(next_ts_us - now));
        } else {
            // we're late; catch up without sleeping
            next_ts_us = now;
        }

        // Pop newest available buffer; if empty, skip (no push this tick)
        GstBuffer *buf = (GstBuffer*)g_async_queue_try_pop(data->queue);
        if (!buf) continue;

        // Push; appsrc(do-timestamp=true) will stamp running-time at push
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), buf);
        if (ret != GST_FLOW_OK) {
            // If downstream is blocked for long, we keep pulling latest frame and keep pacing.
            // (buf is already owned by appsrc or unref'ed by appsrc on failure)
            if (ret == GST_FLOW_FLUSHING) break;
        } else {
            pushed++;
            if ((pushed % 120) == 0) {
                g_print("[pusher] pushed=%" G_GUINT64_FORMAT " @ ~%d fps\n", pushed, data->target_fps);
            }
        }
    }

    // Drain queue on exit
    for (;;) {
        GstBuffer *left = (GstBuffer*)g_async_queue_try_pop(data->queue);
        if (!left) break;
        gst_buffer_unref(left);
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // ---- CLI ----
    gint bitrate_kbps = 10000;
    gint fps          = 60;
    gint width        = 1920;
    gint height       = 1080;
    const gchar *device   = "/dev/video0";
    const gchar *dst_host = "192.168.25.69";
    gint dst_port         = 5004;
    enum { CODEC_H264, CODEC_H265 } codec = CODEC_H264;

    for (int i = 1; i < argc; i++) {
        if      (g_str_has_prefix(argv[i], "--bitrate=")) bitrate_kbps = atoi(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--fps="))     fps          = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--width="))   width        = atoi(argv[i] + 8);
        else if (g_str_has_prefix(argv[i], "--height="))  height       = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--device="))  device       = argv[i] + 9;
        else if (g_str_has_prefix(argv[i], "--host="))    dst_host     = argv[i] + 7;
        else if (g_str_has_prefix(argv[i], "--port="))    dst_port     = atoi(argv[i] + 7);
        else if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *c = argv[i] + 8;
            if (!g_ascii_strcasecmp(c, "h265") || !g_ascii_strcasecmp(c, "hevc")) codec = CODEC_H265;
            else codec = CODEC_H264;
        }
    }

    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d\n",
            device, width, height, fps, bitrate_kbps,
            (codec==CODEC_H264?"H.264":"H.265"), dst_host, dst_port);

    // ---- Build pipelines ----
    CustomData data = {0};
    data.queue = g_async_queue_new();
    data.ring_capacity = 12;       // ~200 ms of cushion at 60 fps
    data.target_fps = fps;
    data.running = TRUE;

    // Capture/appsink: negotiate the camera to the requested fps directly (no videorate)
    // Keep appsink non-blocking (drop=true) so capture never stalls; we manage drops in ring.
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
        "queue max-size-buffers=8 leaky=downstream ! "
        "appsink name=cv_sink emit-signals=true max-buffers=8 drop=true sync=false",
        device, width, height, fps);

    GError *error = NULL;
    GstElement *app_sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!app_sink_pipeline) {
        g_printerr("Failed to create appsink pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        return -1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(app_sink_pipeline), "cv_sink");
    if (!data.appsink) {
        g_printerr("Failed to get appsink element\n");
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    // Encode/appsrc pipeline (non-leaky small queues; padding on HEVC)
    gchar *src_pipeline_str = NULL;
    int idr = fps * 4; if (idr < 60) idr = 60;

    if (codec == CODEC_H264) {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=6 leaky=no ! "
            "omxh264enc name=enc num-slices=8 periodicity-idr=%d "
            "cpb-size=500 gdr-mode=horizontal initial-delay=250 "
            "control-rate=low-latency prefetch-buffer=true "
            "target-bitrate=%d gop-mode=low-delay-p ! "
            "video/x-h264, alignment=nal, stream-format=byte-stream ! "
            "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            "queue max-size-buffers=6 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps, idr, bitrate_kbps, dst_host, dst_port);
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=6 leaky=no ! "
            "omxh265enc name=enc control-rate=constant target-bitrate=%d "
            "gop-mode=low-delay-p gop-length=%d periodicity-idr=%d "
            "num-slices=8 qp-mode=auto prefetch-buffer=true "
            "cpb-size=500 initial-delay=250 gdr-mode=horizontal filler-data=true ! "
            "video/x-h265, alignment=au, profile=main, stream-format=byte-stream ! "
            "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
            "queue max-size-buffers=6 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps, bitrate_kbps, fps, idr, dst_host, dst_port);
    }

    GstElement *app_src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!app_src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "cv_src");
    if (!data.appsrc) {
        g_printerr("Failed to get appsrc element\n");
        gst_object_unref(app_src_pipeline);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    // Configure appsrc: live + timestamps + backpressure (block) so queues upstream don't explode
    g_object_set(data.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "block", TRUE,
                 "max-bytes", 4*1024*1024,
                 NULL);

    // Set caps on appsrc for stable negotiation
    GstCaps *appsrc_caps = gst_caps_new_simple("video/x-raw",
        "format",  G_TYPE_STRING, "NV12",
        "width",   G_TYPE_INT,    width,
        "height",  G_TYPE_INT,    height,
        "framerate", GST_TYPE_FRACTION, fps, 1, NULL);
    gst_app_src_set_caps(GST_APP_SRC(data.appsrc), appsrc_caps);
    gst_caps_unref(appsrc_caps);

    // Hook callback (capture -> ring)
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Shared system clock (reduces drift)
    GstClock *sysclk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(app_sink_pipeline), sysclk);
    gst_pipeline_use_clock(GST_PIPELINE(app_src_pipeline),  sysclk);
    gst_object_unref(sysclk);

    // Bus watches
    data.loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus_sink = gst_element_get_bus(app_sink_pipeline);
    GstBus *bus_src  = gst_element_get_bus(app_src_pipeline);
    gst_bus_add_watch(bus_sink, on_bus_message, data.loop);
    gst_bus_add_watch(bus_src,  on_bus_message, data.loop);
    gst_object_unref(bus_sink);
    gst_object_unref(bus_src);

    // Encoder sink probe
    GstElement *enc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "enc");
    if (enc) {
        GstPad *sinkpad = gst_element_get_static_pad(enc, "sink");
        if (sinkpad) {
            gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe, NULL, NULL);
            gst_object_unref(sinkpad);
        }
        gst_object_unref(enc);
    }

    // Start pipelines
    gst_element_set_state(app_src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);

    // Start pusher thread (pace at target fps)
    data.pusher_thread = g_thread_new("pusher", pusher_main, &data);

    // SIGINT handling
    signal(SIGINT, handle_sigint);
    g_print("Huiiiiiiiiiiiiiing. Press Ctrl+C to exit.\n");

    while (!g_stop) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000);
    }

    // Teardown
    g_print("Stopping...\n");
    data.running = FALSE;
    if (data.pusher_thread) {
        g_thread_join(data.pusher_thread);
    }

    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(app_src_pipeline,  GST_STATE_NULL);

    if (data.queue) {
        // ensure empty
        for (;;) {
            GstBuffer *b = (GstBuffer*)g_async_queue_try_pop(data.queue);
            if (!b) break;
            gst_buffer_unref(b);
        }
        g_async_queue_unref(data.queue);
    }

    g_main_loop_unref(data.loop);
    gst_object_unref(app_sink_pipeline);
    gst_object_unref(app_src_pipeline);
    return 0;
}
