// histequalize_host.c — Ring buffer + pipeline-clock-paced appsrc with explicit 60fps PTS
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

typedef struct {
    // Elements
    GstElement   *appsrc;
    GstElement   *appsink;
    GstElement   *app_src_pipeline;
    GstElement   *app_sink_pipeline;

    // Video info
    gboolean      video_info_valid;
    GstVideoInfo  video_info;

    // Ring buffer (appsink -> pusher)
    GAsyncQueue  *queue;         // holds GstBuffer* (ref'ed)
    gint          ring_capacity; // max buffers retained

    // Pusher thread pacing
    GThread      *pusher_thread;
    gboolean      running;
    gint          target_fps;

    // Pacing with pipeline clock
    GstClock     *clock;         // pipeline clock (ref'ed)
    GstClockTime  base_time;     // pipeline base time
    GstClockTime  frame_duration;// GST_SECOND / fps

    // Main loop
    GMainLoop    *loop;

    // Codec choose
    gboolean      is_h265;
} CustomData;

static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int signum) { (void)signum; g_stop = 1; }

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

// Encoder sink probe: confirm actual input cadence
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

// appsink callback: enqueue latest buffer (drop oldest if ring full)
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

    // Ref and push newest into ring; drop oldest if full
    gst_buffer_ref(buffer);
    while (g_async_queue_length(data->queue) >= data->ring_capacity) {
        GstBuffer *old = (GstBuffer*)g_async_queue_try_pop(data->queue);
        if (old) gst_buffer_unref(old); else break;
    }
    g_async_queue_push(data->queue, buffer);

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// Pusher: wait on pipeline clock each frame slot, set exact PTS/DURATION, push
static gpointer pusher_main(gpointer user_data) {
    CustomData *d = (CustomData*)user_data;

    // Ensure we have valid base_time and clock
    if (!d->clock || d->base_time == GST_CLOCK_TIME_NONE) {
        g_printerr("[pusher] Missing clock/base_time\n");
        return NULL;
    }

    GstClockTime frame = 0; // running-time
    guint64 pushed = 0;

    while (G_LIKELY(d->running && !g_stop)) {
        // Target running-time for this frame
        GstClockTime target = d->base_time + frame;

        // Wait until target on pipeline clock
        GstClockID id = gst_clock_new_single_shot_id(d->clock, target);
        if (gst_clock_id_wait(id, NULL) != GST_CLOCK_OK) {
            gst_clock_id_unref(id);
            if (!d->running || g_stop) break;
        } else {
            gst_clock_id_unref(id);
        }

        // Pop latest buffer (if none, skip this slot)
        GstBuffer *buf = (GstBuffer*)g_async_queue_try_pop(d->queue);
        if (!buf) {
            frame += d->frame_duration;
            continue;
        }

        // Stamp exact PTS/DURATION at 60fps; let DTS be NONE
        GST_BUFFER_PTS(buf) = frame;
        GST_BUFFER_DTS(buf) = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DURATION(buf) = d->frame_duration;

        // Push (non-blocking in terms of timestamps; appsrc may block if downstream stalls)
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), buf);
        if (ret == GST_FLOW_FLUSHING) break;

        pushed++;
        if ((pushed % 120) == 0) {
            g_print("[pusher] pushed=%" G_GUINT64_FORMAT " (rt=%" GST_TIME_FORMAT ")\n",
                    pushed, GST_TIME_ARGS(frame));
        }

        frame += d->frame_duration;
    }

    // Drain remaining buffers
    for (;;) {
        GstBuffer *left = (GstBuffer*)g_async_queue_try_pop(d->queue);
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
    gboolean use_h265     = FALSE;

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
            use_h265 = (!g_ascii_strcasecmp(c, "h265") || !g_ascii_strcasecmp(c, "hevc"));
        }
    }

    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d\n",
            device, width, height, fps, bitrate_kbps,
            (use_h265?"H.265":"H.264"), dst_host, dst_port);

    CustomData d = {0};
    d.queue         = g_async_queue_new();
    d.ring_capacity = 16;              // ~266 ms at 60fps
    d.target_fps    = fps;
    d.frame_duration= GST_SECOND / (guint)fps;
    d.running       = TRUE;
    d.is_h265       = use_h265;

    // ---- Build appsink (capture) pipeline ----
    // Negotiate camera directly to requested fps. Keep capture flowing (drop at source).
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
        "queue max-size-buffers=8 leaky=downstream ! "
        "appsink name=cv_sink emit-signals=true max-buffers=8 drop=true sync=false",
        device, width, height, fps);

    GError *error = NULL;
    d.app_sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!d.app_sink_pipeline) {
        g_printerr("Failed to create appsink pipeline: %s\n", error? error->message : "unknown");
        if (error) g_clear_error(&error);
        return -1;
    }
    d.appsink = gst_bin_get_by_name(GST_BIN(d.app_sink_pipeline), "cv_sink");
    if (!d.appsink) {
        g_printerr("Failed to get appsink element\n");
        gst_object_unref(d.app_sink_pipeline);
        return -1;
    }
    g_signal_connect(d.appsink, "new-sample", G_CALLBACK(new_sample_cb), &d);

    // ---- Build appsrc (encode/stream) pipeline ----
    gchar *src_pipeline_str = NULL;
    int idr = fps * 4; if (idr < 60) idr = 60;

    if (!use_h265) {
        // H.264: set output caps to High / Level 4.2 to avoid encoder throttling
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=6 leaky=no ! "
            "omxh264enc name=enc num-slices=8 periodicity-idr=%d "
            "cpb-size=500 initial-delay=250 control-rate=low-latency prefetch-buffer=true "
            "target-bitrate=%d gop-mode=low-delay-p ! "
            "video/x-h264, stream-format=byte-stream, alignment=nal, profile=high, level=(string)4.2 ! "
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
            "cpb-size=500 initial-delay=250 filler-data=true ! "
            "video/x-h265, alignment=au, profile=main, stream-format=byte-stream ! "
            "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
            "queue max-size-buffers=6 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps, bitrate_kbps, fps, idr, dst_host, dst_port);
    }

    d.app_src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!d.app_src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error? error->message : "unknown");
        if (error) g_clear_error(&error);
        gst_object_unref(d.app_sink_pipeline);
        return -1;
    }
    d.appsrc = gst_bin_get_by_name(GST_BIN(d.app_src_pipeline), "cv_src");
    if (!d.appsrc) {
        g_printerr("Failed to get appsrc element\n");
        gst_object_unref(d.app_src_pipeline);
        gst_object_unref(d.app_sink_pipeline);
        return -1;
    }

    // Make appsrc live but DO NOT timestamp (we set PTS ourselves)
    g_object_set(d.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", FALSE,     // we'll stamp exact PTS/DURATION
                 "block", TRUE,
                 "max-bytes", 4*1024*1024,
                 NULL);

    // If your omxh264enc supports properties for profile/level, you can enforce:
    // GstElement *enc = gst_bin_get_by_name(GST_BIN(d.app_src_pipeline), "enc");
    // if (enc && !use_h265) {
    //     g_object_set(enc, "profile",  "high", NULL);
    //     g_object_set(enc, "level",    "4.2",  NULL);
    //     // Some builds expose "cabac" or "entropy-mode"; set if available:
    //     // g_object_set(enc, "cabac", TRUE, NULL);
    // }
    // if (enc) gst_object_unref(enc);

    // Probe to print encoder input cadence
    {
        GstElement *enc = gst_bin_get_by_name(GST_BIN(d.app_src_pipeline), "enc");
        if (enc) {
            GstPad *sinkpad = gst_element_get_static_pad(enc, "sink");
            if (sinkpad) {
                gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe, NULL, NULL);
                gst_object_unref(sinkpad);
            }
            gst_object_unref(enc);
        }
    }

    // Share the same system clock across both pipelines
    GstClock *sysclk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(d.app_sink_pipeline), sysclk);
    gst_pipeline_use_clock(GST_PIPELINE(d.app_src_pipeline),  sysclk);
    gst_object_unref(sysclk);

    // Bus watches
    d.loop = g_main_loop_new(NULL, FALSE);
    {
        GstBus *bus_sink = gst_element_get_bus(d.app_sink_pipeline);
        GstBus *bus_src  = gst_element_get_bus(d.app_src_pipeline);
        gst_bus_add_watch(bus_sink, on_bus_message, d.loop);
        gst_bus_add_watch(bus_src,  on_bus_message, d.loop);
        gst_object_unref(bus_sink);
        gst_object_unref(bus_src);
    }

    // Start both pipelines first, THEN latch base-time/clock for precise pacing
    gst_element_set_state(d.app_src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(d.app_sink_pipeline, GST_STATE_PLAYING);

    // Latch pipeline clock/base time for appsrc pipeline
    d.clock     = gst_element_get_clock(d.app_src_pipeline); // ref
    d.base_time = gst_element_get_base_time(d.app_src_pipeline);
    if (!d.clock || d.base_time == GST_CLOCK_TIME_NONE) {
        g_printerr("Failed to get pipeline clock/base_time\n");
        return -1;
    }

    signal(SIGINT, handle_sigint);
    g_print("Running. Press Ctrl+C to exit.\n");

    // Start pusher (clock-accurate)
    d.pusher_thread = g_thread_new("pusher", pusher_main, &d);

    while (!g_stop) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000);
    }

    // Teardown
    g_print("Stopping...\n");
    d.running = FALSE;
    if (d.pusher_thread) g_thread_join(d.pusher_thread);

    gst_element_set_state(d.app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(d.app_src_pipeline,  GST_STATE_NULL);

    if (d.clock) gst_object_unref(d.clock);

    if (d.queue) {
        for (;;) {
            GstBuffer *b = (GstBuffer*)g_async_queue_try_pop(d.queue);
            if (!b) break;
            gst_buffer_unref(b);
        }
        g_async_queue_unref(d.queue);
    }

    g_main_loop_unref(d.loop);
    gst_object_unref(d.app_sink_pipeline);
    gst_object_unref(d.app_src_pipeline);
    return 0;
}
