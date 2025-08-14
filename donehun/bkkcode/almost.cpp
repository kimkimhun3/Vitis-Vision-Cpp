// histequalize_host.c — appsink->appsrc bridge tuned for full target bitrate at 60fps
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
    GMainLoop    *loop;
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

// Simple encoder sink pad probe to confirm fps and cadence
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

    // No timestamp edits: appsrc(do-timestamp=true) will stamp running-time.
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), gst_buffer_ref(buffer));
    gst_sample_unref(sample);
    return ret;
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
        if (g_str_has_prefix(argv[i], "--bitrate="))       bitrate_kbps = atoi(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--fps="))      fps          = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--width="))    width        = atoi(argv[i] + 8);
        else if (g_str_has_prefix(argv[i], "--height="))   height       = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--device="))   device       = argv[i] + 9;
        else if (g_str_has_prefix(argv[i], "--host="))     dst_host     = argv[i] + 7;
        else if (g_str_has_prefix(argv[i], "--port="))     dst_port     = atoi(argv[i] + 7);
        else if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *c = argv[i] + 8;
            if (!g_ascii_strcasecmp(c, "h265") || !g_ascii_strcasecmp(c, "hevc")) codec = CODEC_H265;
            else codec = CODEC_H264;
        }
    }

    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d\n",
            device, width, height, fps, bitrate_kbps,
            (codec==CODEC_H264?"H.264":"H.265"), dst_host, dst_port);

    // ---- Pipelines ----
    CustomData data = {0};

    // Capture/appsink: DMABuf via io-mode=dmabuf; DO NOT force (memory:DMABuf) in caps
    // No drop at appsink (to avoid starving encoder at 60fps)
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=cv_sink emit-signals=true max-buffers=3 drop=false sync=false",
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

    // Build appsrc/encoder/UDP pipeline (non-leaky small queues to preserve frames)
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

    error = NULL;
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

    // Configure appsrc as LIVE pacer; block to keep frames (avoid drops)
    g_object_set(data.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "block", TRUE,              // block briefly instead of dropping
                 "max-bytes", 4*1024*1024,   // small cushion
                 NULL);

    // Set caps on appsrc directly for stable negotiation
    GstCaps *appsrc_caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width",  G_TYPE_INT,    width,
        "height", G_TYPE_INT,    height,
        "framerate", GST_TYPE_FRACTION, fps, 1, NULL);
    gst_app_src_set_caps(GST_APP_SRC(data.appsrc), appsrc_caps);
    gst_caps_unref(appsrc_caps);

    // Bridge callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Share the same clock across both top-level pipelines
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

    // Attach probe to encoder sink
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

    signal(SIGINT, handle_sigint);
    g_print("Running. Press Ctrl+C to exit.\n");

    while (!g_stop) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000); // 1 ms
    }

    g_print("Stopping...\n");
    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(app_src_pipeline,  GST_STATE_NULL);
    g_main_loop_unref(data.loop);
    gst_object_unref(app_sink_pipeline);
    gst_object_unref(app_src_pipeline);
    return 0;
}
