// nv12_relay_h265_min.cpp
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <cstdio>
#include <cstdlib>

struct CustomData {
    GstElement *appsrc = nullptr;
    GstElement *appsink = nullptr;
    gboolean caps_set_on_appsrc = FALSE;
};

/* Bus logger */
static gboolean bus_cb(GstBus *, GstMessage *msg, gpointer tag_ptr) {
    const char *tag = (const char *)tag_ptr;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = nullptr; gchar *dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("[%s] ERROR: %s (debug: %s)\n", tag, err->message, dbg ? dbg : "none");
            g_clear_error(&err); g_free(dbg);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("[%s] EOS\n", tag);
            break;
        default: break;
    }
    return TRUE;
}

/* appsink → appsrc relay (zero-copy, preserve timestamps). Set appsrc caps on first sample. */
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    auto *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    if (!data->caps_set_on_appsrc) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (!caps) {
            g_printerr("Sample had no caps\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
        gst_app_src_set_caps(GST_APP_SRC(data->appsrc), gst_caps_copy(caps));
        data->caps_set_on_appsrc = TRUE;
    }

    GstBuffer *in_buf = gst_sample_get_buffer(sample);
    if (!in_buf) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstBuffer *out_buf = gst_buffer_ref(in_buf); // zero-copy handoff
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), out_buf);
    gst_sample_unref(sample);
    return ret;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    /* Defaults: only --fps and --bitrate are supported */
    gint fps = 60;           // camera is 60; use --fps=30 to drop via videorate
    gint bitrate_kbps = 8000;
    const char *host = "192.168.25.69";
    const gint   port = 5004;

    for (int i = 1; i < argc; ++i) {
        if (g_str_has_prefix(argv[i], "--fps="))       fps = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--bitrate=")) bitrate_kbps = atoi(argv[i] + 10);
    }
    if (fps <= 0 || bitrate_kbps <= 0) {
        g_printerr("Usage: %s --fps=<N> --bitrate=<kbps>\n", argv[0]);
        return 1;
    }
    g_print("Config: fps=%d, bitrate=%d kbps, dst=%s:%d\n", fps, bitrate_kbps, host, port);
    g_print("Note: Set resolution beforehand with media-ctl (4K or 1080p).\n");

    CustomData data;
    GError *err = nullptr;

    /* Capture → (optional) videorate drop → caps(fps) → appsink
       - No width/height here: we take whatever resolution media-ctl configured (4K or 1080p).
       - io-mode=2 (SystemMemory) is robust across the appsink/appsrc boundary.
    */
    gchar *sink_pipeline_str = nullptr;
    if (fps < 60) {
        sink_pipeline_str = g_strdup_printf(
            "v4l2src device=/dev/video0 io-mode=2 ! "
            "video/x-raw,format=NV12,framerate=60/1 ! "
            "videorate drop-only=true max-rate=%d ! "
            "video/x-raw,format=NV12,framerate=%d/1 ! "
            "appsink name=relay_sink emit-signals=true max-buffers=1 drop=true sync=false",
            fps, fps);
    } else {
        sink_pipeline_str = g_strdup(
            "v4l2src device=/dev/video0 io-mode=2 ! "
            "video/x-raw,format=NV12,framerate=60/1 ! "
            "appsink name=relay_sink emit-signals=true max-buffers=1 drop=true sync=false");
    }

    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &err);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Create sink pipeline failed: %s\n", err ? err->message : "unknown");
        if (err) g_clear_error(&err);
        return 1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "relay_sink");

    /* appsrc → tiny leaky queue → H.265 encoder → RTP → UDP
       - appsrc caps set dynamically from first sample (so width/height match media-ctl result)
       - do-timestamp=false to preserve upstream PTS/DTS from v4l2src/videorate
    */
    gchar *src_pipeline_str = g_strdup_printf(
        "appsrc name=relay_src is-live=true format=GST_FORMAT_TIME do-timestamp=false block=false "
        "! queue leaky=2 max-size-buffers=4 "
        "! video/x-raw,format=NV12 "
        "! omxh265enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
        "              initial-delay=250 control-rate=low-latency prefetch-buffer=true "
        "              target-bitrate=%d gop-mode=low-delay-p "
        "! video/x-h265,alignment=au "
        "! rtph265pay pt=96 config-interval=1 "
        "! udpsink buffer-size=60000000 host=%s port=%d async=false qos-dscp=60",
        bitrate_kbps, host, port);

    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &err);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Create src pipeline failed: %s\n", err ? err->message : "unknown");
        if (err) g_clear_error(&err);
        gst_object_unref(sink_pipeline);
        return 1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "relay_src");

    /* Hook relay callback */
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    /* Bus watches */
    GstBus *busA = gst_element_get_bus(sink_pipeline);
    GstBus *busB = gst_element_get_bus(src_pipeline);
    gst_bus_add_watch(busA, bus_cb, (gpointer)"CAPTURE");
    gst_bus_add_watch(busB, bus_cb, (gpointer)"ENCODER");
    gst_object_unref(busA);
    gst_object_unref(busB);

    /* Start pipelines */
    gst_element_set_state(src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    g_print("Running… Ctrl+C to quit\n");
    GMainLoop *loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    /* Cleanup */
    g_main_loop_unref(loop);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline,  GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    return 0;
}
