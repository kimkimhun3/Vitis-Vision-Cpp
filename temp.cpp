#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean video_info_valid;
    GstVideoInfo video_info;
    gboolean caps_set_on_appsrc;
    gint desired_fps;
} CustomData;

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *in_buf = gst_sample_get_buffer(sample);
    if (!in_buf) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;

            // Build appsrc caps to MATCH the post-videorate format but with desired FPS
            if (!data->caps_set_on_appsrc) {
                GstStructure *s = gst_caps_get_structure(caps, 0);
                int w = 0, h = 0;
                gst_structure_get_int(s, "width", &w);
                gst_structure_get_int(s, "height", &h);

                // desired_fps/1
                GstCaps *out_caps = gst_caps_new_simple(
                    "video/x-raw",
                    "format", G_TYPE_STRING, "NV12",
                    "width",  G_TYPE_INT,    w,
                    "height", G_TYPE_INT,    h,
                    "framerate", GST_TYPE_FRACTION, data->desired_fps, 1,
                    NULL);
                gst_app_src_set_caps(GST_APP_SRC(data->appsrc), out_caps);
                gst_caps_unref(out_caps);

                data->caps_set_on_appsrc = TRUE;
            }
        } else {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Zero-copy handoff
    GstBuffer *out_buf = gst_buffer_ref(in_buf);
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), out_buf);
    gst_sample_unref(sample);
    return ret;
}

static void attach_bus_watcher(GstElement *pipe, const char *name) {
    GstBus *bus = gst_element_get_bus(pipe);
    gst_bus_add_watch(bus, (GstBusFunc) [](GstBus *b, GstMessage *msg, gpointer u) -> gboolean {
        const char *tag = (const char *)u;
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = NULL; gchar *dbg = NULL;
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
    }, (gpointer)name);
    gst_object_unref(bus);
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    gint target_bitrate = 8000; // kbps
    gint desired_fps = 60;      // default

    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate=")) {
            const gchar *value = argv[i] + strlen("--bitrate=");
            target_bitrate = atoi(value);
        } else if (g_str_has_prefix(argv[i], "--fps=")) {
            const gchar *value = argv[i] + strlen("--fps=");
            desired_fps = atoi(value);
        }
    }
    if (target_bitrate <= 0 || desired_fps <= 0) {
        g_printerr("Usage: %s [--bitrate=kbps] [--fps=N]\n", argv[0]);
        return -1;
    }
    g_print("Using target bitrate: %d kbps, desired FPS: %d\n", target_bitrate, desired_fps);

    CustomData data = {0};
    data.desired_fps = desired_fps;
    GError *error = NULL;

    // A) Capture -> videorate (drop to desired_fps) -> appsink
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=my_sink emit-signals=true max-buffers=2 drop=true sync=false",
        desired_fps);
    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");

    // B) appsrc (advertise desired_fps) -> tiny queue -> encoder -> pay -> udp
    gchar *src_pipeline_str = g_strdup_printf(
        "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=false block=true "
        "! queue leaky=2 max-size-buffers=2 "
        "! video/x-raw,format=NV12,width=1920,height=1080,framerate=%d/1 "
        "! omxh264enc num-slices=4 periodicity-idr=120 cpb-size=200 gdr-mode=horizontal "
        "  initial-delay=100 control-rate=low-latency prefetch-buffer=true target-bitrate=%d gop-mode=low-delay-p "
        "! video/x-h264,alignment=nal "
        "! rtph264pay pt=96 config-interval=1 "
        "! udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
        desired_fps, target_bitrate);
    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error->message);
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return -1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");

    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    gst_element_set_state(src_pipeline, GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    attach_bus_watcher(sink_pipeline, "CAPTURE");
    attach_bus_watcher(src_pipeline,  "ENCODER");

    g_print("Running. Ctrl+C to exit.\n");
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);

    g_main_loop_unref(loop);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline, GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    return 0;
}
