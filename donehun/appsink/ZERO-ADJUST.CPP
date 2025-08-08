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
} CustomData;

// Direct buffer passthrough from appsink to appsrc
// GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
//     CustomData *data = (CustomData *)user_data;
//     GstSample *sample = gst_app_sink_pull_sample(appsink);
//     if (!sample) return GST_FLOW_ERROR;

//     GstBuffer *buffer = gst_sample_get_buffer(sample);
//     if (!buffer) {
//         gst_sample_unref(sample);
//         return GST_FLOW_ERROR;
//     }

//     if (!data->video_info_valid) {
//         GstCaps *caps = gst_sample_get_caps(sample);
//         if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
//             data->video_info_valid = TRUE;
//         } else {
//             gst_sample_unref(sample);
//             return GST_FLOW_ERROR;
//         }
//     }

//     GstBuffer *out_buffer = gst_buffer_copy(buffer);
//     gst_buffer_copy_into(out_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

//     GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), out_buffer);
//     gst_sample_unref(sample);
//     return ret;
// }

GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
        } else {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // 🚫 REMOVE buffer copy
    // ✅ REF buffer to take ownership without copying
    gst_buffer_ref(buffer);

    // Push the original buffer to appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), buffer);

    // Free the sample (but NOT the buffer — it’s ref'd)
    gst_sample_unref(sample);

    return ret;
}



int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    gint bitrate = 8000;  // in kbps
    gint fps = 60;        // default fps

    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate=")) {
            bitrate = atoi(argv[i] + strlen("--bitrate="));
            if (bitrate <= 0) {
                g_printerr("Invalid bitrate value: %s\n", argv[i]);
                return -1;
            }
        } else if (g_str_has_prefix(argv[i], "--fps=")) {
            fps = atoi(argv[i] + strlen("--fps="));
            if (fps <= 0 || fps > 240) {
                g_printerr("Invalid fps value: %s\n", argv[i]);
                return -1;
            }
        }
    }

    g_print("Using FPS: %d, Bitrate: %d kbps\n", fps, bitrate);

    CustomData data = {};
    GError *error = NULL;

    // --- appsink pipeline ---
    gchar *pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true",
        fps);

    GstElement *app_sink_pipeline = gst_parse_launch(pipeline_str, &error);
    g_free(pipeline_str);

    if (!app_sink_pipeline) {
        g_printerr("Failed to create appsink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    data.appsink = gst_bin_get_by_name(GST_BIN(app_sink_pipeline), "cv_sink");

    // --- appsrc pipeline ---

    pipeline_str = g_strdup_printf(
        "appsrc name=cv_src format=GST_FORMAT_TIME is-live=true block=true ! "
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=%d/1 ! "
        "queue ! "
        "omxh265enc skip-frame=1 max-consecutive-skip=4 "
        "gop-mode=low-delay target-bitrate=%d num-slices=4 control-rate=low-latency qp-mode=auto "
        "prefetch-buffer=true cpb-size=500 initial-delay=0 gdr-mode=false "
        "periodicity-idr=60 gop-length=60 filler-data=false ! "
        "video/x-h265,alignment=au,profile=main ! "
        "rtph265pay mtu=1400 ! "
        "queue max-size-buffers=2 ! "
        "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1",
        fps, bitrate);


    GstElement *app_src_pipeline = gst_parse_launch(pipeline_str, &error);
    g_free(pipeline_str);

    if (!app_src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error->message);
        g_clear_error(&error);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    data.appsrc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "cv_src");

    // Link callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Start both pipelines
    gst_element_set_state(app_src_pipeline, GST_STATE_PLAYING);
    gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);

    g_print("Running. Press Ctrl+C to exit.\n");

    // Wait for error or EOS
    GstBus *bus = gst_element_get_bus(app_sink_pipeline);
    gboolean running = TRUE;
    while (running) {
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus, GST_CLOCK_TIME_NONE,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg) {
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = NULL;
                gchar *debug_info = NULL;
                gst_message_parse_error(msg, &err, &debug_info);
                g_printerr("Error: %s\n", err->message);
                g_free(debug_info);
                g_error_free(err);
                running = FALSE;
                break;
            }
            case GST_MESSAGE_EOS:
                g_print("End-Of-Stream reached.\n");
                running = FALSE;
                break;
            default:
                break;
            }
            gst_message_unref(msg);
        }
    }

    // Cleanup
    gst_object_unref(bus);
    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(app_src_pipeline, GST_STATE_NULL);
    gst_object_unref(app_sink_pipeline);
    gst_object_unref(app_src_pipeline);

    return 0;
}
