#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>


typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean video_info_valid;
    GstVideoInfo video_info;
} CustomData;

// Direct buffer passthrough from appsink to appsrc
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

    // Make a copy to ensure GStreamer manages the buffer lifetime correctly
    GstBuffer *out_buffer = gst_buffer_copy(buffer);
    gst_buffer_copy_into(out_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), out_buffer);
    gst_sample_unref(sample);
    return ret;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    CustomData data = {};
    GError *error = NULL;

    // Pipeline 1: Camera capture → appsink
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw, format=NV12, width=3840, height=2160, framerate=60/1 ! "
        "appsink name=my_sink emit-signals=true max-buffers=2 drop=true sync=false"
    );
    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");

    // Pipeline 2: appsrc → omxh264enc → rtph264pay → udpsink
    gchar *src_pipeline_str = g_strdup_printf(
        "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
        "video/x-raw,format=NV12,width=3840,height=2160,framerate=60/1 ! "
        "omxh264enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
        "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=20000 "
        "gop-mode=low-delay-p ! video/x-h264, alignment=nal ! "
        "rtph264pay ! "
        "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60"
    );
    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error->message);
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return -1;
    }

    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");

    // Set up appsink callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    gst_element_set_state(src_pipeline, GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    g_print("Running. Press Ctrl+C to exit.\n");

    // Wait for EOS or error
    GstBus *bus = gst_element_get_bus(sink_pipeline);
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
                g_error_free(err);
                g_free(debug_info);
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

    gst_object_unref(bus);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline, GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);

    return 0;
}
