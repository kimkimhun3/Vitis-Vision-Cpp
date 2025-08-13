#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <iostream>

static GMainLoop *loop = nullptr;

// Callback to handle new samples from appsink
static GstFlowReturn new_sample(GstAppSink *sink, gpointer user_data) {
    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstAppSrc *appsrc = GST_APP_SRC(user_data);

        // Push the buffer to appsrc without modification
        GstFlowReturn ret = gst_app_src_push_buffer(appsrc, gst_buffer_ref(buffer));
        if (ret != GST_FLOW_OK) {
            std::cerr << "Failed to push buffer to appsrc: " << ret << std::endl;
        }

        gst_sample_unref(sample);
    }
    return GST_FLOW_OK;
}

// Callback for appsrc's "need-data" signal
static void start_feed(GstAppSrc *source, guint size, gpointer user_data) {
    // No action needed; appsink drives the data flow
}

// Callback for appsrc's "enough-data" signal
static void stop_feed(GstAppSrc *source, gpointer user_data) {
    // No action needed
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // Create the main loop
    loop = g_main_loop_new(nullptr, FALSE);

    // Create the pipeline elements
    GstElement *pipeline = gst_pipeline_new("video-pipeline");
    GstElement *src = gst_element_factory_make("v4l2src", "source");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    GstElement *appsink = gst_element_factory_make("appsink", "sink");
    GstElement *appsrc = gst_element_factory_make("appsrc", "appsrc");
    GstElement *encoder = gst_element_factory_make("x264enc", "encoder");
    GstElement *rtppay = gst_element_factory_make("rtph264pay", "rtppay");
    GstElement *udpsink = gst_element_factory_make("udpsink", "udpsink");

    if (!pipeline || !src || !capsfilter || !appsink || !appsrc || !encoder || !rtppay || !udpsink) {
        std::cerr << "Failed to create one or more elements" << std::endl;
        return -1;
    }

    // Configure source
    g_object_set(src, "device", "/dev/video0", nullptr);

    // Configure capsfilter
    GstCaps *caps = gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1");
    g_object_set(capsfilter, "caps", caps, nullptr);
    gst_caps_unref(caps);

    // Configure appsink
    g_object_set(appsink, "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample), appsrc);

    // Configure appsrc
    g_object_set(appsrc, "caps", gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1"),
                 "stream-type", GST_APP_STREAM_TYPE_STREAM, "is-live", TRUE, nullptr);
    g_signal_connect(appsrc, "need-data", G_CALLBACK(start_feed), nullptr);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(stop_feed), nullptr);

    // Configure encoder (using x264enc for broader compatibility)
    g_object_set(encoder, "speed-preset", 1, "bitrate", 8000, "tune", 4, nullptr); // ultrafast, 8Mbps, zerolatency

    // Configure rtph264pay
    g_object_set(rtppay, "pt", 96, nullptr);

    // Configure udpsink
    g_object_set(udpsink, "host", "192.168.25.69", "port", 5004, "async", FALSE, "qos-dscp", 60, nullptr);

    // Create two bins: one for capture, one for encoding/streaming
    GstBin *capture_bin = GST_BIN(gst_bin_new("capture-bin"));
    GstBin *stream_bin = GST_BIN(gst_bin_new("stream-bin"));
8
    gst_bin_add_many(capture_bin, src, capsfilter, appsink, nullptr);
    gst_bin_add_many(stream_bin, appsrc, encoder, rtppay, udpsink, nullptr);

    // Link elements in capture bin
    if (!gst_element_link_many(src, capsfilter, appsink, nullptr)) {
        std::cerr << "Failed to link capture bin elements" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    // Link elements in stream bin
    if (!gst_element_link_many(appsrc, encoder, rtppay, udpsink, nullptr)) {
        std::cerr << "Failed to link stream bin elements" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    // Add bins to pipeline
    gst_bin_add_many(GST_BIN(pipeline), GST_ELEMENT(capture_bin), GST_ELEMENT(stream_bin), nullptr);

    // Set pipeline to playing
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Unable to set the pipeline to the playing state" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    // Run the main loop
    g_main_loop_run(loop);

    // Cleanup
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    gst_deinit();

    return 0;
}