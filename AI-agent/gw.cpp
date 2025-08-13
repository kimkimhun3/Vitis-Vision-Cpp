#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <thread>
#include <atomic>

// Global pointers
GstElement *pipeline = nullptr;
GstElement *appsrc = nullptr;
GstElement *appsink = nullptr;

std::atomic<bool> running(true);

// Callback function: called when appsink receives a buffer
static GstFlowReturn on_new_sample_from_sink(GstElement *sink, void *user_data) {
    // Extract sample from appsink
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Get caps from sample (important for format info)
    GstCaps *caps = gst_sample_get_caps(sample);
    if (!caps) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Push buffer to appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), gst_buffer_copy(buffer));

    // Unref the sample (not the buffer if copied)
    gst_sample_unref(sample);

    return ret;
}

int main(int argc, char *argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Create pipeline
    pipeline = gst_pipeline_new("video-pipeline");
    if (!pipeline) {
        std::cerr << "Failed to create pipeline" << std::endl;
        return -1;
    }

    // Create elements
    GstElement *source = gst_element_factory_make("v4l2src", "source");
    GstElement *capsfilter_in = gst_element_factory_make("capsfilter", "capsfilter_in");
    GstElement *videoscale = gst_element_factory_make("videoscale", "videoscale");
    GstElement *capsfilter_out = gst_element_factory_make("capsfilter", "capsfilter_out");
    appsink = gst_element_factory_make("appsink", "appsink");
    appsrc = gst_element_factory_make("appsrc", "appsrc");
    GstElement *encoder = gst_element_factory_make("omxh264enc", "encoder");  // Use nvv4l2h264enc on Jetson if needed
    GstElement *rtppay = gst_element_factory_make("rtph264pay", "rtppay");
    GstElement *udpsink = gst_element_factory_make("udpsink", "udpsink");

    if (!source || !capsfilter_in || !videoscale || !capsfilter_out ||
        !appsink || !appsrc || !encoder || !rtppay || !udpsink) {
        std::cerr << "Failed to create one or more elements" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    // Set source capabilities (assume camera outputs 4K NV12 @60fps)
    GstCaps *input_caps = gst_caps_from_string(
        "video/x-raw,format=NV12,width=3840,height=2160,framerate=60/1"
    );
    g_object_set(capsfilter_in, "caps", input_caps, nullptr);
    gst_caps_unref(input_caps);

    // Set output resolution: 1920x1080 NV12 @60fps
    GstCaps *output_caps = gst_caps_from_string(
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1"
    );
    g_object_set(capsfilter_out, "caps", output_caps, nullptr);
    gst_caps_unref(output_caps);

    // Configure appsink
    g_object_set(appsink, "emit-signals", TRUE, nullptr);
    g_object_set(appsink, "sync", FALSE, nullptr);  // Avoid clock blocking
    g_object_set(appsink, "max-buffers", 4, nullptr);  // Limit queue size
    g_object_set(appsink, "drop", TRUE, nullptr);  // Drop old buffers under load
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample_from_sink), nullptr);

    // Configure appsrc
    g_object_set(appsrc, "caps", output_caps, nullptr);
    g_object_set(appsrc, "format", GST_FORMAT_TIME, nullptr);
    g_object_set(appsrc, "is-live", TRUE, nullptr);
    g_object_set(appsrc, "do-timestamp", TRUE, nullptr);

    // Set UDP destination
    g_object_set(udpsink, "host", "192.168.25.69", "port", 5004, nullptr);
    g_object_set(udpsink, "async", FALSE, "sync", FALSE, nullptr);  // Avoid blocking on clock

    // Optional: tune encoder
    g_object_set(encoder, "control-rate", 1, "target-bitrate", 10000, nullptr);  // VBR, 10 Mbps

    // Build pipeline
    gst_bin_add_many(GST_BIN(pipeline),
                     source, capsfilter_in, videoscale, capsfilter_out,
                     appsink, appsrc, encoder, rtppay, udpsink, nullptr);

    // Link elements
    if (!gst_element_link_many(source, capsfilter_in, videoscale, capsfilter_out, appsink, nullptr)) {
        std::cerr << "Failed to link source -> appsink part" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    if (!gst_element_link_many(appsrc, encoder, rtppay, udpsink, nullptr)) {
        std::cerr << "Failed to link appsrc -> udpsink part" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to start pipeline" << std::endl;
        gst_object_unref(pipeline);
        return -1;
    }

    std::cout << "Pipeline running... Streaming 1920x1080@60fps NV12 via UDP to 192.168.25.69:5004\n";
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Run main loop (minimal)
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}