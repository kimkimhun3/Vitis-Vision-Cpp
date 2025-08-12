#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

std::mutex buffer_mutex;
std::condition_variable buffer_cond;
GstBuffer* global_buffer = nullptr;
std::atomic<bool> new_buffer_available{false};
std::atomic<bool> running{true};

// Callback: appsink receives a buffer
GstFlowReturn on_new_sample_from_sink(GstElement* sink, void* user_data) {
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    GstBuffer* buffer = gst_sample_get_buffer(sample);

    // Deep copy the buffer (appsink may reuse it)
    GstBuffer* buffer_copy = gst_buffer_copy(buffer);

    // Wait for previous buffer to be consumed
    std::unique_lock<std::mutex> lock(buffer_mutex);
    buffer_cond.wait(lock, [] { return !new_buffer_available || !running; });
    
    if (!running) {
        lock.unlock();
        gst_sample_unref(sample);
        gst_buffer_unref(buffer_copy);
        return GST_FLOW_EOS;
    }

    // Set new buffer
    if (global_buffer) {
        gst_buffer_unref(global_buffer);
    }
    global_buffer = buffer_copy;
    new_buffer_available = true;
    lock.unlock();

    // Notify appsrc thread
    buffer_cond.notify_one();

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// Thread function: appsrc pushes buffers
void appsrc_thread(GstElement* appsrc) {
    while (running) {
        std::unique_lock<std::mutex> lock(buffer_mutex);

        // Wait for a new buffer
        buffer_cond.wait(lock, [] { return new_buffer_available.load() || !running.load(); });

        if (!running) break;

        if (global_buffer) {
            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), gst_buffer_ref(global_buffer));
            if (ret != GST_FLOW_OK) {
                g_print("appsrc push failed: %s\n", gst_flow_get_name(ret));
            }

            // Clean up
            gst_buffer_unref(global_buffer);
            global_buffer = nullptr;
            new_buffer_available = false;

            lock.unlock();
            // Notify sink that buffer is consumed
            buffer_cond.notify_one();
        } else {
            lock.unlock();
        }
    }
}

int main(int argc, char* argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);

    // Create pipeline
    GstElement* pipeline = gst_pipeline_new("camera-stream-pipeline");

    // Create elements
    GstElement* v4l2src = gst_element_factory_make("v4l2src", "v4l2-source");
    g_object_set(v4l2src, "device", "/dev/video0", "io-mode", 4, nullptr);

    GstElement* capsfilter_src = gst_element_factory_make("capsfilter", "src-caps");
    GstCaps* src_caps = gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1");
    g_object_set(capsfilter_src, "caps", src_caps, nullptr);
    gst_caps_unref(src_caps);

    GstElement* appsink = gst_element_factory_make("appsink", "sink");
    GstCaps* sink_caps = gst_caps_from_string("video/x-raw,format=NV12");
    g_object_set(appsink, "caps", sink_caps, "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample_from_sink), nullptr);
    gst_caps_unref(sink_caps);

    GstElement* appsrc = gst_element_factory_make("appsrc", "source");
    GstCaps* appsrc_caps = gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1");
    g_object_set(appsrc, "caps", appsrc_caps, "format", GST_FORMAT_TIME, "is-live", TRUE, "do-timestamp", TRUE, nullptr);
    gst_caps_unref(appsrc_caps);

    GstElement* encoder = gst_element_factory_make("omxh265enc", "h265-encoder");
    g_object_set(encoder,
        "num-slices", 8,
        "periodicity-idr", 240,
        "cpb-size", 500,
        "gdr-mode", 2, // horizontal
        "initial-delay", 250,
        "control-rate", 2, // low-latency
        "prefetch-buffer", TRUE,
        "target-bitrate", 8000000, // 8 Mbps
        "gop-mode", 1, // low-delay-p
        nullptr);

    GstElement* rtppay = gst_element_factory_make("rtph265pay", "rtp-pay");
    g_object_set(rtppay, "config-interval", 1, "mtu", 1400, nullptr);

    GstElement* udpsink = gst_element_factory_make("udpsink", "udp-sink");
    g_object_set(udpsink,
        "host", "192.168.25.69",
        "port", 5004,
        "buffer-size", 60000000,
        "async", FALSE,
        "max-lateness", -1,
        "qos-dscp", 60,
        nullptr);

    // Build pipeline
    gst_bin_add_many(GST_BIN(pipeline),
        v4l2src, capsfilter_src, appsink,
        appsrc, encoder, rtppay, udpsink, nullptr);

    // Link: v4l2src → capsfilter → appsink
    if (!gst_element_link_many(v4l2src, capsfilter_src, appsink, nullptr)) {
        g_printerr("Failed to link v4l2src → appsink\n");
        return -1;
    }

    // Link: appsrc → encoder → rtph265pay → udpsink
    if (!gst_element_link_many(appsrc, encoder, rtppay, udpsink, nullptr)) {
        g_printerr("Failed to link appsrc → udpsink\n");
        return -1;
    }

    // Start the appsrc thread
    std::thread src_thread(appsrc_thread, appsrc);

    // Set pipeline to PLAYING
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("Pipeline running...\n");

    // Run main loop
    g_main_loop_run(loop);

    // Cleanup
    running = false;
    buffer_cond.notify_all();
    if (src_thread.joinable()) {
        src_thread.join();
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    if (global_buffer) {
        gst_buffer_unref(global_buffer);
        global_buffer = nullptr;
    }

    return 0;
}