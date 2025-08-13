#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

class VideoStreamer {
private:
    GstElement *pipeline;
    GstElement *camera_src;
    GstElement *camera_caps;
    GstElement *appsink;
    GstElement *appsrc;
    GstElement *encoder;
    GstElement *encoder_caps;
    GstElement *rtp_payloader;
    GstElement *udp_sink;
    
    GstBus *bus;
    GMainLoop *main_loop;
    
    std::atomic<bool> running{false};
    std::thread bus_thread;
    
    // Performance optimization members
    static constexpr int BUFFER_POOL_SIZE = 10;
    GstBufferPool *buffer_pool;
    
public:
    VideoStreamer() : pipeline(nullptr), main_loop(nullptr), buffer_pool(nullptr) {}
    
    ~VideoStreamer() {
        cleanup();
    }
    
    bool initialize() {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        
        // Create main loop
        main_loop = g_main_loop_new(nullptr, FALSE);
        
        // Create pipeline
        pipeline = gst_pipeline_new("video-streamer");
        if (!pipeline) {
            std::cerr << "Failed to create pipeline" << std::endl;
            return false;
        }
        
        // Create elements
        if (!createElement()) {
            return false;
        }
        
        // Set properties
        if (!setElementProperties()) {
            return false;
        }
        
        // Add elements to pipeline
        gst_bin_add_many(GST_BIN(pipeline),
            camera_src, camera_caps, appsink,
            appsrc, encoder, encoder_caps, 
            rtp_payloader, udp_sink, nullptr);
        
        // Link elements
        if (!linkElements()) {
            return false;
        }
        
        // Set up callbacks
        setupCallbacks();
        
        // Set up bus
        setupBus();
        
        std::cout << "GStreamer pipeline initialized successfully" << std::endl;
        return true;
    }
    
    bool start() {
        if (!pipeline) {
            std::cerr << "Pipeline not initialized" << std::endl;
            return false;
        }
        
        // Start pipeline
        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to start pipeline" << std::endl;
            return false;
        }
        
        running = true;
        
        // Start bus monitoring thread
        bus_thread = std::thread([this]() {
            g_main_loop_run(main_loop);
        });
        
        std::cout << "Video streaming started" << std::endl;
        std::cout << "Streaming to UDP: 192.168.25.69:5004" << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;
        
        return true;
    }
    
    void stop() {
        if (!running) return;
        
        running = false;
        
        // Stop pipeline
        gst_element_set_state(pipeline, GST_STATE_NULL);
        
        // Stop main loop
        if (main_loop) {
            g_main_loop_quit(main_loop);
        }
        
        // Wait for bus thread
        if (bus_thread.joinable()) {
            bus_thread.join();
        }
        
        std::cout << "Video streaming stopped" << std::endl;
    }
    
private:
    bool createElement() {
        // Camera source - using v4l2src for Linux camera capture
        camera_src = gst_element_factory_make("v4l2src", "camera-src");
        if (!camera_src) {
            std::cerr << "Failed to create v4l2src element" << std::endl;
            return false;
        }
        
        // Camera caps filter
        camera_caps = gst_element_factory_make("capsfilter", "camera-caps");
        
        // App elements
        appsink = gst_element_factory_make("appsink", "app-sink");
        appsrc = gst_element_factory_make("appsrc", "app-src");
        
        // Encoder - using omxh264enc on ZCU106 board
        encoder = gst_element_factory_make("omxh264enc", "encoder");
        if (!encoder) {
            std::cerr << "Failed to create omxh264enc encoder" << std::endl;
            return false;
        }
        
        // Encoder caps filter
        encoder_caps = gst_element_factory_make("capsfilter", "encoder-caps");
        
        // RTP payloader
        rtp_payloader = gst_element_factory_make("rtph264pay", "rtp-pay");
        
        // UDP sink
        udp_sink = gst_element_factory_make("udpsink", "udp-sink");
        
        if (!camera_caps || !appsink || !appsrc || !encoder_caps || !rtp_payloader || !udp_sink) {
            std::cerr << "Failed to create one or more elements" << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool setElementProperties() {
        // Camera source properties
        g_object_set(camera_src, "device", "/dev/video0", nullptr);
        
        // Camera caps - capture at 1920x1080@60fps NV12
        GstCaps *camera_caps_filter = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "NV12",
            "width", G_TYPE_INT, 1920,
            "height", G_TYPE_INT, 1080,
            "framerate", GST_TYPE_FRACTION, 60, 1,
            nullptr);
        g_object_set(camera_caps, "caps", camera_caps_filter, nullptr);
        gst_caps_unref(camera_caps_filter);
        
        // Appsink properties for optimal performance
        g_object_set(appsink,
            "emit-signals", TRUE,
            "sync", FALSE,
            "max-buffers", 2,  // Keep buffer count low to reduce latency
            "drop", TRUE,      // Drop buffers if downstream is slow
            nullptr);
        
        // Appsrc properties
        g_object_set(appsrc,
            "caps", gst_caps_new_simple("video/x-raw",
                "format", G_TYPE_STRING, "NV12",
                "width", G_TYPE_INT, 1920,
                "height", G_TYPE_INT, 1080,
                "framerate", GST_TYPE_FRACTION, 60, 1,
                nullptr),
            "format", GST_FORMAT_TIME,
            "is-live", TRUE,
            "do-timestamp", TRUE,
            "max-bytes", 0,    // No limit on bytes
            "block", FALSE,    // Non-blocking mode
            nullptr);
        
        // Encoder properties for omxh264enc on ZCU106
        g_object_set(encoder,
            "target-bitrate", 10000,      // 8 Mbps
            "control-rate", 2,       // Variable bitrate
            "preset-level", 2,       // High performance preset
            nullptr);
        
        // Encoder caps
        GstCaps *encoder_caps_filter = gst_caps_new_simple("video/x-h264",
            "profile", G_TYPE_STRING, "high",
            "level", G_TYPE_STRING, "4.0",
            nullptr);
        g_object_set(encoder_caps, "caps", encoder_caps_filter, nullptr);
        gst_caps_unref(encoder_caps_filter);
        
        // RTP payloader properties
        g_object_set(rtp_payloader,
            "config-interval", 1,        // Send SPS/PPS every second
            "pt", 96,                    // Payload type
            nullptr);
        
        // UDP sink properties
        g_object_set(udp_sink,
            "clients", "192.168.25.69:5004",
            "sync", FALSE,
            "async", FALSE,
            nullptr);
        
        return true;
    }
    
    bool linkElements() {
        // Link camera source chain
        if (!gst_element_link_many(camera_src, camera_caps, appsink, nullptr)) {
            std::cerr << "Failed to link camera source elements" << std::endl;
            return false;
        }
        
        // Link encoder chain
        if (!gst_element_link_many(appsrc, encoder, encoder_caps, rtp_payloader, udp_sink, nullptr)) {
            std::cerr << "Failed to link encoder elements" << std::endl;
            return false;
        }
        
        return true;
    }
    
    void setupCallbacks() {
        // Set up appsink callback for high performance
        GstAppSinkCallbacks callbacks = {};
        callbacks.new_sample = onNewSample;
        callbacks.eos = onEOS;
        
        gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &callbacks, this, nullptr);
    }
    
    void setupBus() {
        bus = gst_element_get_bus(pipeline);
        gst_bus_add_watch(bus, busCallback, this);
    }
    
    // High-performance callback function
    static GstFlowReturn onNewSample(GstAppSink *sink, gpointer user_data) {
        VideoStreamer *self = static_cast<VideoStreamer*>(user_data);
        return self->processNewSample(sink);
    }
    
    GstFlowReturn processNewSample(GstAppSink *sink) {
        // Pull sample from appsink
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            return GST_FLOW_ERROR;
        }
        
        // Get buffer from sample
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
        
        // Create a copy of the buffer for appsrc (to avoid conflicts)
        GstBuffer *buffer_copy = gst_buffer_copy(buffer);
        
        // Push buffer to appsrc
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer_copy);
        
        // Cleanup
        gst_sample_unref(sample);
        
        // Handle flow return
        if (ret != GST_FLOW_OK) {
            if (ret == GST_FLOW_FLUSHING) {
                // Pipeline is shutting down, this is normal
                return GST_FLOW_OK;
            }
            std::cerr << "Error pushing buffer to appsrc: " << ret << std::endl;
            return ret;
        }
        
        return GST_FLOW_OK;
    }
    
    static void onEOS(GstAppSink *sink, gpointer user_data) {
        VideoStreamer *self = static_cast<VideoStreamer*>(user_data);
        std::cout << "EOS received from appsink" << std::endl;
        
        // Send EOS to appsrc
        gst_app_src_end_of_stream(GST_APP_SRC(self->appsrc));
    }
    
    static gboolean busCallback(GstBus *bus, GstMessage *message, gpointer user_data) {
        VideoStreamer *self = static_cast<VideoStreamer*>(user_data);
        return self->processBusMessage(message);
    }
    
    gboolean processBusMessage(GstMessage *message) {
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError *error;
                gchar *debug;
                gst_message_parse_error(message, &error, &debug);
                std::cerr << "Error: " << error->message << std::endl;
                std::cerr << "Debug: " << debug << std::endl;
                g_error_free(error);
                g_free(debug);
                g_main_loop_quit(main_loop);
                break;
            }
            case GST_MESSAGE_EOS:
                std::cout << "End of stream" << std::endl;
                g_main_loop_quit(main_loop);
                break;
            case GST_MESSAGE_STATE_CHANGED: {
                if (GST_MESSAGE_SRC(message) == GST_OBJECT(pipeline)) {
                    GstState old_state, new_state;
                    gst_message_parse_state_changed(message, &old_state, &new_state, nullptr);
                    std::cout << "Pipeline state changed from " 
                              << gst_element_state_get_name(old_state) 
                              << " to " << gst_element_state_get_name(new_state) << std::endl;
                }
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError *error;
                gchar *debug;
                gst_message_parse_warning(message, &error, &debug);
                std::cerr << "Warning: " << error->message << std::endl;
                g_error_free(error);
                g_free(debug);
                break;
            }
            default:
                break;
        }
        return TRUE;
    }
    
    void cleanup() {
        if (running) {
            stop();
        }
        
        if (bus) {
            gst_object_unref(bus);
            bus = nullptr;
        }
        
        if (pipeline) {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        
        if (main_loop) {
            g_main_loop_unref(main_loop);
            main_loop = nullptr;
        }
        
        gst_deinit();
    }
};

// Signal handler for graceful shutdown
VideoStreamer *g_streamer = nullptr;
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_streamer) {
        g_streamer->stop();
    }
}

int main() {
    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    VideoStreamer streamer;
    g_streamer = &streamer;
    
    if (!streamer.initialize()) {
        std::cerr << "Failed to initialize video streamer" << std::endl;
        return -1;
    }
    
    if (!streamer.start()) {
        std::cerr << "Failed to start video streaming" << std::endl;
        return -1;
    }
    
    // Keep main thread alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}