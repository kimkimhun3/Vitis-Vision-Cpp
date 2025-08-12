#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>
#include <iostream>
#include <memory>
#include <atomic>

class VideoRelay {
private:
    GstElement *source_pipeline;
    GstElement *sink_pipeline;
    GstAppSink *app_sink;
    GstAppSrc *app_src;
    GMainLoop *main_loop;
    std::atomic<bool> caps_negotiated{false};
    std::atomic<guint64> frame_count{0};
    
    // Pipeline configuration
    static constexpr const char* SOURCE_PIPELINE = 
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
        "appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true";
    
    static constexpr const char* SINK_PIPELINE_TEMPLATE = 
        "appsrc name=src is-live=true do-timestamp=false block=true "
        "format=time stream-type=stream max-bytes=0 ! "
        "omxh265enc num-slices=8 periodicity-idr=240 cpb-size=500 "
        "gdr-mode=horizontal initial-delay=250 control-rate=low-latency "
        "prefetch-buffer=true target-bitrate=%d gop-mode=low-delay-p ! "
        "video/x-h264,alignment=nal ! "
        "rtph264pay config-interval=-1 ! "
        "udpsink host=%s port=%d buffer-size=60000000 async=false "
        "max-lateness=-1 qos-dscp=60";

public:
    VideoRelay() : source_pipeline(nullptr), sink_pipeline(nullptr), 
                   app_sink(nullptr), app_src(nullptr), main_loop(nullptr) {}
    
    ~VideoRelay() {
        cleanup();
    }
    
    bool initialize(const std::string& host = "192.168.25.69", 
                   int port = 5004, int bitrate = 8000) {
        
        // Initialize GStreamer
        if (!gst_is_initialized()) {
            gst_init(nullptr, nullptr);
        }
        
        // Create source pipeline
        GError *error = nullptr;
        source_pipeline = gst_parse_launch(SOURCE_PIPELINE, &error);
        if (!source_pipeline || error) {
            std::cerr << "Failed to create source pipeline: " 
                     << (error ? error->message : "Unknown error") << std::endl;
            if (error) g_error_free(error);
            return false;
        }
        
        // Create sink pipeline
        std::string sink_desc = format_string(SINK_PIPELINE_TEMPLATE, 
                                            bitrate, host.c_str(), port);
        sink_pipeline = gst_parse_launch(sink_desc.c_str(), &error);
        if (!sink_pipeline || error) {
            std::cerr << "Failed to create sink pipeline: " 
                     << (error ? error->message : "Unknown error") << std::endl;
            if (error) g_error_free(error);
            return false;
        }
        
        // Get appsink and appsrc elements
        app_sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(source_pipeline), "sink"));
        app_src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(sink_pipeline), "src"));
        
        if (!app_sink || !app_src) {
            std::cerr << "Failed to get app elements" << std::endl;
            return false;
        }
        
        // Configure appsink callbacks
        GstAppSinkCallbacks sink_callbacks = {};
        sink_callbacks.new_sample = &VideoRelay::on_new_sample_static;
        gst_app_sink_set_callbacks(app_sink, &sink_callbacks, this, nullptr);
        
        // Configure appsrc
        g_object_set(G_OBJECT(app_src),
                    "is-live", TRUE,
                    "format", GST_FORMAT_TIME,
                    "do-timestamp", FALSE,
                    nullptr);
        
        // Setup bus monitoring for both pipelines
        setup_bus_monitoring();
        
        std::cout << "Pipeline initialized successfully" << std::endl;
        std::cout << "Target: " << host << ":" << port 
                 << " @ " << bitrate << " kbps" << std::endl;
        
        return true;
    }
    
    bool start() {
        if (!source_pipeline || !sink_pipeline) {
            std::cerr << "Pipelines not initialized" << std::endl;
            return false;
        }
        
        // Start sink pipeline first (to be ready for data)
        GstStateChangeReturn ret = gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to start sink pipeline" << std::endl;
            return false;
        }
        
        // Small delay to ensure sink pipeline is ready
        g_usleep(100000); // 100ms
        
        // Start source pipeline
        ret = gst_element_set_state(source_pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to start source pipeline" << std::endl;
            return false;
        }
        
        std::cout << "Pipelines started successfully" << std::endl;
        return true;
    }
    
    void run() {
        main_loop = g_main_loop_new(nullptr, FALSE);
        
        // Setup signal handlers for graceful shutdown
        setup_signal_handlers();
        
        std::cout << "Running video relay. Press Ctrl+C to stop." << std::endl;
        g_main_loop_run(main_loop);
    }
    
    void stop() {
        if (main_loop && g_main_loop_is_running(main_loop)) {
            g_main_loop_quit(main_loop);
        }
    }

private:
    static GstFlowReturn on_new_sample_static(GstAppSink *sink, gpointer user_data) {
        return static_cast<VideoRelay*>(user_data)->on_new_sample(sink);
    }
    
    GstFlowReturn on_new_sample(GstAppSink *sink) {
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
        
        // Handle caps negotiation on first frame
        if (!caps_negotiated.load()) {
            GstCaps *caps = gst_sample_get_caps(sample);
            if (caps) {
                gst_app_src_set_caps(app_src, caps);
                caps_negotiated.store(true);
                std::cout << "Caps negotiated: " << gst_caps_to_string(caps) << std::endl;
            }
        }
        
        // Create a copy of the buffer for appsrc
        // This ensures proper memory management and avoids reference issues
        GstBuffer *buffer_copy = gst_buffer_copy(buffer);
        
        // Push buffer to appsrc
        GstFlowReturn flow_ret = gst_app_src_push_buffer(app_src, buffer_copy);
        
        // Update statistics
        guint64 current_frame = frame_count.fetch_add(1) + 1;
        if (current_frame % 300 == 0) { // Log every 5 seconds at 60fps
            std::cout << "Processed " << current_frame << " frames" << std::endl;
        }
        
        // Clean up
        gst_sample_unref(sample);
        
        return flow_ret;
    }
    
    void setup_bus_monitoring() {
        // Monitor source pipeline bus
        GstBus *source_bus = gst_element_get_bus(source_pipeline);
        gst_bus_add_watch(source_bus, &VideoRelay::bus_callback_static, this);
        gst_object_unref(source_bus);
        
        // Monitor sink pipeline bus
        GstBus *sink_bus = gst_element_get_bus(sink_pipeline);
        gst_bus_add_watch(sink_bus, &VideoRelay::bus_callback_static, this);
        gst_object_unref(sink_bus);
    }
    
    static gboolean bus_callback_static(GstBus *bus, GstMessage *message, gpointer user_data) {
        return static_cast<VideoRelay*>(user_data)->bus_callback(bus, message);
    }
    
    gboolean bus_callback(GstBus *bus, GstMessage *message) {
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError *error;
                gchar *debug_info;
                gst_message_parse_error(message, &error, &debug_info);
                std::cerr << "Error from " << GST_OBJECT_NAME(message->src) 
                         << ": " << error->message << std::endl;
                if (debug_info) {
                    std::cerr << "Debug info: " << debug_info << std::endl;
                }
                g_error_free(error);
                g_free(debug_info);
                stop();
                break;
            }
            case GST_MESSAGE_EOS:
                std::cout << "End of stream" << std::endl;
                stop();
                break;
            case GST_MESSAGE_WARNING: {
                GError *error;
                gchar *debug_info;
                gst_message_parse_warning(message, &error, &debug_info);
                std::cout << "Warning from " << GST_OBJECT_NAME(message->src) 
                         << ": " << error->message << std::endl;
                g_error_free(error);
                g_free(debug_info);
                break;
            }
            case GST_MESSAGE_STATE_CHANGED: {
                if (GST_MESSAGE_SRC(message) == GST_OBJECT(source_pipeline) ||
                    GST_MESSAGE_SRC(message) == GST_OBJECT(sink_pipeline)) {
                    GstState old_state, new_state, pending_state;
                    gst_message_parse_state_changed(message, &old_state, &new_state, &pending_state);
                    std::cout << "Pipeline " << GST_OBJECT_NAME(message->src) 
                             << " state changed from " << gst_element_state_get_name(old_state)
                             << " to " << gst_element_state_get_name(new_state) << std::endl;
                }
                break;
            }
            default:
                break;
        }
        return TRUE;
    }
    
    void setup_signal_handlers() {
        // This would typically use Unix signal handling
        // For simplicity, we'll just rely on the main loop
    }
    
    void cleanup() {
        if (source_pipeline) {
            gst_element_set_state(source_pipeline, GST_STATE_NULL);
            gst_object_unref(source_pipeline);
            source_pipeline = nullptr;
        }
        
        if (sink_pipeline) {
            gst_element_set_state(sink_pipeline, GST_STATE_NULL);
            gst_object_unref(sink_pipeline);
            sink_pipeline = nullptr;
        }
        
        if (main_loop) {
            g_main_loop_unref(main_loop);
            main_loop = nullptr;
        }
    }
    
    // Helper function to format strings (C++11 compatible)
    template<typename... Args>
    std::string format_string(const char* format, Args... args) {
        int size = snprintf(nullptr, 0, format, args...);
        if (size <= 0) return "";
        
        std::unique_ptr<char[]> buffer(new char[size + 1]);
        snprintf(buffer.get(), size + 1, format, args...);
        return std::string(buffer.get(), size);
    }
};

// Signal handler for graceful shutdown
static VideoRelay* g_relay_instance = nullptr;
static void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_relay_instance) {
        g_relay_instance->stop();
    }
}

int main(int argc, char *argv[]) {
    // Parse command line arguments
    std::string host = "192.168.25.69";
    int port = 5004;
    int bitrate = 8000;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.find("--host=") == 0) {
            host = arg.substr(7);
        } else if (arg.find("--port=") == 0) {
            port = std::stoi(arg.substr(7));
        } else if (arg.find("--bitrate=") == 0) {
            bitrate = std::stoi(arg.substr(10));
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                     << "Options:\n"
                     << "  --host=HOST       Target host (default: 192.168.25.69)\n"
                     << "  --port=PORT       Target port (default: 5004)\n"
                     << "  --bitrate=RATE    Target bitrate in kbps (default: 8000)\n"
                     << "  --help, -h        Show this help\n";
            return 0;
        }
    }
    
    // Setup signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Create and run video relay
    VideoRelay relay;
    g_relay_instance = &relay;
    
    if (!relay.initialize(host, port, bitrate)) {
        std::cerr << "Failed to initialize video relay" << std::endl;
        return 1;
    }
    
    if (!relay.start()) {
        std::cerr << "Failed to start video relay" << std::endl;
        return 1;
    }
    
    relay.run();
    
    std::cout << "Video relay stopped" << std::endl;
    return 0;
}