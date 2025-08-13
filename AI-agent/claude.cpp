#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

class VideoStreamer {
private:
    GstElement *pipeline;
    GstElement *camera_source;
    GstElement *caps_filter1;
    GstElement *app_sink;
    GstElement *app_src;
    GstElement *h264_encoder;
    GstElement *caps_filter2;
    GstElement *rtp_payloader;
    GstElement *udp_sink;
    
    GstBus *bus;
    GMainLoop *loop;
    
    // Threading and synchronization
    std::atomic<bool> running;
    std::queue<GstBuffer*> buffer_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::thread processing_thread;
    
    // Performance monitoring
    std::atomic<uint64_t> frames_processed;
    std::atomic<uint64_t> frames_dropped;
    std::chrono::steady_clock::time_point start_time;
    
    static const size_t MAX_QUEUE_SIZE = 5; // Reduced queue size for better performance
    static const uint64_t TARGET_BITRATE = 10000000; // 10 Mbps

public:
    VideoStreamer() : pipeline(nullptr), running(false), frames_processed(0), frames_dropped(0) {
        gst_init(nullptr, nullptr);
    }
    
    ~VideoStreamer() {
        cleanup();
    }
    
    bool initialize() {
        // Create pipeline
        pipeline = gst_pipeline_new("video-streamer");
        if (!pipeline) {
            std::cerr << "Failed to create pipeline" << std::endl;
            return false;
        }
        
        // Create elements
        camera_source = gst_element_factory_make("v4l2src", "camera-source");
        caps_filter1 = gst_element_factory_make("capsfilter", "caps-filter1");
        app_sink = gst_element_factory_make("appsink", "app-sink");
        app_src = gst_element_factory_make("appsrc", "app-src");
        h264_encoder = gst_element_factory_make("omxh264enc", "h264-encoder");
        caps_filter2 = gst_element_factory_make("capsfilter", "caps-filter2");
        rtp_payloader = gst_element_factory_make("rtph264pay", "rtp-payloader");
        udp_sink = gst_element_factory_make("udpsink", "udp-sink");
        
        if (!camera_source || !caps_filter1 || !app_sink || !app_src || 
            !h264_encoder || !caps_filter2 || !rtp_payloader || !udp_sink) {
            std::cerr << "Failed to create one or more elements" << std::endl;
            return false;
        }
        
        // Configure camera source - match your working pipeline
        g_object_set(camera_source, 
            "device", "/dev/video0",
            "io-mode", 4,  // GST_V4L2_IO_DMABUF_IMPORT for better performance
            nullptr);
        
        // Configure input caps filter for 1920x1080@60fps NV12
        GstCaps *input_caps = gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1");
        g_object_set(caps_filter1, "caps", input_caps, nullptr);
        gst_caps_unref(input_caps);
        
        // Configure appsink - optimized for high performance
        g_object_set(app_sink,
            "emit-signals", TRUE,
            "sync", FALSE,
            "async", FALSE,
            "drop", TRUE,
            "max-buffers", 2,  // Minimal buffering
            "wait-on-eos", FALSE,
            nullptr);
        
        // Set appsink caps to ensure format consistency
        GstCaps *sink_caps = gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1");
        gst_app_sink_set_caps(GST_APP_SINK(app_sink), sink_caps);
        gst_caps_unref(sink_caps);
        
        // Configure appsrc - match the input stream characteristics
        GstCaps *src_caps = gst_caps_from_string("video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1");
        g_object_set(app_src,
            "caps", src_caps,
            "format", GST_FORMAT_TIME,
            "is-live", TRUE,
            "do-timestamp", TRUE,
            "min-latency", 0,
            "max-latency", 0,
            "block", FALSE,
            "stream-type", GST_APP_STREAM_TYPE_STREAM,
            nullptr);
        gst_caps_unref(src_caps);
        
        // Configure H.264 encoder - step by step to identify issues
        // Start with basic settings first
        g_object_set(h264_encoder,
            "target-bitrate", TARGET_BITRATE / 1000,  // 10000 kbps
            "control-rate", 2,  // constant bitrate (more compatible than low-latency)
            nullptr);
        
        // Add advanced settings one by one with error checking
        GValue val = G_VALUE_INIT;
        
        // Try to set periodicity-idr
        g_value_init(&val, G_TYPE_UINT);
        g_value_set_uint(&val, 240);
        g_object_set_property(G_OBJECT(h264_encoder), "periodicity-idr", &val);
        g_value_unset(&val);
        
        // Try to set num-slices
        g_value_init(&val, G_TYPE_UINT);
        g_value_set_uint(&val, 8);
        g_object_set_property(G_OBJECT(h264_encoder), "num-slices", &val);
        g_value_unset(&val);
        
        // Try other settings with fallbacks
        g_object_set(h264_encoder,
            "prefetch-buffer", TRUE,
            nullptr);
            
        // Optional advanced settings - only set if supported
        GObjectClass *encoder_class = G_OBJECT_GET_CLASS(h264_encoder);
        if (g_object_class_find_property(encoder_class, "cpb-size")) {
            g_object_set(h264_encoder, "cpb-size", 500, nullptr);
        }
        if (g_object_class_find_property(encoder_class, "initial-delay")) {
            g_object_set(h264_encoder, "initial-delay", 250, nullptr);
        }
        if (g_object_class_find_property(encoder_class, "gdr-mode")) {
            g_object_set(h264_encoder, "gdr-mode", 1, nullptr);  // horizontal
        }
        if (g_object_class_find_property(encoder_class, "gop-mode")) {
            g_object_set(h264_encoder, "gop-mode", 1, nullptr);  // low-delay-p
        }
        
        // Configure H.264 caps filter
        GstCaps *h264_caps = gst_caps_from_string("video/x-h264,alignment=nal");
        g_object_set(caps_filter2, "caps", h264_caps, nullptr);
        gst_caps_unref(h264_caps);
        
        // Configure RTP payloader
        g_object_set(rtp_payloader,
            "pt", 96,
            "config-interval", -1,  // Send SPS/PPS with every IDR
            nullptr);
        
        // Configure UDP sink - match your working pipeline
        g_object_set(udp_sink,
            "buffer-size", 60000000,
            "host", "192.168.25.69",
            "port", 5004,
            "async", FALSE,
            "max-lateness", -1,
            "qos-dscp", 60,
            "sync", FALSE,
            nullptr);
        
        // Add elements to pipeline
        gst_bin_add_many(GST_BIN(pipeline),
            camera_source, caps_filter1, app_sink,
            app_src, h264_encoder, caps_filter2, rtp_payloader, udp_sink,
            nullptr);
        
        // Link capture pipeline
        if (!gst_element_link_many(camera_source, caps_filter1, app_sink, nullptr)) {
            std::cerr << "Failed to link capture pipeline" << std::endl;
            return false;
        }
        
        // Link streaming pipeline
        if (!gst_element_link_many(app_src, h264_encoder, caps_filter2, 
                                   rtp_payloader, udp_sink, nullptr)) {
            std::cerr << "Failed to link streaming pipeline" << std::endl;
            return false;
        }
        
        // Connect appsink callback
        g_signal_connect(app_sink, "new-sample", G_CALLBACK(on_new_sample), this);
        
        // Setup bus
        bus = gst_element_get_bus(pipeline);
        
        return true;
    }
    
    // Ultra high-performance callback - minimal processing
    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data) {
        VideoStreamer *streamer = static_cast<VideoStreamer*>(user_data);
        
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            return GST_FLOW_ERROR;
        }
        
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
        
        // Create a reference to the buffer (avoid copying)
        GstBuffer *buffer_ref = gst_buffer_ref(buffer);
        
        // Fast, non-blocking queue operation
        bool queued = false;
        {
            std::unique_lock<std::mutex> lock(streamer->queue_mutex, std::try_to_lock);
            if (lock.owns_lock()) {
                // Drop frames if queue is full to maintain real-time performance
                while (streamer->buffer_queue.size() >= MAX_QUEUE_SIZE) {
                    GstBuffer *old_buffer = streamer->buffer_queue.front();
                    streamer->buffer_queue.pop();
                    gst_buffer_unref(old_buffer);
                    streamer->frames_dropped++;
                }
                
                streamer->buffer_queue.push(buffer_ref);
                queued = true;
            }
        }
        
        if (queued) {
            streamer->queue_cv.notify_one();
            streamer->frames_processed++;
        } else {
            // Drop frame if we couldn't queue it
            gst_buffer_unref(buffer_ref);
            streamer->frames_dropped++;
        }
        
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    void processing_loop() {
        GstClockTime base_time = GST_CLOCK_TIME_NONE;
        GstClockTime frame_duration = gst_util_uint64_scale_int(GST_SECOND, 1, 60); // 60 fps
        guint64 frame_count = 0;
        
        while (running) {
            GstBuffer *buffer = nullptr;
            
            // Get buffer from queue with timeout
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                if (queue_cv.wait_for(lock, std::chrono::milliseconds(33), 
                                     [this] { return !buffer_queue.empty() || !running; })) {
                    if (!running) break;
                    
                    if (!buffer_queue.empty()) {
                        buffer = buffer_queue.front();
                        buffer_queue.pop();
                    }
                }
            }
            
            if (buffer) {
                // Set proper timestamps for constant frame rate
                if (base_time == GST_CLOCK_TIME_NONE) {
                    base_time = gst_element_get_base_time(pipeline);
                    if (base_time == GST_CLOCK_TIME_NONE) {
                        base_time = 0;
                    }
                }
                
                GstClockTime pts = base_time + frame_count * frame_duration;
                GstClockTime dts = pts;
                
                GST_BUFFER_PTS(buffer) = pts;
                GST_BUFFER_DTS(buffer) = dts;
                GST_BUFFER_DURATION(buffer) = frame_duration;
                
                // Push buffer to appsrc with error handling
                GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(app_src), buffer);
                if (ret != GST_FLOW_OK) {
                    if (ret == GST_FLOW_FLUSHING) {
                        // Normal during shutdown
                        break;
                    } else {
                        std::cerr << "Failed to push buffer to appsrc: " << gst_flow_get_name(ret) << std::endl;
                    }
                }
                
                frame_count++;
            } else {
                // Timeout occurred, check if we should continue
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
        // Signal EOS to appsrc
        gst_app_src_end_of_stream(GST_APP_SRC(app_src));
        
        std::cout << "Processing loop ended. Total frames: " << frame_count << std::endl;
    }
    
    bool start() {
        if (!pipeline) {
            std::cerr << "Pipeline not initialized" << std::endl;
            return false;
        }
        
        running = true;
        start_time = std::chrono::steady_clock::now();
        frames_processed = 0;
        frames_dropped = 0;
        
        // Start processing thread with high priority
        processing_thread = std::thread(&VideoStreamer::processing_loop, this);
        
        // Try to set thread priority (optional - don't fail if not possible)
        try {
            pthread_t native_handle = processing_thread.native_handle();
            struct sched_param param;
            param.sched_priority = 10;
            int result = pthread_setschedparam(native_handle, SCHED_FIFO, &param);
            if (result != 0) {
                std::cout << "Note: Could not set high thread priority (run as root for better performance)" << std::endl;
            }
        } catch (...) {
            std::cout << "Note: Thread priority setting not available" << std::endl;
        }
        
        // Set pipeline to playing state with better error handling
        std::cout << "Setting pipeline to READY state..." << std::endl;
        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_READY);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to set pipeline to ready state" << std::endl;
            
            // Try to get more detailed error information
            GstMessage *msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
            if (msg) {
                GError *err;
                gchar *debug_info;
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "Detailed error: " << err->message << std::endl;
                if (debug_info) {
                    std::cerr << "Debug info: " << debug_info << std::endl;
                }
                g_error_free(err);
                g_free(debug_info);
                gst_message_unref(msg);
            }
            
            running = false;
            return false;
        }
        
        // Wait for READY state
        GstState state;
        ret = gst_element_get_state(pipeline, &state, nullptr, 5 * GST_SECOND);
        if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_READY) {
            std::cerr << "Pipeline failed to reach ready state. Current state: " 
                      << gst_element_state_get_name(state) << std::endl;
            running = false;
            return false;
        }
        
        std::cout << "Setting pipeline to PLAYING state..." << std::endl;
        ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to set pipeline to playing state" << std::endl;
            
            // Try to get more detailed error information
            GstMessage *msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
            if (msg) {
                GError *err;
                gchar *debug_info;
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "Detailed error: " << err->message << std::endl;
                if (debug_info) {
                    std::cerr << "Debug info: " << debug_info << std::endl;
                }
                g_error_free(err);
                g_free(debug_info);
                gst_message_unref(msg);
            }
            
            running = false;
            return false;
        }
        
        // Wait for pipeline to reach playing state with timeout
        ret = gst_element_get_state(pipeline, &state, nullptr, 10 * GST_SECOND);
        if (ret != GST_STATE_CHANGE_SUCCESS || state != GST_STATE_PLAYING) {
            std::cerr << "Pipeline failed to reach playing state within timeout. Current state: " 
                      << gst_element_state_get_name(state) << std::endl;
            std::cerr << "State change return: " << ret << std::endl;
            running = false;
            return false;
        }
        
        std::cout << "Video streaming started successfully!" << std::endl;
        std::cout << "Capturing 1920x1080@60fps NV12 -> H.264 @ " << TARGET_BITRATE/1000000 << "Mbps -> UDP 192.168.25.69:5004" << std::endl;
        
        // Create main loop with proper initialization
        loop = g_main_loop_new(nullptr, FALSE);
        if (!loop) {
            std::cerr << "Failed to create main loop" << std::endl;
            running = false;
            return false;
        }
        
        return true;
    }
    
    void run() {
        if (!loop) return;
        
        // Start performance monitoring thread
        std::thread monitor_thread([this]() {
            while (running) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                if (running) {
                    auto now = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
                    if (duration > 0) {
                        double fps_processed = static_cast<double>(frames_processed) / duration;
                        double drop_rate = static_cast<double>(frames_dropped) * 100.0 / 
                                         (frames_processed + frames_dropped + 0.001);
                        
                        std::cout << "Performance: " << fps_processed << " fps processed, "
                                  << "Queue size: " << buffer_queue.size() 
                                  << ", Drop rate: " << drop_rate << "%" << std::endl;
                    }
                }
            }
        });
        
        // Handle bus messages
        std::thread bus_thread([this]() {
            while (running) {
                GstMessage *msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                    static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | 
                                              GST_MESSAGE_WARNING | GST_MESSAGE_STATE_CHANGED));
                
                if (msg) {
                    handle_message(msg);
                    gst_message_unref(msg);
                }
            }
        });
        
        // Run main loop
        g_main_loop_run(loop);
        
        // Cleanup threads
        if (monitor_thread.joinable()) {
            monitor_thread.join();
        }
        if (bus_thread.joinable()) {
            bus_thread.join();
        }
    }
    
    void stop() {
        if (!running) return;
        
        std::cout << "Stopping video streamer..." << std::endl;
        running = false;
        
        // Stop pipeline
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }
        
        // Wake up processing thread
        queue_cv.notify_all();
        
        if (processing_thread.joinable()) {
            processing_thread.join();
        }
        
        if (loop) {
            g_main_loop_quit(loop);
        }
        
        // Clear remaining buffers in queue
        std::lock_guard<std::mutex> lock(queue_mutex);
        while (!buffer_queue.empty()) {
            gst_buffer_unref(buffer_queue.front());
            buffer_queue.pop();
        }
        
        // Print final statistics
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (duration > 0) {
            double avg_fps = static_cast<double>(frames_processed) / duration;
            double total_drop_rate = static_cast<double>(frames_dropped) * 100.0 / 
                                   (frames_processed + frames_dropped + 0.001);
            std::cout << "Final stats: " << avg_fps << " fps average, " 
                      << total_drop_rate << "% frames dropped" << std::endl;
        }
    }
    
private:
    void handle_message(GstMessage *msg) {
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err;
                gchar *debug_info;
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "Error from " << GST_OBJECT_NAME(msg->src) << ": " << err->message << std::endl;
                if (debug_info) {
                    std::cerr << "Debug info: " << debug_info << std::endl;
                }
                g_error_free(err);
                g_free(debug_info);
                stop();
                break;
            }
            case GST_MESSAGE_EOS:
                std::cout << "End of stream" << std::endl;
                stop();
                break;
            case GST_MESSAGE_WARNING: {
                GError *err;
                gchar *debug_info;
                gst_message_parse_warning(msg, &err, &debug_info);
                std::cout << "Warning from " << GST_OBJECT_NAME(msg->src) << ": " << err->message << std::endl;
                if (debug_info) {
                    std::cout << "Debug info: " << debug_info << std::endl;
                }
                g_error_free(err);
                g_free(debug_info);
                break;
            }
            case GST_MESSAGE_STATE_CHANGED: {
                if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline)) {
                    GstState old_state, new_state, pending_state;
                    gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
                    std::cout << "Pipeline state changed from " << gst_element_state_get_name(old_state)
                              << " to " << gst_element_state_get_name(new_state) << std::endl;
                }
                break;
            }
            default:
                break;
        }
    }
    
    void cleanup() {
        stop();
        
        if (bus) {
            gst_object_unref(bus);
            bus = nullptr;
        }
        
        if (pipeline) {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        
        if (loop && g_main_loop_get_context(loop)) {
            g_main_loop_unref(loop);
            loop = nullptr;
        }
    }
};

// Signal handler for graceful shutdown
VideoStreamer *g_streamer = nullptr;

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    if (g_streamer) {
        g_streamer->stop();
    }
}

int main(int argc, char *argv[]) {
    // Install signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "GStreamer High-Performance Video Streamer" << std::endl;
    std::cout << "Optimized for 1920x1080@60fps NV12 -> H.264 @ 10Mbps" << std::endl;
    std::cout << "Based on working pipeline configuration" << std::endl;
    std::cout << "Streaming to 192.168.25.69:5004" << std::endl;
    
    VideoStreamer streamer;
    g_streamer = &streamer;
    
    if (!streamer.initialize()) {
        std::cerr << "Failed to initialize video streamer" << std::endl;
        return -1;
    }
    
    if (!streamer.start()) {
        std::cerr << "Failed to start video streamer" << std::endl;
        return -1;
    }
    
    streamer.run();
    
    std::cout << "Video streamer stopped" << std::endl;
    return 0;
}