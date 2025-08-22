#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

typedef struct {
    GstBuffer *buffer;
    GstVideoInfo video_info;
    std::chrono::high_resolution_clock::time_point timestamp;
} FrameData;

typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean video_info_valid;
    GstVideoInfo video_info;
    
    // Multi-threading for parallel processing
    std::queue<FrameData> frame_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    pthread_t processing_threads[4]; // Multiple threads
    int num_threads;
    gboolean should_stop;
    
    // Enhanced monitoring
    GTimer *processing_timer;
    double total_processing_time;
    int frame_count;
    int dropped_frames;
    int queue_overflows;
    int appsrc_push_failures;
    
    // Frame timing analysis
    std::chrono::high_resolution_clock::time_point last_input_frame;
    std::chrono::high_resolution_clock::time_point last_output_frame;
    double input_fps_actual;
    double output_fps_actual;
    
    // Queue size monitoring
    size_t max_queue_size;
    size_t current_queue_size;
    
    // Optimization options
    gboolean use_fast_mode;
    gboolean use_parallel;
    int downscale_factor;
    
    // Pre-allocated OpenCV matrices for performance
    cv::Mat temp_y_in;
    cv::Mat temp_y_out;
    cv::Mat temp_nv12;
    std::mutex opencv_mutex; // Protect OpenCV operations if needed
    
} CustomData;

// Fast histogram equalization using OpenCV optimizations
void fast_histogram_equalization(const cv::Mat& src, cv::Mat& dst, CustomData* data) {
    if (data->use_fast_mode) {
        // Method 1: Use smaller processing resolution
        if (data->downscale_factor > 1) {
            cv::Mat small_src, small_dst;
            cv::resize(src, small_src, cv::Size(src.cols / data->downscale_factor, src.rows / data->downscale_factor), 0, 0, cv::INTER_LINEAR);
            cv::equalizeHist(small_src, small_dst);
            cv::resize(small_dst, dst, src.size(), 0, 0, cv::INTER_LINEAR);
        } else {
            // Method 2: Use CLAHE (Contrast Limited Adaptive Histogram Equalization) - faster
            static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
            clahe->apply(src, dst);
        }
    } else {
        // Original method
        cv::equalizeHist(src, dst);
    }
}

// Optimized processing with memory reuse
void process_frame_optimized(const FrameData& frame_data, CustomData* data) {
    GstMapInfo map_info;
    if (!gst_buffer_map(frame_data.buffer, &map_info, GST_MAP_READ)) {
        g_printerr("[PROCESSING] Failed to map buffer\n");
        return;
    }

    int width = frame_data.video_info.width;
    int height = frame_data.video_info.height;
    size_t y_size = (size_t)width * (size_t)height;
    size_t uv_size = (size_t)width * (size_t)height / 2;

    try {
        // Reuse pre-allocated matrices to avoid memory allocation overhead
        cv::Mat nv12_input(height * 3 / 2, width, CV_8UC1, map_info.data);
        cv::Mat y_plane_in = nv12_input(cv::Rect(0, 0, width, height));
        
        // Pre-allocate output matrix once
        if (data->temp_y_out.empty() || data->temp_y_out.cols != width || data->temp_y_out.rows != height) {
            data->temp_y_out = cv::Mat(height, width, CV_8UC1);
            data->temp_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
        }

        // Fast histogram equalization
        fast_histogram_equalization(y_plane_in, data->temp_y_out, data);
        
        // Reconstruct NV12 with optimized memory operations
        memcpy(data->temp_nv12.data, data->temp_y_out.data, y_size);
        memset(data->temp_nv12.data + y_size, 128, uv_size);

        // Create output buffer
        GstBuffer *processed_buffer = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
        if (processed_buffer) {
            GstMapInfo processed_map_info;
            if (gst_buffer_map(processed_buffer, &processed_map_info, GST_MAP_WRITE)) {
                memcpy(processed_map_info.data, data->temp_nv12.data, y_size + uv_size);
                gst_buffer_unmap(processed_buffer, &processed_map_info);
                gst_buffer_copy_into(processed_buffer, frame_data.buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

                // Push to appsrc
                GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), processed_buffer);
                if (ret != GST_FLOW_OK) {
                    data->appsrc_push_failures++;
                    g_printerr("[APPSRC] Push failed: %d\n", ret);
                } else {
                    // Update output timing
                    auto now = std::chrono::high_resolution_clock::now();
                    if (data->last_output_frame.time_since_epoch().count() > 0) {
                        auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - data->last_output_frame).count();
                        if (delta > 0) data->output_fps_actual = 1000000.0 / delta;
                    }
                    data->last_output_frame = now;
                }
            } else {
                g_printerr("[PROCESSING] Failed to map processed buffer\n");
                gst_buffer_unref(processed_buffer);
            }
        }

    } catch (const std::exception& e) {
        g_printerr("[OPENCV] Error: %s\n", e.what());
    }

    gst_buffer_unmap(frame_data.buffer, &map_info);
}

// Multi-threaded processing function
void* processing_thread_func(void* user_data) {
    CustomData *data = (CustomData *)user_data;
    
    while (!data->should_stop) {
        FrameData frame_data;
        bool has_frame = false;
        
        // Get frame from queue
        {
            std::unique_lock<std::mutex> lock(data->queue_mutex);
            data->queue_cv.wait_for(lock, std::chrono::milliseconds(100), 
                [data] { return !data->frame_queue.empty() || data->should_stop; });
            
            if (data->should_stop) break;
            
            if (!data->frame_queue.empty()) {
                frame_data = data->frame_queue.front();
                data->frame_queue.pop();
                data->current_queue_size = data->frame_queue.size();
                has_frame = true;
            }
        }
        
        if (!has_frame) continue;
        
        auto processing_start = std::chrono::high_resolution_clock::now();
        
        // Process frame with optimizations
        process_frame_optimized(frame_data, data);
        
        auto processing_end = std::chrono::high_resolution_clock::now();
        double frame_processing_time = std::chrono::duration_cast<std::chrono::microseconds>(processing_end - processing_start).count() / 1000.0;
        
        // Thread-safe stats update
        {
            std::lock_guard<std::mutex> lock(data->queue_mutex);
            data->total_processing_time += frame_processing_time;
            data->frame_count++;

            // Detailed monitoring every 100 frames
            if (data->frame_count % 100 == 0) {
                double avg_processing = data->total_processing_time / data->frame_count;
                
                g_print("\n=== OPTIMIZED PIPELINE MONITORING (Frame %d) ===\n", data->frame_count);
                g_print("Processing: %.2f ms (avg: %.2f ms, theoretical max FPS: %.1f)\n", 
                    frame_processing_time, avg_processing, 1000.0 / avg_processing);
                g_print("Input FPS: %.1f | Output FPS: %.1f\n", data->input_fps_actual, data->output_fps_actual);
                g_print("Queue: current=%zu, max=%zu, overflows=%d\n", 
                    data->current_queue_size, data->max_queue_size, data->queue_overflows);
                g_print("Drops: frames=%d, appsrc_failures=%d\n", data->dropped_frames, data->appsrc_push_failures);
                g_print("Optimization: fast_mode=%s, downscale=%dx, threads=%d\n", 
                    data->use_fast_mode ? "YES" : "NO", data->downscale_factor, data->num_threads);
                
                // Performance analysis
                if (avg_processing > 16.67) {
                    g_print("⚠️  WARNING: Still processing too slow for 60fps (>16.67ms)\n");
                } else {
                    g_print("✅ Processing fast enough for 60fps\n");
                }
                
                if (data->current_queue_size > 5) {
                    g_print("⚠️  WARNING: Queue backing up (size: %zu)\n", data->current_queue_size);
                } else {
                    g_print("✅ Queue healthy\n");
                }
                
                if (data->output_fps_actual > 55.0) {
                    g_print("✅ Output FPS looking good (%.1f)\n", data->output_fps_actual);
                } else if (data->output_fps_actual > 0) {
                    g_print("⚠️  WARNING: Output FPS below expected (%.1f < 60)\n", data->output_fps_actual);
                }
                g_print("============================================\n\n");
            }
        }

        gst_buffer_unref(frame_data.buffer);
    }
    
    return NULL;
}

// Callback when appsink has a new sample
GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("[INPUT] Failed to pull sample\n");
        return GST_FLOW_ERROR;
    }

    // Update input FPS tracking
    auto now = std::chrono::high_resolution_clock::now();
    if (data->last_input_frame.time_since_epoch().count() > 0) {
        auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - data->last_input_frame).count();
        if (delta > 0) data->input_fps_actual = 1000000.0 / delta;
    }
    data->last_input_frame = now;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("[INPUT] Failed to get buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Initialize video info if needed
    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("[INFO] Video: %dx%d\n", data->video_info.width, data->video_info.height);
        } else {
            g_printerr("[INPUT] Failed to extract video info\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Queue management with dynamic sizing
    {
        std::unique_lock<std::mutex> lock(data->queue_mutex);
        
        const size_t MAX_QUEUE_SIZE = 15; // Increased for better buffering
        if (data->frame_queue.size() >= MAX_QUEUE_SIZE) {
            // Drop oldest frame when queue is full
            if (!data->frame_queue.empty()) {
                FrameData old_frame = data->frame_queue.front();
                data->frame_queue.pop();
                gst_buffer_unref(old_frame.buffer);
                data->dropped_frames++;
                data->queue_overflows++;
            }
        }
        
        // Add new frame to queue
        FrameData frame_data;
        frame_data.buffer = gst_buffer_ref(buffer);
        frame_data.video_info = data->video_info;
        frame_data.timestamp = now;
        
        data->frame_queue.push(frame_data);
        data->current_queue_size = data->frame_queue.size();
        
        if (data->current_queue_size > data->max_queue_size) {
            data->max_queue_size = data->current_queue_size;
        }
    }
    
    data->queue_cv.notify_one();
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // Parse arguments with new optimization options
    gboolean use_h265 = FALSE;
    int bitrate_kbps = 10000;
    gboolean use_fast_mode = TRUE; // Default to fast mode
    int num_threads = 2; // Default to 2 threads
    int downscale_factor = 2; // Default 2x downscale for processing

    for (int i = 1; i < argc; ++i) {
        if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *val = strchr(argv[i], '=');
            if (val && g_ascii_strcasecmp(val + 1, "h265") == 0) use_h265 = TRUE;
        } else if (g_str_has_prefix(argv[i], "--bitrate=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                int v = atoi(val + 1);
                if (v > 0) bitrate_kbps = v;
            }
        } else if (g_str_has_prefix(argv[i], "--fast=")) {
            const char *val = strchr(argv[i], '=');
            if (val) use_fast_mode = g_ascii_strcasecmp(val + 1, "true") == 0;
        } else if (g_str_has_prefix(argv[i], "--threads=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                int v = atoi(val + 1);
                if (v > 0 && v <= 8) num_threads = v;
            }
        } else if (g_str_has_prefix(argv[i], "--downscale=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                int v = atoi(val + 1);
                if (v >= 1 && v <= 4) downscale_factor = v;
            }
        }
    }

    g_print("=== OPTIMIZED CONFIGURATION ===\n");
    g_print("Encoder: %s\n", use_h265 ? "H.265" : "H.264");
    g_print("Target bitrate: %d kbps\n", bitrate_kbps);
    g_print("Expected: 1920x1080@60fps\n");
    g_print("Fast mode: %s\n", use_fast_mode ? "ENABLED" : "DISABLED");
    g_print("Processing threads: %d\n", num_threads);
    g_print("Downscale factor: %dx\n", downscale_factor);
    g_print("===============================\n\n");

    CustomData data = {};
    data.video_info_valid = FALSE;
    data.processing_timer = g_timer_new();
    data.total_processing_time = 0.0;
    data.frame_count = 0;
    data.dropped_frames = 0;
    data.queue_overflows = 0;
    data.appsrc_push_failures = 0;
    data.should_stop = FALSE;
    data.max_queue_size = 0;
    data.current_queue_size = 0;
    data.input_fps_actual = 0.0;
    data.output_fps_actual = 0.0;
    data.use_fast_mode = use_fast_mode;
    data.use_parallel = TRUE;
    data.num_threads = num_threads;
    data.downscale_factor = downscale_factor;

    GError *error = NULL;

    // Optimized camera pipeline
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1 ! "
        "queue max-size-buffers=3 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
        "appsink name=my_sink emit-signals=true max-buffers=2 drop=false sync=false"
    );
    
    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");

    // Optimized streaming pipeline
    gchar *src_pipeline_str = NULL;
    if (use_h265) {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true "
            "max-bytes=0 max-buffers=5 ! "
            "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
            "queue max-size-buffers=3 max-size-time=0 max-size-bytes=0 ! "
            "omxh265enc num-slices=4 periodicity-idr=120 cpb-size=2000 gdr-mode=horizontal "
            "initial-delay=0 control-rate=low-latency prefetch-buffer=false target-bitrate=%d "
            "gop-mode=low-delay-p ! "
            "video/x-h265, alignment=au ! "
            "queue max-size-buffers=5 max-size-time=0 max-size-bytes=0 ! "
            "rtph265pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            bitrate_kbps
        );
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true "
            "max-bytes=0 max-buffers=5 ! "
            "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
            "queue max-size-buffers=3 max-size-time=0 max-size-bytes=0 ! "
            "omxh264enc num-slices=4 periodicity-idr=120 cpb-size=2000 gdr-mode=horizontal "
            "initial-delay=0 control-rate=low-latency prefetch-buffer=false target-bitrate=%d "
            "gop-mode=low-delay-p ! "
            "video/x-h264, alignment=nal ! "
            "queue max-size-buffers=5 max-size-time=0 max-size-bytes=0 ! "
            "rtph264pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            bitrate_kbps
        );
    }

    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error->message);
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return -1;
    }

    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");

    // Start multiple processing threads
    for (int i = 0; i < data.num_threads; i++) {
        pthread_create(&data.processing_threads[i], NULL, processing_thread_func, &data);
    }

    // Register callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Start pipelines
    gst_element_set_state(src_pipeline, GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    g_print("Running with optimizations. Press Ctrl+C to exit.\n\n");

    // Main loop
    GstBus *bus = gst_element_get_bus(sink_pipeline);
    gboolean running = TRUE;
    while (running) {
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus, 100 * GST_MSECOND,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg) {
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = NULL;
                gchar *debug_info = NULL;
                gst_message_parse_error(msg, &err, &debug_info);
                g_printerr("Pipeline Error: %s\n", err->message);
                if (debug_info) g_printerr("Debug: %s\n", debug_info);
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

    // Cleanup
    data.should_stop = TRUE;
    data.queue_cv.notify_all();
    
    for (int i = 0; i < data.num_threads; i++) {
        pthread_join(data.processing_threads[i], NULL);
    }

    // Clear remaining frames
    while (!data.frame_queue.empty()) {
        FrameData frame = data.frame_queue.front();
        data.frame_queue.pop();
        gst_buffer_unref(frame.buffer);
    }

    gst_object_unref(bus);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline, GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    g_timer_destroy(data.processing_timer);

    g_print("\n=== FINAL OPTIMIZED STATS ===\n");
    g_print("Total frames processed: %d\n", data.frame_count);
    g_print("Frames dropped: %d\n", data.dropped_frames);
    g_print("Queue overflows: %d\n", data.queue_overflows);
    g_print("AppSrc push failures: %d\n", data.appsrc_push_failures);
    if (data.frame_count > 0) {
        g_print("Average processing time: %.2f ms\n", data.total_processing_time / data.frame_count);
    }

    return 0;
}