#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <atomic>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>

// OpenCL/FPGA includes
#include <CL/cl.h>
#include <CL/opencl.h>
#include <vector>
#include "xcl2.hpp"

struct Counters {
    // Frame counters for rate calculation
    std::atomic<uint64_t> camera_frames{0};      // Frames captured from camera
    std::atomic<uint64_t> fpga_input_frames{0};  // Frames sent to FPGA
    std::atomic<uint64_t> fpga_output_frames{0}; // Frames processed by FPGA
    std::atomic<uint64_t> encoder_frames{0};     // Frames sent to encoder
    std::atomic<uint64_t> output_bytes{0};       // Output bytes for bitrate calculation

    // Previous counts for rate calculation
    uint64_t prev_camera_frames{0};
    uint64_t prev_fpga_input_frames{0};
    uint64_t prev_fpga_output_frames{0};
    uint64_t prev_encoder_frames{0};
    uint64_t prev_output_bytes{0};

    std::atomic<uint64_t> processing_errors{0};
    std::atomic<uint64_t> total_processing_time_us{0};
    std::atomic<uint64_t> total_idle_calls{0};
};

// FPGA OpenCL context structure - now supports multiple contexts for worker threads
struct FPGAContext {
    cl::Platform platform;
    cl::Device device;
    cl::Context context;
    cl::CommandQueue queue;
    cl::Program program;
    cl::Kernel kernel;
    
    // Memory buffers - aligned for FPGA
    cl::Buffer img_y_in;
    cl::Buffer img_y_in_ref;
    cl::Buffer img_y_out;
    
    // Host buffers - aligned for better performance
    std::vector<uint8_t> host_in_buffer;
    std::vector<uint8_t> host_out_buffer;
    
    bool initialized{false};
    int max_width{0};
    int max_height{0};
    int worker_id{0}; // Worker thread identifier
    
    ~FPGAContext() {
        cleanup();
    }
    
    void cleanup() {
        initialized = false;
    }
};

// Worker thread structure
struct WorkerThread {
    std::thread thread;
    FPGAContext fpga_ctx;
    std::atomic<bool> stop{false};
    int worker_id{0};
    
    // Per-worker statistics
    std::atomic<uint64_t> frames_processed{0};
    std::atomic<uint64_t> processing_time_us{0};
    std::atomic<uint64_t> processing_errors{0};
};

struct CustomData {
    GstElement  *appsrc{nullptr};
    GstElement  *appsink{nullptr};
    gboolean     video_info_valid{FALSE};
    GstVideoInfo video_info{};

    // Worker thread processing
    GAsyncQueue *work_q{nullptr};        // Input queue for worker threads
    GAsyncQueue *output_q{nullptr};      // Output queue from worker threads
    std::vector<WorkerThread> workers;   // Worker thread pool
    guint        output_idle_source_id{0}; // GLib idle source for output processing
    gboolean     output_processing_active{FALSE};
    int          num_workers{2};         // Number of worker threads
    std::atomic<bool> stop{false};
    
    // Queue management
    int          max_queue_depth{12};    // Maximum frames to queue (increased for workers)
    gboolean     drop_frames{TRUE};      // Enable frame dropping when overloaded
    std::mutex   video_info_mutex;       // Protect video_info access from workers

    Counters     ctr{};
    GMainLoop   *loop{nullptr};
};

/* ---------- FPGA OpenCL Initialization for Worker ---------- */

static gboolean init_fpga_context_worker(FPGAContext* ctx, int width, int height, int worker_id) {
    if (ctx->initialized && ctx->max_width >= width && ctx->max_height >= height) {
        return TRUE; // Already initialized with sufficient size
    }
    
    // Clean up previous context if reinitializing
    if (ctx->initialized) {
        ctx->cleanup();
    }
    
    ctx->worker_id = worker_id;
    
    try {
        // Get Xilinx device using XCL utilities
        std::vector<cl::Device> devices = xcl::get_xil_devices();
        if (devices.empty()) {
            g_printerr("Worker %d: No Xilinx devices found\n", worker_id);
            return FALSE;
        }
        
        ctx->device = devices[0];
        std::string device_name = ctx->device.getInfo<CL_DEVICE_NAME>();
        g_print("Worker %d: Using device: %s\n", worker_id, device_name.c_str());
        
        // Create context and command queue for this worker
        ctx->context = cl::Context(ctx->device);
        ctx->queue = cl::CommandQueue(ctx->context, ctx->device, CL_QUEUE_PROFILING_ENABLE | CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE);
        
        // Load binary file using XCL utilities (matching your approach)
        std::string binaryFile = xcl::find_binary_file(device_name, "krnl_hist_equalize");
        cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
        std::vector<cl::Device> devices_for_program = {ctx->device};
        ctx->program = cl::Program(ctx->context, devices_for_program, bins);
        
        // Create kernel
        ctx->kernel = cl::Kernel(ctx->program, "equalizeHist_accel");
        
        // Set up buffers for maximum expected size
        ctx->max_width = std::max(width, 1920);
        ctx->max_height = std::max(height, 1080);
        size_t max_y_size = ctx->max_width * ctx->max_height;
        
        // Align to 256-bit boundaries for FPGA efficiency
        size_t aligned_size = ((max_y_size + 31) / 32) * 32;
        
        // Create OpenCL buffers using C++ API
        ctx->img_y_in = cl::Buffer(ctx->context, CL_MEM_READ_ONLY, aligned_size);
        ctx->img_y_in_ref = cl::Buffer(ctx->context, CL_MEM_READ_ONLY, aligned_size);
        ctx->img_y_out = cl::Buffer(ctx->context, CL_MEM_WRITE_ONLY, aligned_size);
        
        // Allocate host buffers
        ctx->host_in_buffer.resize(aligned_size);
        ctx->host_out_buffer.resize(aligned_size);
        
        ctx->initialized = TRUE;
        g_print("Worker %d: FPGA context initialized for max size %dx%d\n", worker_id, ctx->max_width, ctx->max_height);
        return TRUE;
        
    } catch (const GError& e) {
        g_printerr("OpenCL initialization error");
        return FALSE;
    } catch (const std::exception& e) {
        g_printerr("Worker %d: Exception in init_fpga_context: %s\n", worker_id, e.what());
        return FALSE;
    }
}

/* ---------- Pad probes ---------- */

static GstPadProbeReturn probe_cam_out(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        d->ctr.camera_frames.fetch_add(1, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_apps_sink(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        d->ctr.fpga_input_frames.fetch_add(1, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_encoder_sink(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.encoder_frames.fetch_add(1, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_output(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.output_bytes.fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

/* ---------- Worker thread function ---------- */

static void worker_thread_func(CustomData* d, WorkerThread* worker) {
    g_print("Worker %d: Started\n", worker->worker_id);
    
    while (!worker->stop.load(std::memory_order_acquire) && !d->stop.load(std::memory_order_acquire)) {
        // Get work item from queue (blocking with timeout)
        gpointer item = g_async_queue_timeout_pop(d->work_q, 100000); // 100ms timeout
        if (!item) {
            continue; // Timeout, check stop condition
        }
        
        GstBuffer *inbuf = (GstBuffer*)item;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        try {
            // Get video info safely
            GstVideoInfo video_info;
            bool video_info_valid = false;
            {
                std::lock_guard<std::mutex> lock(d->video_info_mutex);
                video_info = d->video_info;
                video_info_valid = d->video_info_valid;
            }
            
            if (!video_info_valid) {
                gst_buffer_unref(inbuf);
                worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            
            int width = video_info.width;
            int height = video_info.height;
            
            // Initialize FPGA context for this worker if needed
            if (!init_fpga_context_worker(&worker->fpga_ctx, width, height, worker->worker_id)) {
                gst_buffer_unref(inbuf);
                worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            
            // Map input buffer for reading
            GstMapInfo map_info;
            if (!gst_buffer_map(inbuf, &map_info, GST_MAP_READ)) {
                gst_buffer_unref(inbuf);
                worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            
            size_t y_size = (size_t)width * (size_t)height;
            size_t uv_size = (size_t)width * (size_t)height / 2;
            
            if (map_info.size < y_size + uv_size) {
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            
            FPGAContext &ctx = worker->fpga_ctx;
            
            // Copy Y plane data to host buffer
            memcpy(ctx.host_in_buffer.data(), map_info.data, y_size);
            
            // Transfer input data to FPGA using C++ API (non-blocking)
            ctx.queue.enqueueWriteBuffer(ctx.img_y_in, CL_FALSE, 0, y_size, ctx.host_in_buffer.data());
            
            // Transfer reference data to FPGA (same as input for histogram equalization) (non-blocking)
            ctx.queue.enqueueWriteBuffer(ctx.img_y_in_ref, CL_FALSE, 0, y_size, ctx.host_in_buffer.data());

            // Set kernel arguments using C++ API
            ctx.kernel.setArg(0, ctx.img_y_in);
            ctx.kernel.setArg(1, ctx.img_y_in_ref);
            ctx.kernel.setArg(2, ctx.img_y_out);
            ctx.kernel.setArg(3, height);
            ctx.kernel.setArg(4, width);

            // Execute kernel (non-blocking)
            cl::Event kernel_event;
            ctx.queue.enqueueTask(ctx.kernel, nullptr, &kernel_event);

            // Read back result (blocking on kernel completion)
            ctx.queue.enqueueReadBuffer(ctx.img_y_out, CL_TRUE, 0, y_size, ctx.host_out_buffer.data());

            // Create output buffer
            GstBuffer *outbuf = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
            if (!outbuf) {
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            // Map output buffer and reconstruct NV12
            GstMapInfo out_map_info;
            if (gst_buffer_map(outbuf, &out_map_info, GST_MAP_WRITE)) {
                // Copy processed Y plane from FPGA
                memcpy(out_map_info.data, ctx.host_out_buffer.data(), y_size);
                // Fill UV with neutral value 128
                memset(out_map_info.data + y_size, 128, uv_size);
                gst_buffer_unmap(outbuf, &out_map_info);
            } else {
                gst_buffer_unref(outbuf);
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            gst_buffer_unmap(inbuf, &map_info);
            gst_buffer_unref(inbuf);

            // Fresh timestamps in appsrc pipeline
            GST_BUFFER_PTS(outbuf)      = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DTS(outbuf)      = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DURATION(outbuf) = GST_CLOCK_TIME_NONE;

            // Push to output queue for main thread to handle
            g_async_queue_push(d->output_q, outbuf);
            
            // Update worker statistics
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            worker->frames_processed.fetch_add(1, std::memory_order_relaxed);
            worker->processing_time_us.fetch_add(duration.count(), std::memory_order_relaxed);
            
            d->ctr.fpga_output_frames.fetch_add(1, std::memory_order_relaxed);
            d->ctr.total_processing_time_us.fetch_add(duration.count(), std::memory_order_relaxed);

        } catch (const GError& e) {
        g_printerr("OpenCL initialization error");
        } catch (const std::exception& e) {
            gst_buffer_unref(inbuf);
            worker->processing_errors.fetch_add(1, std::memory_order_relaxed);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            g_printerr("Worker %d: Exception: %s\n", worker->worker_id, e.what());
        }
    }
    
    g_print("Worker %d: Stopped\n", worker->worker_id);
}

/* ---------- Main thread idle processing ---------- */

/* ---------- Input frame management with worker threads ---------- */

static gboolean manage_input_queue(CustomData *d) {
    // Aggressive frame dropping if queue is backing up
    int current_queue_length = g_async_queue_length(d->work_q);
    if (current_queue_length > d->max_queue_depth && d->drop_frames) {
        // Drop frames to prevent backup
        int frames_to_drop = current_queue_length - (d->max_queue_depth / 2);
        for (int i = 0; i < frames_to_drop; i++) {
            gpointer dropped_item = g_async_queue_try_pop(d->work_q);
            if (dropped_item) {
                gst_buffer_unref((GstBuffer*)dropped_item);
                d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed); // Count as processing error for monitoring
            } else {
                break;
            }
        }
        g_print("Dropped %d frames to prevent queue backup (queue was %d deep)\n", 
                frames_to_drop, current_queue_length);
        return TRUE; // Frames were dropped
    }
    return FALSE; // No frames dropped
}

/* ---------- appsink callback: O(1) enqueue + trigger processing ---------- */

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    auto *d = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;
    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    if (!inbuf) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    // Cache caps once (diagnostic only) - thread-safe access
    if (!d->video_info_valid) {
        if (GstCaps *caps = gst_sample_get_caps(sample)) {
            std::lock_guard<std::mutex> lock(d->video_info_mutex);
            if (!d->video_info_valid) { // Double-check pattern
                if (gst_video_info_from_caps(&d->video_info, caps)) {
                    d->video_info_valid = TRUE;
                    g_print("Video info: %dx%d\n", d->video_info.width, d->video_info.height);
                }
            }
        }
    }

    // Manage input queue (drop frames if needed)
    manage_input_queue(d);

    // O(1): ref buffer, queue to worker threads, unref sample
    gst_buffer_ref(inbuf);
    g_async_queue_push(d->work_q, inbuf);

    // Trigger output processing if not already active
    if (!d->output_processing_active) {
        d->output_processing_active = TRUE;
        d->output_idle_source_id = g_idle_add(process_output_frames_idle, d);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

/* ---------- periodic status ---------- */

static gboolean status_tick(gpointer user_data) {
    auto *d = (CustomData*)user_data;

    // Get current frame counts
    uint64_t current_camera = d->ctr.camera_frames.load();
    uint64_t current_fpga_in = d->ctr.fpga_input_frames.load();
    uint64_t current_fpga_out = d->ctr.fpga_output_frames.load();
    uint64_t current_encoder = d->ctr.encoder_frames.load();
    uint64_t current_output_bytes = d->ctr.output_bytes.load();

    // Calculate frame rates (frames per second over 2 second interval)
    double camera_fps = (current_camera - d->ctr.prev_camera_frames) / 2.0;
    double fpga_input_fps = (current_fpga_in - d->ctr.prev_fpga_input_frames) / 2.0;
    double fpga_output_fps = (current_fpga_out - d->ctr.prev_fpga_output_frames) / 2.0;
    double encoder_fps = (current_encoder - d->ctr.prev_encoder_frames) / 2.0;
    
    // Calculate output bitrate in kbps (kilobits per second over 2 second interval)
    double output_bitrate_kbps = ((current_output_bytes - d->ctr.prev_output_bytes) * 8.0) / (2.0 * 1000.0);

    // Queue lengths and processing info
    const int input_qlen = g_async_queue_length(d->work_q);
    const int output_qlen = g_async_queue_length(d->output_q);
    const uint64_t proc_errors = d->ctr.processing_errors.load();
    const uint64_t total_proc_time = d->ctr.total_processing_time_us.load();
    const uint64_t processed_total = current_fpga_out;

    double avg_proc_time_ms = 0.0;
    if (processed_total > 0) {
        avg_proc_time_ms = (double)total_proc_time / (double)processed_total / 1000.0; // convert µs to ms
    }

    // Worker statistics
    g_print(
        "\n=== FPGA WORKER FRAME RATE MONITORING (every 2s) ===\n"
        "Camera Capture Rate:     %6.1f fps\n"
        "FPGA Input Rate:         %6.1f fps\n"
        "FPGA Output Rate:        %6.1f fps\n"
        "Encoder Input Rate:      %6.1f fps\n"
        "Output Bitrate:          %6.1f kbps\n"
        "\n"
        "Input Queue: %d (max=%d) | Output Queue: %d | Processing Errors/Drops: %" G_GUINT64_FORMAT "\n"
        "Avg Process Time: %.2f ms | Output Processing: %s\n"
        "Workers: %d | Frame Dropping: %s\n",
        camera_fps,
        fpga_input_fps,
        fpga_output_fps,
        encoder_fps,
        output_bitrate_kbps,
        input_qlen, d->max_queue_depth, output_qlen, proc_errors,
        avg_proc_time_ms,
        d->output_processing_active ? "ACTIVE" : "IDLE",
        d->num_workers,
        d->drop_frames ? "ENABLED" : "DISABLED"
    );

    // Individual worker statistics
    for (size_t i = 0; i < d->workers.size(); i++) {
        const auto& worker = d->workers[i];
        uint64_t worker_frames = worker.frames_processed.load();
        uint64_t worker_time = worker.processing_time_us.load();
        uint64_t worker_errors = worker.processing_errors.load();
        
        double worker_avg_time = 0.0;
        if (worker_frames > 0) {
            worker_avg_time = (double)worker_time / (double)worker_frames / 1000.0; // µs to ms
        }
        
        g_print("  Worker %d: %" G_GUINT64_FORMAT " frames, %.2f ms avg, %" G_GUINT64_FORMAT " errors | FPGA: %s\n",
                (int)i, worker_frames, worker_avg_time, worker_errors,
                worker.fpga_ctx.initialized ? "OK" : "NOT INIT");
    }

    // Store current counts as previous for next calculation
    d->ctr.prev_camera_frames = current_camera;
    d->ctr.prev_fpga_input_frames = current_fpga_in;
    d->ctr.prev_fpga_output_frames = current_fpga_out;
    d->ctr.prev_encoder_frames = current_encoder;
    d->ctr.prev_output_bytes = current_output_bytes;

    return TRUE;
}

/* ---------- bus watch (quit main loop) ---------- */

static gboolean bus_cb(GstBus *bus, GstMessage *msg, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *e=NULL; gchar *dbg=NULL;
            gst_message_parse_error(msg, &e, &dbg);
            g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), e->message);
            g_error_free(e); g_free(dbg);
            if (d->loop) g_main_loop_quit(d->loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("EOS from %s\n", GST_OBJECT_NAME(msg->src));
            if (d->loop) g_main_loop_quit(d->loop);
            break;
        default: break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);
    gst_init(&argc, &argv);

    gboolean use_h265 = FALSE;
    int bitrate_kbps = 20000; // Match basic.cpp default (20 Mbps)
    int v_width = 1920, v_height = 1080, fps = 60; // defaults

    // --- argv parsing ---
    for (int i=1;i<argc;++i){
        if (g_str_has_prefix(argv[i],"--codec=")) { const char* v=strchr(argv[i],'='); if(v&&g_ascii_strcasecmp(v+1,"h265")==0) use_h265=TRUE; }
        else if (g_strcmp0(argv[i],"--codec")==0 && i+1<argc){ if (g_ascii_strcasecmp(argv[i+1],"h265")==0) use_h265=TRUE; }
        else if (g_str_has_prefix(argv[i],"--bitrate=")) { const char* v=strchr(argv[i],'='); if(v){ int b=atoi(v+1); if(b>0) bitrate_kbps=b; } }
        else if (g_strcmp0(argv[i],"--bitrate")==0 && i+1<argc){ int b=atoi(argv[i+1]); if(b>0) bitrate_kbps=b; }
        else if (g_str_has_prefix(argv[i],"--width=")) { const char* v=strchr(argv[i],'='); if(v){ int w=atoi(v+1); if(w>0) v_width=w; } }
        else if (g_strcmp0(argv[i],"--width")==0 && i+1<argc){ int w=atoi(argv[i+1]); if(w>0) v_width=w; }
        else if (g_str_has_prefix(argv[i],"--height=")) { const char* v=strchr(argv[i],'='); if(v){ int h=atoi(v+1); if(h>0) v_height=h; } }
        else if (g_strcmp0(argv[i],"--height")==0 && i+1<argc){ int h=atoi(argv[i+1]); if(h>0) v_height=h; }
        else if (g_str_has_prefix(argv[i],"--fps=")) { const char* v=strchr(argv[i],'='); if(v){ int f=atoi(v+1); if(f>0) fps=f; } }
        else if (g_strcmp0(argv[i],"--fps")==0 && i+1<argc){ int f=atoi(argv[i+1]); if(f>0) fps=f; }
    }
    g_print("Encoder: %s, target-bitrate: %d kbps, FPGA main thread processing, %dx%d@%dfps\n",
            use_h265 ? "H.265" : "H.264", bitrate_kbps, v_width, v_height, fps);

    CustomData d{};
    d.work_q = g_async_queue_new();
    d.frames_per_batch = 1; // Start with 1 frame per batch for FPGA
    d.processing_active = FALSE;
    d.max_queue_depth = 6; // Reasonable queue depth for 60fps
    d.drop_frames = TRUE;  // Enable aggressive frame dropping

    // Capture pipeline with more aggressive buffering for FPGA
    GError *err=NULL;
    gchar *sink_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "queue name=q_cam leaky=downstream max-size-buffers=12 max-size-time=0 max-size-bytes=0 ! "
        "appsink name=cv_sink emit-signals=true max-buffers=2 drop=true sync=false",
        v_width, v_height, fps
    );
    GstElement *sink_pipe = gst_parse_launch(sink_str, &err);
    g_free(sink_str);
    if (!sink_pipe) { g_printerr("Create sink pipeline failed: %s\n", err?err->message:"?"); g_clear_error(&err); return -1; }
    d.appsink = gst_bin_get_by_name(GST_BIN(sink_pipe), "cv_sink");
    if (!d.appsink) { g_printerr("Failed to find appsink 'cv_sink'\n"); gst_object_unref(sink_pipe); return -1; }

    // Streaming pipeline (same as before)
    gchar *src_str=NULL;
    if (use_h265) {
        src_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "queue name=q_after_src leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 ! "
            "omxh265enc name=enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h265,alignment=au ! "
            "rtph265pay name=pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            v_width, v_height, fps, bitrate_kbps
        );
    } else {
        src_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "queue name=q_after_src leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 ! "
            "omxh264enc name=enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h264,alignment=nal ! "
            "rtph264pay name=pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            v_width, v_height, fps, bitrate_kbps
        );
    }
    GstElement *src_pipe = gst_parse_launch(src_str, &err);
    g_free(src_str);
    if (!src_pipe) {
        g_printerr("Create src pipeline failed: %s\n", err?err->message:"?");
        g_clear_error(&err);
        gst_object_unref(sink_pipe);
        return -1;
    }
    d.appsrc = gst_bin_get_by_name(GST_BIN(src_pipe), "my_src");
    if (!d.appsrc) {
        g_printerr("Failed to find appsrc 'my_src'\n");
        gst_object_unref(src_pipe);
        gst_object_unref(sink_pipe);
        return -1;
    }

    // Setup probes for frame rate monitoring
    {
        GstElement *q_cam = gst_bin_get_by_name(GST_BIN(sink_pipe), "q_cam");
        if (q_cam) {
            if (GstPad *p = gst_element_get_static_pad(q_cam, "src")) { 
                gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_cam_out, &d, NULL); 
                gst_object_unref(p); 
            }
            gst_object_unref(q_cam);
        }
    }
    
    { 
        if (GstPad *p = gst_element_get_static_pad(d.appsink, "sink")) { 
            gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_apps_sink, &d, NULL); 
            gst_object_unref(p); 
        } 
    }
    
    {
        GstElement *enc = gst_bin_get_by_name(GST_BIN(src_pipe), "enc");
        if (enc) { 
            if (GstPad *p = gst_element_get_static_pad(enc, "sink")) { 
                gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_encoder_sink, &d, NULL); 
                gst_object_unref(p); 
            } 
            gst_object_unref(enc); 
        }
    }
    
    {
        GstElement *pay = gst_bin_get_by_name(GST_BIN(src_pipe), "pay");
        if (pay) { 
            if (GstPad *p = gst_element_get_static_pad(pay, "src")) { 
                gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_output, &d, NULL); 
                gst_object_unref(p); 
            } 
            gst_object_unref(pay); 
        }
    }

    // Callback (triggers main thread idle processing)
    g_signal_connect(d.appsink, "new-sample", G_CALLBACK(new_sample_cb), &d);

    // GLib loop + watches + status timer
    d.loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus_sink = gst_element_get_bus(sink_pipe);
    GstBus *bus_src  = gst_element_get_bus(src_pipe);
    gst_bus_add_watch(bus_sink, bus_cb, &d);
    gst_bus_add_watch(bus_src,  bus_cb, &d);
    g_timeout_add_seconds(2, status_tick, &d);

    // Start & run
    gst_element_set_state(src_pipe,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipe, GST_STATE_PLAYING);
    g_print("FPGA histogram equalization processing with frame rate monitoring. Press Ctrl+C to exit.\n");
    g_print("Make sure equalizeHist_accel.xclbin is in the current directory.\n");
    g_main_loop_run(d.loop);

    // Shutdown
    d.stop.store(true, std::memory_order_release);
    
    // Remove idle source if still active
    if (d.processing_active && d.idle_source_id > 0) {
        g_source_remove(d.idle_source_id);
        d.processing_active = FALSE;
    }
    
    // Drain worker queue
    while (GstBuffer *b = (GstBuffer*)g_async_queue_try_pop(d.work_q)) {
        gst_buffer_unref(b);
    }

    if (d.work_q) { 
        g_async_queue_unref(d.work_q); 
        d.work_q = nullptr; 
    }

    gst_element_set_state(sink_pipe, GST_STATE_NULL);
    gst_element_set_state(src_pipe,  GST_STATE_NULL);
    gst_object_unref(bus_sink);
    gst_object_unref(bus_src);
    gst_object_unref(d.appsink);
    gst_object_unref(d.appsrc);
    gst_object_unref(sink_pipe);
    gst_object_unref(src_pipe);
    g_main_loop_unref(d.loop);
    
    g_print("FPGA main thread processing shutdown complete.\n");
    return 0;
}