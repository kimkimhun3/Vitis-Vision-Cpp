// relay_debug_nv12_mainthread_fpga.cpp
// Main thread processing with FPGA histogram equalization + frame rate monitoring.
//
// Build:
// g++ -O3 -DNDEBUG -std=c++17 relay_debug_nv12_mainthread_fpga.cpp -o relay_debug_mainthread_fpga \
//   $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0) -lpthread \
//   -lxilinxopencl -lOpenCL

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

// OpenCL/FPGA includes
#include <CL/cl.h>
#include <vector>

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

// FPGA OpenCL context structure
struct FPGAContext {
    cl_platform_id platform{nullptr};
    cl_device_id device{nullptr};
    cl_context context{nullptr};
    cl_command_queue queue{nullptr};
    cl_program program{nullptr};
    cl_kernel kernel{nullptr};
    
    // Memory buffers - aligned for FPGA
    cl_mem img_y_in{nullptr};
    cl_mem img_y_in_ref{nullptr};
    cl_mem img_y_out{nullptr};
    
    // Host buffers - aligned for better performance
    std::vector<uint8_t> host_in_buffer;
    std::vector<uint8_t> host_out_buffer;
    
    bool initialized{false};
    int max_width{0};
    int max_height{0};
    
    ~FPGAContext() {
        cleanup();
    }
    
    void cleanup() {
        if (img_y_in) clReleaseMemObject(img_y_in);
        if (img_y_in_ref) clReleaseMemObject(img_y_in_ref);
        if (img_y_out) clReleaseMemObject(img_y_out);
        if (kernel) clReleaseKernel(kernel);
        if (program) clReleaseProgram(program);
        if (queue) clReleaseCommandQueue(queue);
        if (context) clReleaseContext(context);
        
        img_y_in = img_y_in_ref = img_y_out = nullptr;
        kernel = nullptr;
        program = nullptr;
        queue = nullptr;
        context = nullptr;
        initialized = false;
    }
};

struct CustomData {
    GstElement  *appsrc{nullptr};
    GstElement  *appsink{nullptr};
    gboolean     video_info_valid{FALSE};
    GstVideoInfo video_info{};

    // Main thread processing (replaces worker threads)
    GAsyncQueue *work_q{nullptr};        // Still need the queue
    guint        idle_source_id{0};      // GLib idle source ID
    gboolean     processing_active{FALSE}; // Track if idle processing is running
    int          frames_per_batch{1};    // Process N frames per idle call (start with 1 for FPGA)
    gint64       avg_frame_time_us{10000}; // Rolling average processing time (higher for FPGA)
    std::atomic<bool> stop{false};

    Counters     ctr{};
    GMainLoop   *loop{nullptr};
    
    // FPGA context
    FPGAContext fpga_ctx{};
};

/* ---------- FPGA OpenCL Initialization ---------- */

static gboolean init_fpga_context(CustomData *d, int width, int height) {
    FPGAContext &ctx = d->fpga_ctx;
    
    if (ctx.initialized && ctx.max_width >= width && ctx.max_height >= height) {
        return TRUE; // Already initialized with sufficient size
    }
    
    // Clean up previous context if reinitializing
    if (ctx.initialized) {
        ctx.cleanup();
    }
    
    cl_int err;
    
    // Get platform
    cl_uint num_platforms;
    err = clGetPlatformIDs(0, nullptr, &num_platforms);
    if (err != CL_SUCCESS || num_platforms == 0) {
        g_printerr("Failed to get OpenCL platforms\n");
        return FALSE;
    }
    
    std::vector<cl_platform_id> platforms(num_platforms);
    err = clGetPlatformIDs(num_platforms, platforms.data(), nullptr);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to get platform IDs\n");
        return FALSE;
    }
    
    // Find Xilinx platform
    for (auto platform : platforms) {
        char platform_name[256];
        err = clGetPlatformInfo(platform, CL_PLATFORM_NAME, sizeof(platform_name), platform_name, nullptr);
        if (err == CL_SUCCESS && strstr(platform_name, "Xilinx")) {
            ctx.platform = platform;
            break;
        }
    }
    
    if (!ctx.platform) {
        g_printerr("Xilinx platform not found\n");
        return FALSE;
    }
    
    // Get device
    err = clGetDeviceIDs(ctx.platform, CL_DEVICE_TYPE_ACCELERATOR, 1, &ctx.device, nullptr);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to get FPGA device\n");
        return FALSE;
    }
    
    // Create context
    ctx.context = clCreateContext(nullptr, 1, &ctx.device, nullptr, nullptr, &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create OpenCL context\n");
        return FALSE;
    }
    
    // Create command queue
    ctx.queue = clCreateCommandQueue(ctx.context, ctx.device, 0, &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create command queue\n");
        return FALSE;
    }
    
    // Load and create program (you need to provide the .xclbin file path)
    // This is a placeholder - you need to load your actual bitstream
    const char* xclbin_path = "equalizeHist_accel.xclbin"; // Update this path
    FILE* fp = fopen(xclbin_path, "rb");
    if (!fp) {
        g_printerr("Failed to open xclbin file: %s\n", xclbin_path);
        return FALSE;
    }
    
    fseek(fp, 0, SEEK_END);
    size_t binary_size = ftell(fp);
    rewind(fp);
    
    std::vector<char> binary(binary_size);
    fread(binary.data(), 1, binary_size, fp);
    fclose(fp);
    
    const char* binary_ptr = binary.data();
    ctx.program = clCreateProgramWithBinary(ctx.context, 1, &ctx.device, &binary_size,
                                           (const unsigned char**)&binary_ptr, nullptr, &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create program with binary\n");
        return FALSE;
    }
    
    err = clBuildProgram(ctx.program, 1, &ctx.device, nullptr, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to build program\n");
        return FALSE;
    }
    
    // Create kernel
    ctx.kernel = clCreateKernel(ctx.program, "equalizeHist_accel", &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create kernel\n");
        return FALSE;
    }
    
    // Set up buffers for maximum expected size
    ctx.max_width = std::max(width, 1920);
    ctx.max_height = std::max(height, 1080);
    size_t max_y_size = ctx.max_width * ctx.max_height;
    
    // Align to 256-bit boundaries for FPGA efficiency
    size_t aligned_size = ((max_y_size + 31) / 32) * 32;
    
    // Create OpenCL buffers
    ctx.img_y_in = clCreateBuffer(ctx.context, CL_MEM_READ_ONLY, aligned_size, nullptr, &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create input buffer\n");
        return FALSE;
    }
    
    ctx.img_y_in_ref = clCreateBuffer(ctx.context, CL_MEM_READ_ONLY, aligned_size, nullptr, &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create input reference buffer\n");
        return FALSE;
    }
    
    ctx.img_y_out = clCreateBuffer(ctx.context, CL_MEM_WRITE_ONLY, aligned_size, nullptr, &err);
    if (err != CL_SUCCESS) {
        g_printerr("Failed to create output buffer\n");
        return FALSE;
    }
    
    // Allocate host buffers
    ctx.host_in_buffer.resize(aligned_size);
    ctx.host_out_buffer.resize(aligned_size);
    
    ctx.initialized = TRUE;
    g_print("FPGA context initialized for max size %dx%d\n", ctx.max_width, ctx.max_height);
    return TRUE;
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

/* ---------- Single frame processing function with FPGA ---------- */

static gboolean process_single_frame_fpga(CustomData *d, GstBuffer *inbuf) {
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Map input buffer for reading
        GstMapInfo map_info;
        if (!gst_buffer_map(inbuf, &map_info, GST_MAP_READ)) {
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        if (!d->video_info_valid) {
            gst_buffer_unmap(inbuf, &map_info);
            return FALSE;
        }

        int width = d->video_info.width;
        int height = d->video_info.height;
        size_t y_size = (size_t)width * (size_t)height;
        size_t uv_size = (size_t)width * (size_t)height / 2;

        if (map_info.size < y_size + uv_size) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        // Initialize FPGA context if needed
        if (!init_fpga_context(d, width, height)) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        FPGAContext &ctx = d->fpga_ctx;
        cl_int err;

        // Copy Y plane data to host buffer
        memcpy(ctx.host_in_buffer.data(), map_info.data, y_size);
        
        // For histogram equalization, we use the same frame as both input and reference
        memcpy(ctx.host_in_buffer.data(), map_info.data, y_size); // Same data for reference
        
        // Transfer input data to FPGA
        err = clEnqueueWriteBuffer(ctx.queue, ctx.img_y_in, CL_FALSE, 0, y_size, 
                                  ctx.host_in_buffer.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }
        
        // Transfer reference data to FPGA (same as input for histogram equalization)
        err = clEnqueueWriteBuffer(ctx.queue, ctx.img_y_in_ref, CL_FALSE, 0, y_size,
                                  ctx.host_in_buffer.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        // Set kernel arguments
        err = clSetKernelArg(ctx.kernel, 0, sizeof(cl_mem), &ctx.img_y_in);
        err |= clSetKernelArg(ctx.kernel, 1, sizeof(cl_mem), &ctx.img_y_in_ref);
        err |= clSetKernelArg(ctx.kernel, 2, sizeof(cl_mem), &ctx.img_y_out);
        err |= clSetKernelArg(ctx.kernel, 3, sizeof(int), &height);
        err |= clSetKernelArg(ctx.kernel, 4, sizeof(int), &width);
        
        if (err != CL_SUCCESS) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        // Execute kernel
        err = clEnqueueTask(ctx.queue, ctx.kernel, 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        // Read back result
        err = clEnqueueReadBuffer(ctx.queue, ctx.img_y_out, CL_TRUE, 0, y_size,
                                 ctx.host_out_buffer.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        // Update rolling average processing time
        gint64 frame_time = duration.count();
        d->avg_frame_time_us = (d->avg_frame_time_us * 9 + frame_time) / 10; // Rolling average
        
        d->ctr.total_processing_time_us.fetch_add(frame_time, std::memory_order_relaxed);

        // Create output buffer
        GstBuffer *outbuf = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
        if (!outbuf) {
            gst_buffer_unmap(inbuf, &map_info);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
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
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return FALSE;
        }

        gst_buffer_unmap(inbuf, &map_info);

        // Fresh timestamps in appsrc pipeline
        GST_BUFFER_PTS(outbuf)      = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DTS(outbuf)      = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DURATION(outbuf) = GST_CLOCK_TIME_NONE;

        // Count FPGA output frame
        d->ctr.fpga_output_frames.fetch_add(1, std::memory_order_relaxed);

        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), outbuf);
        if (ret != GST_FLOW_OK) {
            gst_buffer_unref(outbuf);
            return FALSE;
        }

        return TRUE;

    } catch (const std::exception& e) {
        d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
        g_printerr("FPGA processing error: %s\n", e.what());
        return FALSE;
    }
}

/* ---------- Main thread idle processing ---------- */

static gboolean process_frames_idle(gpointer user_data) {
    auto *d = (CustomData*)user_data;
    
    if (d->stop.load(std::memory_order_acquire)) {
        d->processing_active = FALSE;
        return G_SOURCE_REMOVE;
    }
    
    d->ctr.total_idle_calls.fetch_add(1, std::memory_order_relaxed);
    
    gint64 start_time = g_get_monotonic_time();
    const gint64 TIME_BUDGET_US = 15000; // 15ms budget per idle call (higher for FPGA)
    int processed_count = 0;
    
    // Conservative batch size for FPGA - typically process one frame at a time
    int max_batch = 1;
    if (d->avg_frame_time_us < 5000) { // < 5ms per frame (very fast FPGA)
        max_batch = 2;
    }
    
    // Process frames within time budget
    while (processed_count < max_batch) {
        // Non-blocking pop - return immediately if no frames
        gpointer item = g_async_queue_try_pop(d->work_q);
        if (!item) {
            // No more frames available
            if (processed_count == 0) {
                // No frames processed, can deactivate idle processing
                d->processing_active = FALSE;
                return G_SOURCE_REMOVE;
            }
            break; // Processed some frames, continue idle source
        }
        
        GstBuffer *inbuf = (GstBuffer*)item;
        
        // Process single frame with FPGA
        if (process_single_frame_fpga(d, inbuf)) {
            processed_count++;
        }
        
        gst_buffer_unref(inbuf); // Release input buffer
        
        // Check if we've exceeded our time budget
        gint64 elapsed = g_get_monotonic_time() - start_time;
        if (elapsed > TIME_BUDGET_US) {
            break; // Yield control back to main loop
        }
    }
    
    // Update batch size for next time
    d->frames_per_batch = max_batch;
    
    return G_SOURCE_CONTINUE; // Keep the idle source active
}

/* ---------- appsink callback: O(1) enqueue + trigger processing ---------- */

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    auto *d = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;
    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    if (!inbuf) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    // Cache caps once (diagnostic only)
    if (!d->video_info_valid) {
        if (GstCaps *caps = gst_sample_get_caps(sample)) {
            if (gst_video_info_from_caps(&d->video_info, caps)) {
                d->video_info_valid = TRUE;
                g_print("Video info: %dx%d\n", d->video_info.width, d->video_info.height);
            }
        }
    }

    // O(1): ref buffer, queue to main thread processing, unref sample
    gst_buffer_ref(inbuf);
    g_async_queue_push(d->work_q, inbuf);

    // Trigger idle processing if not already active
    if (!d->processing_active) {
        d->processing_active = TRUE;
        d->idle_source_id = g_idle_add(process_frames_idle, d);
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

    // Queue length and processing info
    const int qlen = g_async_queue_length(d->work_q);
    const uint64_t proc_errors = d->ctr.processing_errors.load();
    const uint64_t total_proc_time = d->ctr.total_processing_time_us.load();
    const uint64_t processed_total = current_fpga_out;

    double avg_proc_time_ms = 0.0;
    if (processed_total > 0) {
        avg_proc_time_ms = (double)total_proc_time / (double)processed_total / 1000.0; // convert µs to ms
    }

    g_print(
        "\n=== FPGA FRAME RATE MONITORING (every 2s) ===\n"
        "Camera Capture Rate:     %6.1f fps\n"
        "FPGA Input Rate:         %6.1f fps\n"
        "FPGA Output Rate:        %6.1f fps\n"
        "Encoder Input Rate:      %6.1f fps\n"
        "Output Bitrate:          %6.1f kbps\n"
        "\n"
        "Queue Length: %d | Processing Errors: %" G_GUINT64_FORMAT " | Avg Process Time: %.2f ms\n"
        "Processing Status: %s (batch=%d, avg_frame_time=%.1fms)\n"
        "FPGA Status: %s\n",
        camera_fps,
        fpga_input_fps,
        fpga_output_fps,
        encoder_fps,
        output_bitrate_kbps,
        qlen, proc_errors, avg_proc_time_ms,
        d->processing_active ? "ACTIVE" : "IDLE",
        d->frames_per_batch,
        d->avg_frame_time_us / 1000.0,
        d->fpga_ctx.initialized ? "INITIALIZED" : "NOT INITIALIZED"
    );

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

    // Capture pipeline (same as before)
    GError *err=NULL;
    gchar *sink_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "queue name=q_cam leaky=downstream max-size-buffers=8 max-size-time=0 max-size-bytes=0 ! "
        "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true sync=false",
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