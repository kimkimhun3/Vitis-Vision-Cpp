// histequalize_host.cpp — NV12 appsink -> FPGA Y equalize -> NV12 appsrc -> encoder -> RTP
// - Drops OLDEST frames when overloaded (both GStreamer queues and C++ queue)
// - Keeps PTS, low-latency settings
// - Uses mmap (io-mode=2) so gst_buffer_map() is reliable

#include "common/xf_headers.hpp"
#include "common/xf_params.hpp"
#include "gst/gstpad.h"
#include "gst/gstsample.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/imgcodecs.hpp"
#include "xcl2.hpp"
#include "xf_config_params.h"
#include "xf_hist_equalize_tb_config.h"

#include <cstddef>
#include <unistd.h>
#include <glib.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <string>
#include <cstring>

// ------------ Constants / CLI ------------
#define MAX_PROCESSING_QUEUE 4

static char *port = (char *)"5000"; // (unused; kept for compatibility)
static gboolean disable_rtcp = FALSE; // (unused)
static char *out = (char *)"h264";   // (unused)
static int bitrate = 10000;          // kbps default 10Mbps
static char *fps = (char *)"60";
static char *in = (char *)"/dev/video0";
static char *input_resolution = (char *)"4K";
static int v_width = 3840;
static int v_height = 2160;
static char *codec = (char *)"h264"; // h264 | h265 | hevc
int k = 4;

static GOptionEntry entries[] = {
    {"port", 'p', 0, G_OPTION_ARG_STRING, &port, "RTSP port (unused)", NULL},
    {"in", 'i', 0, G_OPTION_ARG_STRING, &in, "Input device (default: /dev/video0)", NULL},
    {"out", 'o', 0, G_OPTION_ARG_STRING, &out, "Output (unused)", NULL},
    {"codec", 'c', 0, G_OPTION_ARG_STRING, &codec, "Codec: h264|h265|hevc (default: h264)", NULL},
    {"bitrate", 'b', 0, G_OPTION_ARG_INT, &bitrate, "Bitrate in kbps (default: 10000)", NULL},
    {"fps", 'f', 0, G_OPTION_ARG_STRING, &fps, "Frames per second (default: 60)", NULL},
    {"input", 'r', 0, G_OPTION_ARG_STRING, &input_resolution, "Input resolution: 2K or 4K (default: 4K)", NULL},
    {"k", 'k', 0, G_OPTION_ARG_INT, &k, "K parameter (unused here, default: 4)", NULL},
    {NULL}
};

// ------------ Frame / App State ------------
struct FrameData {
    GstBuffer *input_buffer;   // ref'd for worker mapping
    GstBuffer *output_buffer;  // from pool or allocate
    size_t y_size;
    size_t uv_size;
    int width;
    int height;
    GstClockTime timestamp;
};

typedef struct {
    // gst
    GstElement *app_source = nullptr;
    GstElement *app_sink   = nullptr;
    gboolean    video_info_valid = FALSE;
    GstVideoInfo video_info{};

    // opencl
    cl::Context context;
    cl::CommandQueue q;
    cl::Kernel krnl;

    // device buffers (Y in/out)
    cl::Buffer imageToDeviceY1;
    cl::Buffer imageToDeviceY2;
    cl::Buffer imageFromDevice;
    bool   buffers_allocated = false;
    size_t allocated_y_size  = 0;

    // pool
    GstBufferPool *buffer_pool = nullptr;

    // threading
    std::queue<FrameData> processing_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    bool processing_thread_running = true;
    std::thread processing_thread;

    // perf
    int frame_count = 0;
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point last_fps_check;

    GTimer *rate_timer = nullptr;
} CustomData;

// ------------ Helpers ------------
static gboolean set_resolution_from_input(const char* input_res) {
    if (g_strcmp0(input_res, "4K") == 0) {
        v_width = 3840; v_height = 2160;
        g_print("Selected 4K resolution: %dx%d\n", v_width, v_height);
        return TRUE;
    } else if (g_strcmp0(input_res, "2K") == 0) {
        v_width = 1920; v_height = 1080;
        g_print("Selected 2K resolution: %dx%d\n", v_width, v_height);
        return TRUE;
    }
    g_printerr("Invalid input resolution: %s. Supported values: 2K, 4K\n", input_res);
    return FALSE;
}

static void initialize_buffer_pool(CustomData *data, int width, int height) {
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width",  G_TYPE_INT,    width,
        "height", G_TYPE_INT,    height,
        NULL);

    data->buffer_pool = gst_buffer_pool_new();
    GstStructure *config = gst_buffer_pool_get_config(data->buffer_pool);
    gst_buffer_pool_config_set_params(config, caps, width * height * 3 / 2, 8, 16);
    gst_buffer_pool_set_config(data->buffer_pool, config);
    gst_buffer_pool_set_active(data->buffer_pool, TRUE);
    gst_caps_unref(caps);

    g_print("Buffer pool initialized with %dx%d NV12 buffers\n", width, height);
}

static void update_performance_stats(CustomData *data) {
    data->frame_count++;
    auto now = std::chrono::high_resolution_clock::now();
    if (data->frame_count == 1) {
        data->start_time = now;
        data->last_fps_check = now;
    }
    if (data->frame_count % 60 == 0) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - data->last_fps_check).count();
        if (ms > 0) {
            double fps_current = 60000.0 / ms;
            auto sec_total = std::chrono::duration_cast<std::chrono::seconds>(now - data->start_time).count();
            double fps_avg = (sec_total > 0) ? (double)data->frame_count / (double)sec_total : 0.0;
            size_t qsz;
            { std::lock_guard<std::mutex> lk(data->queue_mutex); qsz = data->processing_queue.size(); }
            g_print("[STATS] Current=%.1f fps | Average=%.1f fps | Queue=%zu | Total=%d\n",
                    fps_current, fps_avg, qsz, data->frame_count);
        }
        data->last_fps_check = now;
    }
}

// ------------ Worker Path ------------
static void process_frame_async(CustomData *data, const FrameData &frame_data) {
    // Map input buffer (safe — lifetime held by this thread)
    GstMapInfo in_map{};
    if (!gst_buffer_map(frame_data.input_buffer, &in_map, GST_MAP_READ)) {
        g_printerr("Worker: failed to map input buffer\n");
        gst_buffer_unref(frame_data.input_buffer);
        gst_buffer_unref(frame_data.output_buffer);
        return;
    }
    if (in_map.size < frame_data.y_size + frame_data.uv_size) {
        g_printerr("Worker: input buffer too small (%zu < %zu)\n",
                   in_map.size, frame_data.y_size + frame_data.uv_size);
        gst_buffer_unmap(frame_data.input_buffer, &in_map);
        gst_buffer_unref(frame_data.input_buffer);
        gst_buffer_unref(frame_data.output_buffer);
        return;
    }

    const unsigned char *y_src = in_map.data;

    try {
        // Kernel args
        data->krnl.setArg(0, data->imageToDeviceY1);
        data->krnl.setArg(1, data->imageToDeviceY2);
        data->krnl.setArg(2, data->imageFromDevice);
        data->krnl.setArg(3, frame_data.height);
        data->krnl.setArg(4, frame_data.width);

        // IO and kernel
        cl::Event we1, we2, ke;
        std::vector<cl::Event> wes;

        data->q.enqueueWriteBuffer(data->imageToDeviceY1, CL_FALSE, 0, frame_data.y_size, y_src, nullptr, &we1);
        data->q.enqueueWriteBuffer(data->imageToDeviceY2, CL_FALSE, 0, frame_data.y_size, y_src, nullptr, &we2);
        wes.push_back(we1); wes.push_back(we2);

        data->q.enqueueTask(data->krnl, &wes, &ke);

        // Map output and read
        GstMapInfo out_map{};
        if (!gst_buffer_map(frame_data.output_buffer, &out_map, GST_MAP_WRITE)) {
            g_printerr("Worker: failed to map output buffer\n");
            gst_buffer_unmap(frame_data.input_buffer, &in_map);
            gst_buffer_unref(frame_data.input_buffer);
            gst_buffer_unref(frame_data.output_buffer);
            return;
        }
        if (out_map.size < frame_data.y_size + frame_data.uv_size) {
            g_printerr("Worker: output buffer too small\n");
            gst_buffer_unmap(frame_data.output_buffer, &out_map);
            gst_buffer_unmap(frame_data.input_buffer, &in_map);
            gst_buffer_unref(frame_data.input_buffer);
            gst_buffer_unref(frame_data.output_buffer);
            return;
        }

        std::vector<cl::Event> kes{ke};
        cl::Event re;
        data->q.enqueueReadBuffer(data->imageFromDevice, CL_TRUE, 0, frame_data.y_size,
                                  out_map.data, &kes, &re);

        // UV = 128
        std::memset(out_map.data + frame_data.y_size, 128, frame_data.uv_size);

        gst_buffer_unmap(frame_data.output_buffer, &out_map);
    }
catch (const GError& e) {
    g_printerr("OpenCL initialization error");
  }
catch (const GError& e) {
    g_printerr("OpenCL initialization error");
  }

    // Done with input
    gst_buffer_unmap(frame_data.input_buffer, &in_map);
    gst_buffer_unref(frame_data.input_buffer);

    // Timestamp & push to appsrc (ownership transfer on success)
    GST_BUFFER_PTS(frame_data.output_buffer) = frame_data.timestamp;
    GST_BUFFER_DTS(frame_data.output_buffer) = frame_data.timestamp;

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->app_source), frame_data.output_buffer);
    if (ret != GST_FLOW_OK) {
        g_printerr("appsrc push failed: %d\n", ret);
        gst_buffer_unref(frame_data.output_buffer);
    }
}

static void processing_thread_func(CustomData *data) {
    g_print("Processing thread started\n");
    while (data->processing_thread_running) {
        FrameData job{};
        {
            std::unique_lock<std::mutex> lk(data->queue_mutex);
            data->queue_cv.wait(lk, [data]{
                return !data->processing_queue.empty() || !data->processing_thread_running;
            });
            if (!data->processing_thread_running) break;
            job = data->processing_queue.front();
            data->processing_queue.pop();
        }
        process_frame_async(data, job);
    }
    g_print("Processing thread stopped\n");
}

// ------------ Appsink callback ------------
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("Failed to pull sample\n");
        return GST_FLOW_ERROR;
    }
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("No buffer in sample\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (!caps || !gst_video_info_from_caps(&data->video_info, caps)) {
            g_printerr("Failed to get video info from caps\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
        data->video_info_valid = TRUE;
        initialize_buffer_pool(data, data->video_info.width, data->video_info.height);
        g_print("Negotiated: %dx%d NV12 @ %s/1\n", data->video_info.width, data->video_info.height, fps);
    }

    const int width  = data->video_info.width;
    const int height = data->video_info.height;
    if (width <= 0 || height <= 0) {
        g_printerr("Invalid dimensions %dx%d\n", width, height);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    const size_t y_size  = (size_t)width * (size_t)height;
    const size_t uv_size = y_size / 2;

    // (Re)allocate CL buffers if needed
    if (!data->buffers_allocated || data->allocated_y_size != y_size) {
        try {
            data->imageToDeviceY1 = cl::Buffer(data->context, CL_MEM_READ_ONLY,  y_size);
            data->imageToDeviceY2 = cl::Buffer(data->context, CL_MEM_READ_ONLY,  y_size);
            data->imageFromDevice = cl::Buffer(data->context, CL_MEM_WRITE_ONLY, y_size);
            data->buffers_allocated = true;
            data->allocated_y_size = y_size;
            g_print("CL buffers allocated for %dx%d\n", width, height);
        } catch (const GError& e) {
    g_printerr("OpenCL initialization error");
  }
    }

    // Acquire output buffer
    GstBuffer *processed_buffer = nullptr;
    GstFlowReturn acq = gst_buffer_pool_acquire_buffer(data->buffer_pool, &processed_buffer, nullptr);
    if (acq != GST_FLOW_OK || !processed_buffer) {
        processed_buffer = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
        if (!processed_buffer) {
            g_printerr("Failed to alloc output buffer\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Build job BEFORE locking
    FrameData frame_data{};
    frame_data.input_buffer  = gst_buffer_ref(buffer);  // keep for worker mapping
    frame_data.output_buffer = processed_buffer;
    frame_data.y_size  = y_size;
    frame_data.uv_size = uv_size;
    frame_data.width   = width;
    frame_data.height  = height;
    frame_data.timestamp = GST_BUFFER_PTS(buffer);

    // Drop OLDEST when full
    {
        std::lock_guard<std::mutex> lock(data->queue_mutex);
        while (data->processing_queue.size() >= MAX_PROCESSING_QUEUE) {
            FrameData old = data->processing_queue.front();
            data->processing_queue.pop();
            gst_buffer_unref(old.input_buffer);
            gst_buffer_unref(old.output_buffer);
            g_printerr("Queue full → dropped oldest frame\n");
        }
        data->processing_queue.push(frame_data);
    }
    data->queue_cv.notify_one();

    update_performance_stats(data);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// ------------ Encoder chain ------------
static gchar* get_encoder_string(const char* codec_type, guint target_bitrate_kbps) {
    if (g_strcmp0(codec_type, "h265") == 0 || g_strcmp0(codec_type, "hevc") == 0) {
        return g_strdup_printf(
            "omxh265enc target-bitrate=%u num-slices=1 "
            "control-rate=low-latency qp-mode=auto prefetch-buffer=true "
            "cpb-size=200 initial-delay=200 "
            "gdr-mode=auto periodicity-idr=30 gop-length=30 filler-data=true ! "
            "video/x-h265, alignment=au, profile=main, stream-format=byte-stream ! "
            "rtph265pay mtu=1400", target_bitrate_kbps);
    } else {
        return g_strdup_printf(
            "omxh264enc target-bitrate=%u num-slices=1 "
            "control-rate=low-latency qp-mode=auto prefetch-buffer=true "
            "cpb-size=200 initial-delay=200 "
            "gdr-mode=auto periodicity-idr=30 gop-length=30 filler-data=true ! "
            "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
            "rtph264pay mtu=1400", target_bitrate_kbps);
    }
}

// ------------ main ------------
int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    GOptionContext *optctx = g_option_context_new("- GStreamer Video Processing Application");
    GError *error = NULL;
    g_option_context_add_main_entries(optctx, entries, NULL);
    g_option_context_add_group(optctx, gst_init_get_option_group());
    if (!g_option_context_parse(optctx, &argc, &argv, &error)) {
        g_printerr("Option parse error: %s\n", error->message);
        g_option_context_free(optctx);
        g_clear_error(&error);
        return -1;
    }
    g_option_context_free(optctx);

    if (!set_resolution_from_input(input_resolution)) return -1;

    if (g_strcmp0(codec, "h264") && g_strcmp0(codec, "h265") && g_strcmp0(codec, "hevc")) {
        g_printerr("Invalid codec: %s. Use h264 | h265 | hevc\n", codec);
        return -1;
    }

    g_print("=== Configuration ===\n");
    g_print("Input device: %s\n", in);
    g_print("Resolution : %dx%d (%s)\n", v_width, v_height, input_resolution);
    g_print("FPS        : %s\n", fps);
    g_print("Codec      : %s\n", codec);
    g_print("Bitrate    : %d kbps\n", bitrate);
    g_print("Processing : Async FPGA Y-equalize (NV12)\n");
    g_print("=====================\n\n");

    // OpenCL init
    CustomData data{};
    try {
        std::vector<cl::Device> devices = xcl::get_xil_devices();
        if (devices.empty()) {
            g_printerr("No Xilinx devices found\n");
            return -1;
        }
        cl::Device device = devices[0];
        cl::Context context(device);
        cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE | CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE);

        std::string device_name = device.getInfo<CL_DEVICE_NAME>();
        std::string binaryFile = xcl::find_binary_file(device_name, "krnl_hist_equalize");
        cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
        devices.resize(1);
        cl::Program program(context, devices, bins);
        cl::Kernel krnl(program, "equalizeHist_accel");

        data.q = q;
        data.krnl = krnl;
        data.context = context;

        std::cout << "Input Image Bit Depth: " << XF_DTPIXELDEPTH(IN_TYPE, NPPCX) << std::endl;
        std::cout << "Input Image Channels : " << XF_CHANNELS(IN_TYPE, NPPCX) << std::endl;
        std::cout << "NPPC                 : " << NPPCX << std::endl;

        g_print("OpenCL initialization completed\n");
    } catch (const GError& e) {
    g_printerr("OpenCL initialization error");
  }catch (const GError& e) {
    g_printerr("OpenCL initialization error");
  }

    // Worker thread
    data.processing_thread_running = true;
    data.processing_thread = std::thread(processing_thread_func, &data);

    guint target_bitrate_kbps = (guint)bitrate;

    // ---- appsink pipeline (CPU-mappable buffers; drop oldest) ----
    gchar *pipeline_str = g_strdup_printf(
        "v4l2src device=%s do-timestamp=true io-mode=2 ! "                                  // mmap
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=%s/1 ! "
        "videorate drop-only=true max-rate=%s ! "
        "queue name=qsink max-size-buffers=2 leaky=upstream ! "                             // drop oldest
        "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true",                   // keep only latest
        in, v_width, v_height, fps, fps);

    GError *perr = NULL;
    GstElement *app_sink_pipeline = gst_parse_launch(pipeline_str, &perr);
    g_free(pipeline_str);
    if (!app_sink_pipeline) {
        g_printerr("Create appsink pipeline failed: %s\n", perr ? perr->message : "unknown");
        if (perr) g_clear_error(&perr);
        data.processing_thread_running = false;
        data.queue_cv.notify_all();
        if (data.processing_thread.joinable()) data.processing_thread.join();
        return -1;
    }

    // ---- appsrc pipeline (RTP) ----
    gchar *encoder_str = get_encoder_string(codec, target_bitrate_kbps);
    pipeline_str = g_strdup_printf(
        "appsrc name=cv_src format=GST_FORMAT_TIME is-live=true do-timestamp=false "
        "stream-type=0 max-bytes=0 block=false ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=%s/1 ! "
        "queue name=qencin max-size-buffers=8 leaky=upstream ! "                            // drop oldest to encoder
        "%s ! "
        "queue name=qpayout max-size-buffers=4 leaky=upstream ! "
        "multiudpsink clients=192.168.25.69:5004 sync=false async=false",
        v_width, v_height, fps, encoder_str);
    g_free(encoder_str);

    GstElement *app_src_pipeline = gst_parse_launch(pipeline_str, &perr);
    g_free(pipeline_str);
    if (!app_src_pipeline) {
        g_printerr("Create appsrc pipeline failed: %s\n", perr ? perr->message : "unknown");
        if (perr) g_clear_error(&perr);
        data.processing_thread_running = false;
        data.queue_cv.notify_all();
        if (data.processing_thread.joinable()) data.processing_thread.join();
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    data.app_sink   = gst_bin_get_by_name(GST_BIN(app_sink_pipeline), "cv_sink");
    data.app_source = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "cv_src");
    data.rate_timer = g_timer_new(); g_timer_start(data.rate_timer);

    g_object_set(data.app_source,
                 "is-live", TRUE,
                 "do-timestamp", FALSE,   // use our PTS
                 "stream-type", 0,        // GST_APP_STREAM_TYPE_STREAM
                 "max-bytes", 0,          // unlimited
                 "block", FALSE,
                 NULL);

    g_object_set(data.app_sink, "emit-signals", TRUE, NULL);

    if (gst_element_set_state(app_src_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        g_printerr("appsrc pipeline failed to PLAY\n");
        gst_object_unref(app_src_pipeline);
        gst_object_unref(app_sink_pipeline);
        data.processing_thread_running = false;
        data.queue_cv.notify_all();
        if (data.processing_thread.joinable()) data.processing_thread.join();
        return -1;
    }

    g_signal_connect(data.app_sink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    if (gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        g_printerr("appsink pipeline failed to PLAY\n");
        gst_element_set_state(app_src_pipeline, GST_STATE_NULL);
        gst_object_unref(app_src_pipeline);
        gst_object_unref(app_sink_pipeline);
        data.processing_thread_running = false;
        data.queue_cv.notify_all();
        if (data.processing_thread.joinable()) data.processing_thread.join();
        return -1;
    }

    g_print("\nRTP app ready. Streaming %s @ %sx% s to 192.168.25.69:5004\n", codec, fps, fps);
    g_print("Ctrl+C to stop…\n\n");

    // Wait on appsink bus for EOS/ERROR
    GstBus *bus = gst_element_get_bus(app_sink_pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (msg) {
        GError *err = NULL; gchar *dbg = NULL;
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("Pipeline error: %s\n", err->message);
            g_error_free(err); g_free(dbg);
        } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            g_print("End of stream.\n");
        }
        gst_message_unref(msg);
    }

    // ------------ Teardown ------------
    g_print("\nShutting down…\n");
    data.processing_thread_running = false;
    data.queue_cv.notify_all();
    if (data.processing_thread.joinable()) data.processing_thread.join();

    {   // drain queue safely
        std::lock_guard<std::mutex> lk(data.queue_mutex);
        while (!data.processing_queue.empty()) {
            FrameData fd = data.processing_queue.front();
            data.processing_queue.pop();
            gst_buffer_unref(fd.input_buffer);
            gst_buffer_unref(fd.output_buffer);
        }
    }

    gst_object_unref(bus);
    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_object_unref(app_sink_pipeline);
    gst_element_set_state(app_src_pipeline, GST_STATE_NULL);
    gst_object_unref(app_src_pipeline);

    if (data.buffer_pool) {
        gst_buffer_pool_set_active(data.buffer_pool, FALSE);
        gst_object_unref(data.buffer_pool);
    }

    auto secs = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - data.start_time).count();
    if (secs > 0) {
        double avg = (double)data.frame_count / (double)secs;
        g_print("Final: %d frames in %ld s (%.1f fps avg)\n", data.frame_count, secs, avg);
    }
    return 0;
}
