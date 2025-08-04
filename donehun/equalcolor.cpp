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
#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gst/video/video.h>
#include <opencv2/opencv.hpp>

#define DEFAULT_RTSP_PORT "5000"
#define DEFAULT_DISABLE_RTCP FALSE
#define CAMERA_FPS 30

extern "C" {
static char *port = (char *)DEFAULT_RTSP_PORT;
static gboolean disable_rtcp = DEFAULT_DISABLE_RTCP;
static char *out = "h264";
static int bitrate = 6000;
static char *fps = "60";
static char *in = "/dev/video0";
static char *input_resolution = "4K";
static int v_width = 3840;
static int v_height = 2160;
int k = 4;

static GOptionEntry entries[] = {
    {"port", 'p', 0, G_OPTION_ARG_STRING, &port, "RTSP port (default: 5000)", NULL},
    {"in", 'i', 0, G_OPTION_ARG_STRING, &in, "Input device (default: /dev/video0)", NULL},
    {"out", 'o', 0, G_OPTION_ARG_STRING, &out, "Output format (default: h264)", NULL},
    {"bitrate", 'b', 0, G_OPTION_ARG_INT, &bitrate, "Bitrate in kbps (default: 6000)", NULL},
    {"fps", 'f', 0, G_OPTION_ARG_STRING, &fps, "Frames per second (default: 60)", NULL},
    {"input", 'r', 0, G_OPTION_ARG_STRING, &input_resolution, "Input resolution: 2K or 4K (default: 4K)", NULL},
    {"k", 'k', 0, G_OPTION_ARG_INT, &k, "K parameter (default: 4)", NULL},
    {NULL}
};

// Simple pad probe approach - more reliable than identity handoff
typedef struct {
    cl::Context context;
    cl::CommandQueue q;
    cl::Kernel krnl;
    gboolean opencl_initialized;
    GstVideoInfo video_info;
    gboolean video_info_valid;
} ProcessingData;

ProcessingData g_processing_data;

// Function to set resolution based on input parameter
static gboolean set_resolution_from_input(const char* input_res) {
    if (g_strcmp0(input_res, "4K") == 0) {
        v_width = 3840;
        v_height = 2160;
        g_print("Selected 4K resolution: %dx%d\n", v_width, v_height);
        return TRUE;
    } else if (g_strcmp0(input_res, "2K") == 0) {
        v_width = 1920;
        v_height = 1080;
        g_print("Selected 2K resolution: %dx%d\n", v_width, v_height);
        return TRUE;
    } else {
        g_printerr("Invalid input resolution: %s. Supported values: 2K, 4K\n", input_res);
        return FALSE;
    }
}

// Pad probe callback - this ensures we get writable buffers
static GstPadProbeReturn pad_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    ProcessingData *data = (ProcessingData *)user_data;
    
    if (!data->opencl_initialized) {
        return GST_PAD_PROBE_OK;
    }
    
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer) {
        return GST_PAD_PROBE_OK;
    }
    
    // Extract video info if not already done
    if (!data->video_info_valid) {
        GstCaps *caps = gst_pad_get_current_caps(pad);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Video info extracted: %dx%d\n", 
                   data->video_info.width, data->video_info.height);
        }
        if (caps) gst_caps_unref(caps);
        
        if (!data->video_info_valid) {
            g_printerr("Failed to extract video info\n");
            return GST_PAD_PROBE_OK;
        }
    }
    
    // Make buffer writable if it's not already
    if (!gst_buffer_is_writable(buffer)) {
        // Create a writable copy
        buffer = gst_buffer_make_writable(buffer);
        GST_PAD_PROBE_INFO_DATA(info) = buffer;
    }
    
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READWRITE)) {
        g_printerr("Failed to map buffer for read/write\n");
        return GST_PAD_PROBE_OK;
    }
    
    int width = data->video_info.width;
    int height = data->video_info.height;
    size_t y_size = width * height;
    size_t uv_size = width * height / 2;
    
    // Validate buffer size
    if (map_info.size < y_size + uv_size) {
        g_printerr("Buffer size mismatch: expected %zu, got %zu\n", 
                  y_size + uv_size, map_info.size);
        gst_buffer_unmap(buffer, &map_info);
        return GST_PAD_PROBE_OK;
    }
    
    try {
        // Extract Y plane from NV12
        cv::Mat nv12_input(height * 3 / 2, width, CV_8UC1, map_info.data);
        cv::Mat y_plane_in = nv12_input(cv::Rect(0, 0, width, height)).clone();
        cv::Mat y_plane_out(height, width, CV_8UC1);
        
        // Allocate device buffers
        cl::Buffer imageToDeviceY1(data->context, CL_MEM_READ_ONLY, y_size);
        cl::Buffer imageToDeviceY2(data->context, CL_MEM_READ_ONLY, y_size);
        cl::Buffer imageFromDevice(data->context, CL_MEM_WRITE_ONLY, y_size);
        
        // Set kernel arguments
        data->krnl.setArg(0, imageToDeviceY1);
        data->krnl.setArg(1, imageToDeviceY2);
        data->krnl.setArg(2, imageFromDevice);
        data->krnl.setArg(3, height);
        data->krnl.setArg(4, width);
        
        // Transfer data to device and run kernel
        data->q.enqueueWriteBuffer(imageToDeviceY1, CL_TRUE, 0, y_size, y_plane_in.data);
        data->q.enqueueWriteBuffer(imageToDeviceY2, CL_TRUE, 0, y_size, y_plane_in.data);
        data->q.enqueueTask(data->krnl);
        data->q.finish();
        data->q.enqueueReadBuffer(imageFromDevice, CL_TRUE, 0, y_size, y_plane_out.data);
        data->q.finish();
        
        // Write equalized Y plane back to buffer (in-place modification)
        memcpy((guint8*)map_info.data, y_plane_out.data, y_size);
        // UV plane remains unchanged at map_info.data + y_size
        
    } catch (const GError& e) {
    g_printerr("OpenCL initialization error");
  }
    
    gst_buffer_unmap(buffer, &map_info);
    return GST_PAD_PROBE_OK;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    
    GOptionContext *optctx;
    GError *error = NULL;
    GstElement *pipeline;
    GstElement *queue_element;
    GstPad *queue_src_pad;
    
    // Initialize processing data
    memset(&g_processing_data, 0, sizeof(g_processing_data));
    g_processing_data.opencl_initialized = FALSE;
    g_processing_data.video_info_valid = FALSE;
    
    optctx = g_option_context_new("- GStreamer Single Pipeline Video Processing");
    g_option_context_add_main_entries(optctx, entries, NULL);
    g_option_context_add_group(optctx, gst_init_get_option_group());
    if (!g_option_context_parse(optctx, &argc, &argv, &error)) {
        g_printerr("Error parsing options: %s\n", error->message);
        g_option_context_free(optctx);
        g_clear_error(&error);
        return -1;
    }
    g_option_context_free(optctx);
    
    // Set resolution based on input parameter
    if (!set_resolution_from_input(input_resolution)) {
        return -1;
    }
    
    // Print configuration
    g_print("=== Configuration ===\n");
    g_print("Input device: %s\n", in);
    g_print("Resolution: %dx%d (%s)\n", v_width, v_height, input_resolution);
    g_print("FPS: %s\n", fps);
    g_print("Bitrate: %d kbps\n", bitrate);
    g_print("Port: %s\n", port);
    g_print("====================\n\n");
    
    // Initialize OpenCL/FPGA
    try {
        std::vector<cl::Device> devices = xcl::get_xil_devices();
        if (devices.empty()) {
            g_printerr("No Xilinx devices found\n");
            return -1;
        }
        
        cl::Device device = devices[0];
        g_processing_data.context = cl::Context(device);
        g_processing_data.q = cl::CommandQueue(g_processing_data.context, device, CL_QUEUE_PROFILING_ENABLE);
        
        std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPPCX) << std::endl;
        std::cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPPCX) << std::endl;
        std::cout << "NPPC:" << NPPCX << std::endl;
        
        std::string device_name = device.getInfo<CL_DEVICE_NAME>();
        std::string binaryFile = xcl::find_binary_file(device_name, "krnl_hist_equalize");
        cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
        devices.resize(1);
        cl::Program program(g_processing_data.context, devices, bins);
        g_processing_data.krnl = cl::Kernel(program, "equalizeHist_accel");
        
        g_processing_data.opencl_initialized = TRUE;
        
        g_print("OpenCL/FPGA initialized successfully\n");
        
    } 
    catch (const GError& e) {
    g_printerr("OpenCL initialization error");
    return -1;
  }
    
    // Create single pipeline with queue element for pad probe
    gchar *pipeline_str = g_strdup_printf(
        "v4l2src device=%s do-timestamp=false io-mode=4 ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=%s/1 ! "
        "videorate drop-only=true max-rate=60 ! "
        "queue name=processor max-size-buffers=2 ! "  // Queue for pad probe
        "omxh264enc target-bitrate=%u num-slices=1 "
        "control-rate=Constant qp-mode=fixed prefetch-buffer=true "
        "cpb-size=200 initial-delay=200 "
        "gdr-mode=disabled periodicity-idr=30 gop-length=30 filler-data=true ! "
        "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
        "rtph264pay mtu=1400 ! "
        "queue max-size-buffers=2 ! "
        "udpsink clients=192.168.25.69:5004 auto-multicast=false",
        in, v_width, v_height, fps, (guint)bitrate);
    
    pipeline = gst_parse_launch(pipeline_str, &error);
    g_free(pipeline_str);
    
    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }
    
    // Get the queue element and add pad probe on its source pad
    queue_element = gst_bin_get_by_name(GST_BIN(pipeline), "processor");
    if (!queue_element) {
        g_printerr("Failed to get queue element\n");
        gst_object_unref(pipeline);
        return -1;
    }
    
    // Get the source pad of the queue element
    queue_src_pad = gst_element_get_static_pad(queue_element, "src");
    if (!queue_src_pad) {
        g_printerr("Failed to get queue source pad\n");
        gst_object_unref(queue_element);
        gst_object_unref(pipeline);
        return -1;
    }
    
    // Add pad probe for processing
    gst_pad_add_probe(queue_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                     pad_probe_cb, &g_processing_data, NULL);
    
    gst_object_unref(queue_src_pad);
    gst_object_unref(queue_element);
    
    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return -1;
    }
    
    g_print("\nSingle Pipeline Application is ready.\n");
    g_print("Pipeline: v4l2src -> videorate -> queue(FPGA pad probe) -> omxh264enc -> rtph264pay -> udpsink\n\n");
    
    // Wait for error or EOS
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus, GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    
    if (msg != nullptr) {
        GError *err;
        gchar *debug_info;
        
        switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(msg, &err, &debug_info);
            g_printerr("Error received: %s\n", err->message);
            g_error_free(err);
            g_free(debug_info);
            break;
        case GST_MESSAGE_EOS:
            g_print("End-Of-Stream reached.\n");
            break;
        default:
            break;
        }
        gst_message_unref(msg);
    }
    
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}
}