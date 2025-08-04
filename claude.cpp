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
#include <glib.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
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
static char *input_resolution = "4K"; // New parameter for resolution
static int v_width = 3840;  // Will be set dynamically
static int v_height = 2160; // Will be set dynamically
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

typedef struct {
  GstElement *app_source;
  GstElement *app_sink;
  gboolean video_info_valid;
  GstVideoInfo video_info;

  cl::Context context;
  cl::CommandQueue q;
  cl::Kernel krnl;

  GTimer *rate_timer;
} CustomData;

int counter = 0;

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

struct InterleaveUV_Parallel {
  // Pointers to the start of the output NV12 UV plane, and input U/V planes
  uchar *nv12_uv_plane_ptr;
  const uchar *u_plane_ptr;
  const uchar *v_plane_ptr;

  // Dimensions for calculating offsets
  int uv_plane_width_pixels; // Width of U/V plane in pixels (width / 2)
  size_t u_v_plane_stride;   // Stride of U/V planes in bytes (width / 2)

  // Constructor to initialize member variables
  InterleaveUV_Parallel(uchar *nv12_uv, const uchar *u_data,
                        const uchar *v_data, int uv_width, size_t u_v_stride)
      : nv12_uv_plane_ptr(nv12_uv), u_plane_ptr(u_data), v_plane_ptr(v_data),
        uv_plane_width_pixels(uv_width), u_v_plane_stride(u_v_stride) {}

  // The operator() that will be called by parallel_for_ for each range of rows
  void operator()(const cv::Range &range) const {
    // Iterate over the rows assigned to this thread
    for (int row = range.start; row < range.end; ++row) {
      // Calculate starting pointers for the current row in each plane
      const uchar *u_row_ptr = u_plane_ptr + (row * u_v_plane_stride);
      const uchar *v_row_ptr = v_plane_ptr + (row * u_v_plane_stride);
      // NV12 UV plane has 2 bytes per U/V pixel (U then V)
      uchar *nv12_uv_row_ptr =
          nv12_uv_plane_ptr + (row * uv_plane_width_pixels * 2);

      // Interleave U and V components for the current row
      for (int col = 0; col < uv_plane_width_pixels; ++col) {
        nv12_uv_row_ptr[2 * col] = u_row_ptr[col];     // U component
        nv12_uv_row_ptr[2 * col + 1] = v_row_ptr[col]; // V component
      }
    }
  }
};

// Called when appsink has a new sample
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
  CustomData *data = (CustomData *)user_data;
  GstSample *sample = NULL;
  GstBuffer *buffer = NULL;
  GstBuffer *processed_buffer = NULL;
  GstMapInfo map_info;
  GstMapInfo processed_map_info;
  GstCaps *caps = NULL;
  GstFlowReturn ret = GST_FLOW_OK;

  cv::Mat converted_frame;

  gdouble elapsed_time = g_timer_elapsed(data->rate_timer, NULL);
  g_print("Elapsed time %.2f    Counter: %d\n", elapsed_time, counter++);

  // Pull the sample
  sample = gst_app_sink_pull_sample(appsink);

  if (!sample) {
    g_warning("Failed to pull sample from appsink");
    return GST_FLOW_ERROR;
  }

  // Get buffer and caps
  buffer = gst_sample_get_buffer(sample);
  caps = gst_sample_get_caps(sample);
  if (!buffer || !caps) {
    g_warning("Failed to get buffer or caps from sample");
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  // Store video info if not already done (or if caps change)
  if (!data->video_info_valid) {
    if (gst_video_info_from_caps(&data->video_info, caps)) {
      data->video_info_valid = TRUE;
      g_print("Received initial caps: %s\n", gst_caps_to_string(caps));
    } else {
      g_warning("Failed to parse video info from caps");
    }
  }

  // Map the *incoming* buffer for reading
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
    g_warning("Failed to map input buffer for reading");
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  // convert the incoming video into BGR format
  if (GST_VIDEO_INFO_FORMAT(&data->video_info) == GST_VIDEO_FORMAT_YUY2) {
    cv::Mat yuy2_image_input =
        cv::Mat(data->video_info.height, data->video_info.width, CV_8UC2,
                map_info.data);
    cv::cvtColor(yuy2_image_input, converted_frame, cv::COLOR_YUV2BGR_YUY2);
  } else if (GST_VIDEO_INFO_FORMAT(&data->video_info) ==
             GST_VIDEO_FORMAT_NV12) {
    // NV12 format has Y plane first, then interleaved UV plane
    cv::Mat nv12_image_input =
        cv::Mat(data->video_info.height + data->video_info.height / 2,
                data->video_info.width, CV_8UC1, map_info.data);
    cv::cvtColor(nv12_image_input, converted_frame, cv::COLOR_YUV2BGR_NV12);
  } else if (GST_VIDEO_INFO_FORMAT(&data->video_info) ==
             GST_VIDEO_FORMAT_I420) {
    // I420 format has Y plane, then U plane, then V plane
    cv::Mat i420_image_input =
        cv::Mat(data->video_info.height + data->video_info.height / 2,
                data->video_info.width, CV_8UC1, map_info.data);
    cv::cvtColor(i420_image_input, converted_frame, cv::COLOR_YUV2BGR_I420);
  } else if (GST_VIDEO_INFO_FORMAT(&data->video_info) == GST_VIDEO_FORMAT_BGR) {
    converted_frame =
        cv::Mat(data->video_info.height, data->video_info.width, CV_8UC3,
                map_info.data, data->video_info.stride[0]);
  } else {
    g_print(
        "Unsupported GStreamer format: %s\n",
        gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&data->video_info)));
    gst_caps_unref(caps);
    gst_buffer_unmap(buffer, &map_info);
    ret = GST_FLOW_ERROR;
    return ret;
  }

  // --- OpenCV Processing ---
  int width = GST_VIDEO_INFO_WIDTH(&data->video_info);
  int height = GST_VIDEO_INFO_HEIGHT(&data->video_info);
  gsize stride = GST_VIDEO_INFO_PLANE_STRIDE(&data->video_info, 0);

#ifdef __cplusplus
  try {
    const cv::Mat input_frame(height, width, CV_IN_TYPE, converted_frame.data);
    cv::Mat in_img_copy, out_img;

    // create memory for output images
    input_frame.copyTo(in_img_copy);
    out_img.create(height, width, CV_OUT_TYPE);
    /////////////////////////////////////// CL ///////////////////////////

    cl::Buffer imageToDevice1(data->context, CL_MEM_READ_ONLY,
                              height * width * CHANNEL_TYPE_3);
    cl::Buffer imageToDevice2(data->context, CL_MEM_READ_ONLY,
                              height * width * CHANNEL_TYPE_3);
    cl::Buffer imageFromDevice(data->context, CL_MEM_WRITE_ONLY,
                               height * width * CHANNEL_TYPE_3);

    // Set the kernel arguments
    data->krnl.setArg(0, imageToDevice1);
    data->krnl.setArg(1, imageToDevice2);
    data->krnl.setArg(2, imageFromDevice);
    data->krnl.setArg(3, height);
    data->krnl.setArg(4, width);

    data->q.enqueueWriteBuffer(imageToDevice1, CL_TRUE, 0,
                               height * width * CHANNEL_TYPE_3,
                               input_frame.data);
    data->q.enqueueWriteBuffer(imageToDevice2, CL_TRUE, 0,
                               height * width * CHANNEL_TYPE_3,
                               in_img_copy.data);

    cl::Event event_sp;

    // Launch the kernel
    data->q.enqueueTask(data->krnl, NULL, &event_sp);
    clWaitForEvents(1, (const cl_event *)&event_sp);

    data->q.enqueueReadBuffer(imageFromDevice, CL_TRUE, 0,
                              height * width * CHANNEL_TYPE_3, out_img.data);
    data->q.finish();

    // --- Create a NEW GstBuffer for the processed data ---
    cv::Mat yuv_i420;
    cv::cvtColor(out_img, yuv_i420, cv::COLOR_BGR2YUV_I420);

    // Extract Y, U, V plane data pointers from the I420 Mat
    // Y plane is (width x height)
    uchar *y_data = yuv_i420.data;
    // U plane is (width/2 x height/2), starts after Y plane
    uchar *u_data = yuv_i420.data + (width * height);
    // V plane is (width/2 x height/2), starts after U plane
    uchar *v_data =
        yuv_i420.data + (width * height) + ((width / 2) * (height / 2));

    cv::Mat nv12_image = cv::Mat(height * 3 / 2, width, CV_8UC1);

    memcpy(nv12_image.data, y_data, width * height);

    // Get pointer to the start of the NV12 UV plane
    uchar *nv12_uv_plane_start_ptr = nv12_image.data + (width * height);

    // multithreading
    int uv_plane_height_pixels = height / 2;
    int uv_plane_width_pixels = width / 2;
    size_t u_v_plane_stride = width / 2;

    // Call parallel_for_ to distribute the rows of the UV plane among threads
    cv::parallel_for_(cv::Range(0, uv_plane_height_pixels),
                      InterleaveUV_Parallel(nv12_uv_plane_start_ptr, u_data,
                                            v_data, uv_plane_width_pixels,
                                            u_v_plane_stride));

    gsize processed_size = width * height * 1.5;
    processed_buffer = gst_buffer_new_allocate(NULL, processed_size, NULL);

    if (!processed_buffer) {
      g_warning("Failed to allocate output buffer");
      ret = GST_FLOW_ERROR;
    } else if (gst_buffer_map(processed_buffer, &processed_map_info,
                              GST_MAP_WRITE)) {

      memcpy(processed_map_info.data, nv12_image.data, processed_size);
      gst_buffer_unmap(processed_buffer, &processed_map_info);

      gst_buffer_copy_into(processed_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS,
                           0, -1);
    } else {
      g_warning("Failed to map output buffer for writing");
      gst_buffer_unref(processed_buffer); // Clean up allocated buffer
      processed_buffer = NULL;
      ret = GST_FLOW_ERROR;
    }
  } catch (const cv::Exception &e) {
    g_warning("OpenCV exception: %s", e.what());
    ret = GST_FLOW_ERROR;
  } catch (...) {
    g_warning("Unknown exception during OpenCV processing.");
    ret = GST_FLOW_ERROR;
  }
#endif
  // ------------------------

  gst_buffer_unmap(buffer, &map_info);

  // Push the processed buffer to appsrc (if successfully created)
  if (processed_buffer) {
    GstFlowReturn push_ret = gst_app_src_push_buffer(
        GST_APP_SRC(data->app_source), processed_buffer);
    // processed_buffer ownership is transferred to appsrc, no need to unref
    // here unless push fails

    if (push_ret != GST_FLOW_OK) {
      g_warning("Failed to push buffer to appsrc (%d)", push_ret);
      gst_buffer_unref(processed_buffer);
      ret = push_ret; // Propagate the error
    }
  } else {
    g_warning("No processed buffer to push.");
    if (ret == GST_FLOW_OK)
      ret =
          GST_FLOW_ERROR; // Ensure we signal an error if buffer wasn't created
  }

  gst_sample_unref(sample);

  return ret;
}

int main(int argc, char *argv[]) {
  gst_init(&argc, &argv);

  GOptionContext *optctx;
  GError *error = NULL;
  GstElement *app_sink_pipeline;
  GstElement *app_src_pipeline;
  CustomData data;

  // Initialize custom data
  memset(&data, 0, sizeof(data));
  data.video_info_valid = FALSE;

  optctx = g_option_context_new("- GStreamer Video Processing Application");
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

  std::vector<cl::Device> devices = xcl::get_xil_devices();
  cl::Device device = devices[0];
  cl::Context context(device);

  cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE);

  std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPPCX)
            << std::endl;
  std::cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPPCX)
            << std::endl;
  std::cout << "NPPC:" << NPPCX << std::endl;

  std::string device_name = device.getInfo<CL_DEVICE_NAME>();
  std::string binaryFile =
      xcl::find_binary_file(device_name, "krnl_hist_equalize");
  cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
  devices.resize(1);
  cl::Program program(context, devices, bins);
  cl::Kernel krnl(program, "equalizeHist_accel");

  data.q = q;
  data.krnl = krnl;
  data.context = context;

  guint target_bitrate_kbps = bitrate;
  guint max_bitrate_bps =
      bitrate * 95 / 100 * 1000; // e.g. 5000 * 0.95 * 1000 = 4750000 bps

  // app sink pipeline
  gchar *pipeline_str =
      g_strdup_printf("v4l2src device=%s do-timestamp=false io-mode=4 ! "
                      "video/x-raw, width=%d, height=%d, framerate=%s/1 ! "
                      "videorate drop-only=true max-rate=60 ! appsink "
                      "name=cv_sink emit-signals=true max-buffers=1 drop=true",
                      in, v_width, v_height, fps);
  app_sink_pipeline = gst_parse_launch(pipeline_str, &error);
  g_free(pipeline_str);
  if (!app_sink_pipeline) {
    g_printerr("Failed to create appsink: %s\n", error->message);
    g_clear_error(&error);
    return -1;
  }

  // app src pipeline
  pipeline_str = g_strdup_printf(
      "appsrc name=cv_src format=GST_FORMAT_TIME ! video/x-raw, format=NV12, "
      "width=%d, height=%d, framerate=%s/1 ! "
      "queue ! omxh264enc target-bitrate=%u num-slices=1 "
      "control-rate=Constant qp-mode=fixed prefetch-buffer=true "
      "cpb-size=200 initial-delay=200 "
      "gdr-mode=disabled periodicity-idr=30 gop-length=30 filler-data=true ! "
      "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
      "rtph264pay mtu=1400 ! "
      "queue max-size-buffers=2  ! "
      "udpsink clients=192.168.25.69:5004 auto-multicast=false",
      v_width, v_height, fps, target_bitrate_kbps);
  app_src_pipeline = gst_parse_launch(pipeline_str, &error);
  g_free(pipeline_str);
  if (!app_src_pipeline) {
    g_printerr("Failed to create appsrc: %s\n", error->message);
    g_clear_error(&error);
    return -1;
  }

  data.video_info_valid = FALSE;
  data.app_sink = gst_bin_get_by_name(GST_BIN(app_sink_pipeline), "cv_sink");
  data.app_source = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "cv_src");
  data.rate_timer = g_timer_new();
  g_timer_start(data.rate_timer);

  g_object_set(data.app_sink, "emit-signals", TRUE, NULL);

  GstStateChangeReturn ret =
      gst_element_set_state(app_src_pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the appsrc to the playing state.\n");
    gst_object_unref(app_src_pipeline);
    return -1;
  }
  g_signal_connect(data.app_sink, "new-sample", G_CALLBACK(new_sample_cb),
                   &data);

  ret = gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the appsink to the playing state.\n");
    gst_object_unref(app_sink_pipeline);
    return -1;
  }

  g_print("\nRTP Application is ready.\n");
  GstBus *bus = gst_element_get_bus(app_sink_pipeline);
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

  gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
  gst_object_unref(app_sink_pipeline);
  gst_element_set_state(app_src_pipeline, GST_STATE_NULL);
  gst_object_unref(app_src_pipeline);
  return 0;
}
}