#include "common/xf_headers.hpp"
#include "common/xf_params.hpp"
#include "xcl2.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>

#include <cstring>
#include <vector>
#include <iostream>

// ---------------- CLI / options ----------------
#define DEFAULT_RTSP_PORT "5000"
#define DEFAULT_DISABLE_RTCP FALSE

static char *port = (char *)DEFAULT_RTSP_PORT;
static gboolean disable_rtcp = DEFAULT_DISABLE_RTCP;
static char *out = (char*)"h264";
static int bitrate = 6000;
static char *fps = (char*)"60";
static char *in = (char*)"/dev/video0";
static char *input_resolution = (char*)"4K";
static int v_width = 3840;
static int v_height = 2160;
static int k = 4;

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

static gboolean set_resolution_from_input(const char* input_res) {
    if (g_strcmp0(input_res, "4K") == 0) {
        v_width = 3840; v_height = 2160;
        g_print("Selected 4K resolution: %dx%d\n", v_width, v_height);
        return TRUE;
    } else if (g_strcmp0(input_res, "2K") == 0) {
        v_width = 1920; v_height = 1080;
        g_print("Selected 2K resolution: %dx%d\n", v_width, v_height);
        return TRUE;
    } else {
        g_printerr("Invalid input resolution: %s. Supported values: 2K, 4K\n", input_res);
        return FALSE;
    }
}

// Row-wise copy helper (handles arbitrary strides)
static inline void copy_plane_rows(uint8_t* dst, int dst_stride,
                                   const uint8_t* src, int src_stride,
                                   int row_bytes, int rows) {
  for (int r = 0; r < rows; ++r) {
    std::memcpy(dst + (size_t)r*dst_stride, src + (size_t)r*src_stride, (size_t)row_bytes);
  }
}

// ---------------- App state ----------------
typedef struct {
  GstElement *app_source = nullptr;
  GstElement *app_sink   = nullptr;
  gboolean video_info_valid = FALSE;
  GstVideoInfo video_info{};

  cl::Context context;
  cl::CommandQueue q;
  cl::Kernel krnl;
} CustomData;

// ---------------- Appsink callback ----------------
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("appsink: failed to pull sample\n");
        return GST_FLOW_ERROR;
    }
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("appsink: sample has no buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Parse caps once
    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Video info: %dx%d %s\n",
                    data->video_info.width, data->video_info.height,
                    gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&data->video_info)));
        } else {
            g_printerr("Failed to extract GstVideoInfo\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }
    const int width  = data->video_info.width;
    const int height = data->video_info.height;

    // Map input buffer (system memory)
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
        g_printerr("Failed to map input buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Use GstVideoMeta offsets/strides when available
    const GstVideoMeta *vmeta = gst_buffer_get_video_meta(buffer);
    const bool have_meta = (vmeta != nullptr);

    const uint8_t* src_y  = nullptr;
    const uint8_t* src_uv = nullptr;
    int src_y_stride  = width;
    int src_uv_stride = width;

    if (have_meta) {
        src_y        = map_info.data + vmeta->offset[0];
        src_uv       = map_info.data + vmeta->offset[1];
        src_y_stride = vmeta->stride[0];
        src_uv_stride= vmeta->stride[1];
    } else {
        // tightly packed fallback
        src_y  = map_info.data;
        src_uv = map_info.data + (size_t)width * (size_t)height;
    }

    // Build contiguous Y (W*H) for the kernel
    const size_t y_size = (size_t)width * (size_t)height;
    std::vector<uint8_t> y_in(y_size);
    copy_plane_rows(y_in.data(), width, src_y, src_y_stride, width, height);

    // Output Y (equalized)
    std::vector<uint8_t> y_out(y_size);

    // ---- Run FPGA kernel: equalize Y only (single-port kernel, 4 args) ----
    try {
        cl::Buffer dIn (data->context, CL_MEM_READ_ONLY,  y_size);
        cl::Buffer dOut(data->context, CL_MEM_WRITE_ONLY, y_size);

        // equalizeHist_accel(img_y_in, img_y_out, rows, cols)
        data->krnl.setArg(0, dIn);
        data->krnl.setArg(1, dOut);
        data->krnl.setArg(2, height);
        data->krnl.setArg(3, width);

        data->q.enqueueWriteBuffer(dIn, CL_TRUE, 0, y_size, y_in.data());
        data->q.enqueueTask(data->krnl);
        data->q.finish();
        data->q.enqueueReadBuffer(dOut, CL_TRUE, 0, y_size, y_out.data());
        data->q.finish();
    } catch (const cl::Error &e) {
        g_printerr("OpenCL error: %s (%d)\n", e.what(), e.err());
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    } catch (const std::exception &e) {
        g_printerr("Exception: %s\n", e.what());
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // ---- Rebuild NV12: Y' + original UV (color preserved) ----
    const int dst_y_stride  = width;             // publish tight layout
    const int dst_uv_stride = width;             // NV12 UV plane row bytes = width
    const size_t y_bytes_out  = (size_t)dst_y_stride * (size_t)height;
    const size_t uv_bytes_out = (size_t)dst_uv_stride * (size_t)(height/2);
    const size_t total_bytes  = y_bytes_out + uv_bytes_out;

    GstBuffer *processed = gst_buffer_new_and_alloc(total_bytes);
    if (!processed) {
        g_printerr("Failed to allocate output buffer\n");
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Attach video meta (two planes, tight strides)
    {
      gsize offsets[GST_VIDEO_MAX_PLANES] = { 0, (gsize)y_bytes_out, 0, 0 };
      gint  strides[GST_VIDEO_MAX_PLANES] = { dst_y_stride, dst_uv_stride, 0, 0 };
      gst_buffer_add_video_meta_full(processed, GST_VIDEO_FRAME_FLAG_NONE,
                                     GST_VIDEO_FORMAT_NV12, width, height,
                                     2, offsets, strides);
    }

    // Fill payload
    GstMapInfo out_map;
    if (!gst_buffer_map(processed, &out_map, GST_MAP_WRITE)) {
        g_printerr("Failed to map output buffer\n");
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        gst_buffer_unref(processed);
        return GST_FLOW_ERROR;
    }

    uint8_t *dst_y  = out_map.data;
    uint8_t *dst_uv = out_map.data + y_bytes_out;

    // Y' equalized
    copy_plane_rows(dst_y, dst_y_stride, y_out.data(), width, width, height);

    // UV preserved from source
    copy_plane_rows(dst_uv, dst_uv_stride, src_uv, src_uv_stride, width, height/2);

    gst_buffer_unmap(processed, &out_map);

    // Preserve timestamps from input
    gst_buffer_copy_into(processed, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, (gsize)-1);

    // Push to appsrc
    GstFlowReturn fret = gst_app_src_push_buffer(GST_APP_SRC(data->app_source), processed);
    if (fret != GST_FLOW_OK) {
        g_printerr("appsrc push failed: %d\n", fret);
        gst_buffer_unref(processed);
    }

    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// ---------------- Main ----------------
int main(int argc, char *argv[]) {
  gst_init(&argc, &argv);

  // Parse CLI
  GOptionContext *optctx = g_option_context_new("- Y-only histogram equalization (color preserved)");
  GError *err = NULL;
  g_option_context_add_main_entries(optctx, entries, NULL);
  g_option_context_add_group(optctx, gst_init_get_option_group());
  if (!g_option_context_parse(optctx, &argc, &argv, &err)) {
    g_printerr("Error parsing options: %s\n", err->message);
    g_option_context_free(optctx);
    g_clear_error(&err);
    return -1;
  }
  g_option_context_free(optctx);

  if (!set_resolution_from_input(input_resolution)) return -1;

  g_print("=== Configuration ===\n");
  g_print("Input device: %s\n", in);
  g_print("Resolution: %dx%d (%s)\n", v_width, v_height, input_resolution);
  g_print("FPS: %s\n", fps);
  g_print("Bitrate: %d kbps\n", bitrate);
  g_print("====================\n\n");

  // OpenCL init (single-port kernel)
  CustomData data{};
  try {
    std::vector<cl::Device> devices = xcl::get_xil_devices();
    if (devices.empty()) { g_printerr("No Xilinx devices found\n"); return -1; }
    cl::Device device = devices[0];
    cl::Context context(device);
    cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE);

    std::string device_name = device.getInfo<CL_DEVICE_NAME>();
    std::string binaryFile = xcl::find_binary_file(device_name, "krnl_hist_equalize");
    cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
    devices.resize(1);
    cl::Program program(context, devices, bins);
    cl::Kernel krnl(program, "equalizeHist_accel");  // expects 4 args: in, out, rows, cols

    data.context = context;
    data.q = q;
    data.krnl = krnl;
  } catch (const cl::Error &e) {
    g_printerr("OpenCL init error: %s (%d)\n", e.what(), e.err());
    return -1;
  } catch (const std::exception &e) {
    g_printerr("OpenCL init exception: %s\n", e.what());
    return -1;
  }

  // GStreamer pipelines (system-memory NV12)
  // Input
  gchar *pin_desc = g_strdup_printf(
      "v4l2src device=%s do-timestamp=true io-mode=2 ! "
      "video/x-raw, format=NV12, width=%d, height=%d, framerate=%s/1 ! "
      "videorate drop-only=true max-rate=%s ! "
      "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true sync=false",
      in, v_width, v_height, fps, fps);
  GError *perr = nullptr;
  GstElement *pin = gst_parse_launch(pin_desc, &perr);
  g_free(pin_desc);
  if (!pin) {
    g_printerr("Failed to create input pipeline: %s\n", perr ? perr->message : "?");
    if (perr) g_error_free(perr);
    return -1;
  }

  // Output (OMX encoder)
  gchar *pout_desc = g_strdup_printf(
      "appsrc name=cv_src is-live=true format=time do-timestamp=true ! "
      "video/x-raw, format=NV12, width=%d, height=%d, framerate=%s/1 ! "
      "queue ! omxh264enc target-bitrate=%u num-slices=1 "
      "control-rate=Constant qp-mode=fixed prefetch-buffer=true "
      "cpb-size=200 initial-delay=200 "
      "gdr-mode=disabled periodicity-idr=30 gop-length=30 filler-data=true ! "
      "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
      "rtph264pay mtu=1400 pt=96 config-interval=1 ! "
      "udpsink clients=192.168.25.69:5004 auto-multicast=false sync=false",
      v_width, v_height, fps, (guint)bitrate*1000);
  GstElement *pout = gst_parse_launch(pout_desc, &perr);
  g_free(pout_desc);
  if (!pout) {
    g_printerr("Failed to create output pipeline: %s\n", perr ? perr->message : "?");
    if (perr) g_error_free(perr);
    gst_object_unref(pin);
    return -1;
  }

  data.app_sink   = gst_bin_get_by_name(GST_BIN(pin),  "cv_sink");
  data.app_source = gst_bin_get_by_name(GST_BIN(pout), "cv_src");

  // Enforce caps on appsrc (some OMX variants need explicit caps)
  {
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
      "format",  G_TYPE_STRING, "NV12",
      "width",   G_TYPE_INT,    v_width,
      "height",  G_TYPE_INT,    v_height,
      "framerate", GST_TYPE_FRACTION, atoi(fps), 1, NULL);
    g_object_set(data.app_source, "caps", caps, NULL);
    gst_caps_unref(caps);
  }

  // Hook callback
  g_signal_connect(data.app_sink, "new-sample", G_CALLBACK(new_sample_cb), &data);

  // Start pipelines
  if (gst_element_set_state(pout, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Failed to start output pipeline\n");
    gst_object_unref(pin);
    gst_object_unref(pout);
    return -1;
  }
  if (gst_element_set_state(pin, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Failed to start input pipeline\n");
    gst_element_set_state(pout, GST_STATE_NULL);
    gst_object_unref(pin);
    gst_object_unref(pout);
    return -1;
  }

  g_print("Running... %s %dx%d@%sfps bitrate=%dkbps (Y-only equalize; color preserved; single-port kernel)\n",
          in, v_width, v_height, fps, bitrate);

  // Wait for EOS/ERROR on input
  GstBus *bus = gst_element_get_bus(pin);
  GstMessage *msg = gst_bus_timed_pop_filtered(
      bus, GST_CLOCK_TIME_NONE,
      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  if (msg) {
    GError *e=nullptr; gchar *dbg=nullptr;
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      gst_message_parse_error(msg, &e, &dbg);
      g_printerr("Error received: %s\n", e ? e->message : "?");
      if (e) g_error_free(e);
      if (dbg) g_free(dbg);
    } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
      g_print("End-Of-Stream reached.\n");
    }
    gst_message_unref(msg);
  }
  gst_object_unref(bus);

  gst_element_set_state(pin,  GST_STATE_NULL);
  gst_element_set_state(pout, GST_STATE_NULL);
  if (data.app_sink)   gst_object_unref(data.app_sink);
  if (data.app_source) gst_object_unref(data.app_source);
  gst_object_unref(pin);
  gst_object_unref(pout);
  return 0;
}
