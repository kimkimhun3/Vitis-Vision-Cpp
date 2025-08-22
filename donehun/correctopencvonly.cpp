// Modified host.cpp - OpenCV histogram equalization instead of FPGA acceleration
#include "common/xf_headers.hpp"
#include "common/xf_params.hpp"
#include "gst/gstpad.h"
#include "gst/gstsample.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/imgcodecs.hpp"
// Remove OpenCL includes
// #include "xcl2.hpp"
// #include "xf_config_params.h"
// #include "xf_hist_equalize_tb_config.h"

#include <cstddef>
#include <unistd.h>
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

  // Remove OpenCL/FPGA members - replace with timing info
  GTimer *processing_timer;
  double total_processing_time;
  int frame_count;

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

// Callback to extract video info when caps are negotiated
static void on_pad_added(GstElement *element, GstPad *pad, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;
    
    GstCaps *caps = gst_pad_get_current_caps(pad);
    if (caps) {
        if (gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Video info extracted: %dx%d\n", 
                   data->video_info.width, data->video_info.height);
        }
        gst_caps_unref(caps);
    }
}

// Called when appsink has a new sample
GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("Failed to pull sample from appsink\n");
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("Failed to get buffer from sample\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Extract video info if not already done
    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Video info extracted from sample: %dx%d\n", 
                   data->video_info.width, data->video_info.height);
        } else {
            g_printerr("Failed to extract video info from sample\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
        g_printerr("Failed to map buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    int width = data->video_info.width;
    int height = data->video_info.height;
    
    // Validate dimensions
    if (width <= 0 || height <= 0) {
        g_printerr("Invalid dimensions: %dx%d\n", width, height);
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Calculate buffer sizes
    size_t y_size = width * height;
    size_t uv_size = width * height / 2;
    
    // Validate buffer size
    if (map_info.size < y_size + uv_size) {
        g_printerr("Buffer size mismatch: expected %zu, got %zu\n", 
                  y_size + uv_size, map_info.size);
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Step 1: Split Y from NV12 input
    cv::Mat nv12_input(height * 3 / 2, width, CV_8UC1, map_info.data);
    cv::Mat y_plane_in = nv12_input(cv::Rect(0, 0, width, height)).clone();
    cv::Mat y_plane_out(height, width, CV_8UC1);

    try {
        // START TIMING - Replace FPGA processing with OpenCV histogram equalization
        g_timer_start(data->processing_timer);
        
        // REPLACE THIS SECTION: Instead of OpenCL/FPGA processing, use OpenCV
        cv::equalizeHist(y_plane_in, y_plane_out);
        
        // END TIMING
        g_timer_stop(data->processing_timer);
        double frame_processing_time = g_timer_elapsed(data->processing_timer, NULL) * 1000.0; // Convert to milliseconds
        data->total_processing_time += frame_processing_time;
        data->frame_count++;
        
        // Print performance stats every 100 frames
        if (data->frame_count % 100 == 0) {
            double avg_processing_time = data->total_processing_time / data->frame_count;
            g_print("OpenCV Processing Stats - Frame %d: Current: %.2f ms, Average: %.2f ms, FPS potential: %.1f\n",
                   data->frame_count, frame_processing_time, avg_processing_time, 1000.0 / avg_processing_time);
        }

        // Step 5: Reconstruct NV12 with equalized Y and neutral UV (same as original)
        cv::Mat nv12_output(height * 3 / 2, width, CV_8UC1);
        memcpy(nv12_output.data, y_plane_out.data, y_size);
        memset(nv12_output.data + y_size, 128, uv_size);

        // Step 6: Push NV12 frame to appsrc (same as original)
        GstBuffer *processed_buffer = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
        if (!processed_buffer) {
            g_printerr("Failed to allocate processed buffer\n");
            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        GstMapInfo processed_map_info;
        if (gst_buffer_map(processed_buffer, &processed_map_info, GST_MAP_WRITE)) {
            memcpy(processed_map_info.data, nv12_output.data, y_size + uv_size);
            gst_buffer_unmap(processed_buffer, &processed_map_info);
            gst_buffer_copy_into(processed_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);
            
            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->app_source), processed_buffer);
            if (ret != GST_FLOW_OK) {
                g_printerr("Failed to push buffer to appsrc: %d\n", ret);
            }
        } else {
            g_printerr("Failed to map processed buffer\n");
            gst_buffer_unref(processed_buffer);
        }
    } catch (const std::exception& e) {
        g_printerr("OpenCV processing error in new_sample_cb: %s\n", e.what());
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
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
  data.processing_timer = g_timer_new();
  data.total_processing_time = 0.0;         //[FPGA: Histogram Equalization]

  data.frame_count = 0;

  optctx = g_option_context_new("- GStreamer Video Processing Application (OpenCV Version)");
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
  g_print("=== Configuration (OpenCV Version) ===\n");
  g_print("Input device: %s\n", in);
  g_print("Resolution: %dx%d (%s)\n", v_width, v_height, input_resolution);
  g_print("FPS: %s\n", fps);
  g_print("Bitrate: %d kbps\n", bitrate);
  g_print("Port: %s\n", port);
  g_print("Processing: OpenCV cv::equalizeHist (CPU)\n");
  g_print("===================HUI===================\n\n");

  // REMOVE OpenCL/FPGA initialization - no longer needed!
  g_print("Using OpenCV histogram equalization instead of FPGA acceleration\n");

  guint target_bitrate_kbps = bitrate;
  guint max_bitrate_bps = bitrate * 95 / 100 * 1000; // e.g. 5000 * 0.95 * 1000 = 4750000 bps

  // app sink pipeline with format specification (SAME AS ORIGINAL)
  gchar *pipeline_str =
      g_strdup_printf("v4l2src device=%s do-timestamp=false io-mode=4 ! "
                      "video/x-raw, format=NV12, width=%d, height=%d, framerate=60/1 ! "
                      "videorate drop-only=true max-rate=%d ! appsink "
                      "name=cv_sink emit-signals=true max-buffers=1 drop=true",
                      in, v_width, v_height, fps);
  app_sink_pipeline = gst_parse_launch(pipeline_str, &error);
  g_free(pipeline_str);
  if (!app_sink_pipeline) {
    g_printerr("Failed to create appsink: %s\n", error->message);
    g_clear_error(&error);
    return -1;
  }

  // app src pipeline (SAME AS ORIGINAL)
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

  // Connect pad-added signal to extract video info
  g_signal_connect(app_sink_pipeline, "pad-added", G_CALLBACK(on_pad_added), &data);

  GstStateChangeReturn ret = gst_element_set_state(app_src_pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the appsrc to the playing state.\n");
    gst_object_unref(app_src_pipeline);
    return -1;
  }
  g_signal_connect(data.app_sink, "new-sample", G_CALLBACK(new_sample_cb), &data);

  ret = gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the appsink to the playing state.\n");
    gst_object_unref(app_sink_pipeline);
    return -1;
  }

  g_print("\nOpenCV RTP Application is ready.\n");
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

  // Print final performance statistics
  if (data.frame_count > 0) {
    double avg_processing_time = data.total_processing_time / data.frame_count;
    g_print("\n=== Final OpenCV Performance Stats ===\n");
    g_print("Total frames processed: %d\n", data.frame_count);
    g_print("Average processing time: %.2f ms per frame\n", avg_processing_time);
    g_print("Theoretical max FPS: %.1f\n", 1000.0 / avg_processing_time);
    g_print("Total processing time: %.2f seconds\n", data.total_processing_time / 1000.0);
    g_print("=====================================\n");
  }

  gst_object_unref(bus);
  g_timer_destroy(data.processing_timer);
  g_timer_destroy(data.rate_timer);

  gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
  gst_object_unref(app_sink_pipeline);
  gst_element_set_state(app_src_pipeline, GST_STATE_NULL);
  gst_object_unref(app_src_pipeline);
  return 0;
}
}