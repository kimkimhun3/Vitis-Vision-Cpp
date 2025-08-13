#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <iostream>

// Structure to hold all our information, so we can pass it to callbacks
typedef struct _CustomData {
  GstElement *capture_pipeline;
  GstElement *stream_pipeline;
  GstElement *app_source;
  GMainLoop *main_loop;
} CustomData;

/**
 * @brief This callback function is called when new data is available from the appsink.
 * * This function is the core of the application. It retrieves the video sample
 * from the capture pipeline's appsink and immediately pushes it to the
 * streaming pipeline's appsrc. This forwarding is very efficient as it
 * avoids memory copies of the video buffer itself.
 * * @param sink The GstAppSink element that triggered the signal.
 * @param data A pointer to our custom data structure.
 * @return GstFlowReturn GST_FLOW_OK on success, or an error code.
 */
static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer data) {
  GstSample *sample;
  CustomData *custom_data = (CustomData *)data;
  GstFlowReturn ret;

  // Pull the sample from the appsink
  sample = gst_app_sink_pull_sample(sink);
  if (sample) {
    // Push the sample into the appsrc of the second pipeline.
    // This function will automatically handle buffer timestamping.
    ret = gst_app_src_push_sample(GST_APP_SRC(custom_data->app_source), sample);
    
    // The sample is no longer needed, so we unreference it
    gst_sample_unref(sample);

    if (ret != GST_FLOW_OK) {
        g_printerr("Failed to push sample to appsrc.\n");
        return ret;
    }
  }

  return GST_FLOW_OK;
}

/**
 * @brief This function is called when an error message is posted on the bus.
 */
static void error_cb(GstBus *bus, GstMessage *msg, CustomData *data) {
    GError *err;
    gchar *debug_info;

    gst_message_parse_error(msg, &err, &debug_info);
    g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
    g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
    g_clear_error(&err);
    g_free(debug_info);

    g_main_loop_quit(data->main_loop);
}

int main(int argc, char *argv[]) {
  CustomData data;
  GstBus *bus_capture, *bus_stream;
  GstElement *app_sink;

  // 1. Initialize GStreamer
  gst_init(&argc, &argv);

  // 2. Define the pipelines
  // Capture Pipeline: v4l2src -> capsfilter -> appsink
  // We use 'v4l2src' for capturing from a V4L2 device (like a camera).
  // The 'appsink' element is configured to emit signals so our callback is triggered.
  const char *capture_pipeline_str = 
    "v4l2src device=/dev/video0 ! "
    "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
    "appsink name=receiversink emit-signals=true sync=false";

  // Stream Pipeline: appsrc -> capsfilter -> encoder -> payloader -> udpsink
  // 'appsrc' is where we will feed the video data from the first pipeline.
  // 'omxh264enc' is a hardware-accelerated H.264 encoder.
  // 'rtph264pay' packetizes the H.264 stream into RTP packets.
  // 'udpsink' sends the RTP packets over the network to the specified client.
  const char *stream_pipeline_str = 
    "appsrc name=sendersrc format=time ! "
    "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
    "omxh264enc ! "
    "video/x-h264,stream-format=byte-stream ! "
    "h264parse ! "
    "rtph264pay config-interval=-1 ! "
    "udpsink host=192.168.25.69 port=5004 sync=false";

  // 3. Create the pipeline elements
  data.capture_pipeline = gst_parse_launch(capture_pipeline_str, NULL);
  data.stream_pipeline = gst_parse_launch(stream_pipeline_str, NULL);

  if (!data.capture_pipeline || !data.stream_pipeline) {
    g_printerr("One or both pipelines could not be created. Exiting.\n");
    return -1;
  }

  // 4. Get the appsink and appsrc elements by name
  app_sink = gst_bin_get_by_name(GST_BIN(data.capture_pipeline), "receiversink");
  data.app_source = gst_bin_get_by_name(GST_BIN(data.stream_pipeline), "sendersrc");

  if (!app_sink || !data.app_source) {
      g_printerr("Could not retrieve app sink or app source. Exiting.\n");
      return -1;
  }

  // 5. Configure the appsink
  // We set the 'emit-signals' property to TRUE to receive the 'new-sample' signal.
  g_object_set(app_sink, "emit-signals", TRUE, NULL);
  g_signal_connect(app_sink, "new-sample", G_CALLBACK(on_new_sample), &data);

  // 6. Create a GMainLoop
  data.main_loop = g_main_loop_new(NULL, FALSE);

  // 7. Add message handlers for errors
  bus_capture = gst_element_get_bus(data.capture_pipeline);
  gst_bus_add_watch(bus_capture, (GstBusFunc)error_cb, &data);
  gst_object_unref(bus_capture);

  bus_stream = gst_element_get_bus(data.stream_pipeline);
  gst_bus_add_watch(bus_stream, (GstBusFunc)error_cb, &data);
  gst_object_unref(bus_stream);
  
  // 8. Start playing both pipelines
  std::cout << "Starting pipelines..." << std::endl;
  gst_element_set_state(data.capture_pipeline, GST_STATE_PLAYING);
  gst_element_set_state(data.stream_pipeline, GST_STATE_PLAYING);

  // 9. Run the GMainLoop
  // This will block until g_main_loop_quit() is called.
  std::cout << "Running... Press Ctrl+C to stop." << std::endl;
  g_main_loop_run(data.main_loop);

  // 10. Cleanup
  std::cout << "Stopping pipelines..." << std::endl;
  gst_object_unref(app_sink);
  gst_object_unref(data.app_source);
  gst_element_set_state(data.capture_pipeline, GST_STATE_NULL);
  gst_element_set_state(data.stream_pipeline, GST_STATE_NULL);
  gst_object_unref(data.capture_pipeline);
  gst_object_unref(data.stream_pipeline);
  g_main_loop_unref(data.main_loop);

  std::cout << "Cleanup complete. Exiting." << std::endl;
  return 0;
}
