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
 * This function is the core of the application. It retrieves the video sample
 * from the capture pipeline's appsink and immediately pushes it to the
 * streaming pipeline's appsrc. This forwarding is very efficient as it
 * avoids memory copies of the video buffer itself.
 * @param sink The GstAppSink element that triggered the signal.
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
 * @brief This function is called when a message is posted on the bus.
 * We handle different message types here.
 */
static void bus_cb(GstBus *bus, GstMessage *msg, CustomData *data) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug_info;

            // This is the correct way to handle errors.
            // We parse the error message and print it.
            gst_message_parse_error(msg, &err, &debug_info);
            g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
            g_clear_error(&err);
            g_free(debug_info);

            g_main_loop_quit(data->main_loop);
            break;
        }
        case GST_MESSAGE_EOS:
            // Handle End-Of-Stream
            g_print("End-Of-Stream reached on %s.\n", GST_OBJECT_NAME(msg->src));
            // We don't quit the main loop here, as one pipeline might end before the other.
            // The application will be terminated by Ctrl+C or a critical error.
            break;
        case GST_MESSAGE_STATE_CHANGED:
            // We are only interested in state-changed messages from the pipeline
            if (GST_MESSAGE_SRC(msg) == GST_OBJECT(data->capture_pipeline) || GST_MESSAGE_SRC(msg) == GST_OBJECT(data->stream_pipeline)) {
                GstState old_state, new_state, pending_state;
                gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
                g_print("Pipeline '%s' state changed from %s to %s.\n",
                    GST_OBJECT_NAME(msg->src),
                    gst_element_state_get_name(old_state),
                    gst_element_state_get_name(new_state));
            }
            break;
        default:
            // We are not interested in other messages
            break;
    }
}

int main(int argc, char *argv[]) {
  CustomData data;
  GstBus *bus_capture, *bus_stream;
  GstElement *app_sink;

  // 1. Initialize GStreamer
  gst_init(&argc, &argv);

  // 2. Define the pipelines
  // Capture Pipeline: v4l2src -> capsfilter -> appsink
  // We use 'v4l2src' with io-mode=4 (DMABUF) for zero-copy capture.
  // The 'appsink' is tuned to drop frames if the consumer is too slow.
  const char *capture_pipeline_str = 
    "v4l2src device=/dev/video0 io-mode=4 ! "
    "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
    "appsink name=receiversink emit-signals=true max-buffers=2 drop=true sync=false";

  // Stream Pipeline: appsrc -> capsfilter -> encoder -> payloader -> udpsink
  // 'appsrc' is set to block, creating backpressure to the sink.
  // 'omxh264enc' is tuned with properties from the working gst-launch command for low latency and performance.
  // 'udpsink' is tuned for high-throughput streaming.
  const char *stream_pipeline_str = 
    "appsrc name=sendersrc format=time is-live=true block=true ! "
    "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
    "omxh264enc target-bitrate=10000000 num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal initial-delay=250 control-rate=low-latency prefetch-buffer=true gop-mode=low-delay-p ! "
    "video/x-h264,alignment=nal,stream-format=byte-stream ! "
    "h264parse ! "
    "rtph264pay config-interval=-1 ! "
    "udpsink host=192.168.25.69 port=5004 buffer-size=60000000 max-lateness=-1 qos-dscp=60 sync=false";

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

  // 7. Add message handlers for the buses
  bus_capture = gst_element_get_bus(data.capture_pipeline);
  gst_bus_add_watch(bus_capture, (GstBusFunc)bus_cb, &data);
  gst_object_unref(bus_capture);

  bus_stream = gst_element_get_bus(data.stream_pipeline);
  gst_bus_add_watch(bus_stream, (GstBusFunc)bus_cb, &data);
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
  // app_source is part of the pipeline and will be unref'd with it
  gst_element_set_state(data.capture_pipeline, GST_STATE_NULL);
  gst_element_set_state(data.stream_pipeline, GST_STATE_NULL);
  gst_object_unref(data.capture_pipeline);
  gst_object_unref(data.stream_pipeline);
  g_main_loop_unref(data.main_loop);

  std::cout << "Cleanup complete. Exiting." << std::endl;
  return 0;
}
