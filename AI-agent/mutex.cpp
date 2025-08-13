// Build:
//   g++ -O3 -std=c++17 gstreamer_appsink_appsrc_relay_h264_udp.cpp -o relay \
//       `pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0`
// Run examples:
//   ./relay --fps 30 --bitrate 10000
//   ./relay --fps 60 --bitrate 10000
// Notes:
// - Captures NV12 frames via v4l2src -> appsink, forwards buffers to appsrc with zero-copy.
// - Encodes with omxh264enc using target-bitrate in kbps.
// - Streams as RTP/H264 to a UDP destination using multiudpsink with default host=192.168.25.69 and port=5004.

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

struct Options {
  std::string device = "/dev/video0";
  int width = 1920;
  int height = 1080;
  int fps = 60; // Test with 30 or 60
  int bitrate = 10000; // in kbps
  std::string host = "192.168.25.69";
  int port = 5004;
};

static std::atomic<bool> g_running{true};
static void handle_sigint(int) { g_running = false; }

int main(int argc, char** argv) {
  Options opt;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--device" && i + 1 < argc) opt.device = argv[++i];
    else if (a == "--width" && i + 1 < argc) opt.width = std::stoi(argv[++i]);
    else if (a == "--height" && i + 1 < argc) opt.height = std::stoi(argv[++i]);
    else if (a == "--fps" && i + 1 < argc) opt.fps = std::stoi(argv[++i]);
    else if (a == "--bitrate" && i + 1 < argc) opt.bitrate = std::stoi(argv[++i]);
  }

  gst_init(&argc, &argv);
  std::signal(SIGINT, handle_sigint);

  GstElement *v4l2src = gst_element_factory_make("v4l2src", NULL);
  GstElement *capsfilter = gst_element_factory_make("capsfilter", NULL);
  GstElement *sink = gst_element_factory_make("appsink", NULL);

  GstElement *src = gst_element_factory_make("appsrc", NULL);
  GstElement *q_in = gst_element_factory_make("queue", NULL);
  GstElement *enc = gst_element_factory_make("omxh264enc", NULL);
  GstElement *parse = gst_element_factory_make("h264parse", NULL);
  GstElement *pay = gst_element_factory_make("rtph264pay", NULL);
  GstElement *sink_udp = gst_element_factory_make("multiudpsink", NULL);

  if (!v4l2src || !capsfilter || !sink || !src || !q_in || !enc || !parse || !pay || !sink_udp) {
    std::cerr << "Failed to create GStreamer elements." << std::endl;
    return 1;
  }

  GstElement* pipe_capture = gst_pipeline_new(NULL);
  GstElement* pipe_encode = gst_pipeline_new(NULL);

  g_object_set(v4l2src, "device", opt.device.c_str(), NULL);

  GstCaps* caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, "NV12",
      "width", G_TYPE_INT, opt.width,
      "height", G_TYPE_INT, opt.height,
      "framerate", GST_TYPE_FRACTION, opt.fps, 1,
      NULL);
  g_object_set(capsfilter, "caps", caps, NULL);
  gst_caps_unref(caps);

  g_object_set(sink, "emit-signals", FALSE, "sync", FALSE, "max-buffers", 8, "drop", TRUE, NULL);
  gst_bin_add_many(GST_BIN(pipe_capture), v4l2src, capsfilter, sink, NULL);
  gst_element_link_many(v4l2src, capsfilter, sink, NULL);

  g_object_set(src, "is-live", TRUE, "format", GST_FORMAT_TIME, "block", TRUE, "do-timestamp", TRUE, NULL);
  g_object_set(q_in, "max-size-buffers", 30, "leaky", 2, NULL);

  g_object_set(enc, "target-bitrate", opt.bitrate, NULL);
  g_object_set(pay, "pt", 96, "config-interval", 1, NULL);
  std::string clients = opt.host + ":" + std::to_string(opt.port);
  g_object_set(sink_udp, "clients", clients.c_str(), "async", FALSE, "sync", FALSE, NULL);

  GstCaps* appsrc_caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, "NV12",
      "width", G_TYPE_INT, opt.width,
      "height", G_TYPE_INT, opt.height,
      "framerate", GST_TYPE_FRACTION, opt.fps, 1,
      NULL);
  gst_app_src_set_caps(GST_APP_SRC(src), appsrc_caps);
  gst_caps_unref(appsrc_caps);

  gst_bin_add_many(GST_BIN(pipe_encode), src, q_in, enc, parse, pay, sink_udp, NULL);
  gst_element_link_many(src, q_in, enc, parse, pay, sink_udp, NULL);

  gst_element_set_state(pipe_encode, GST_STATE_PLAYING);
  gst_element_set_state(pipe_capture, GST_STATE_PLAYING);

  std::thread pump([&]() {
    GstAppSink* asink = GST_APP_SINK(sink);
    GstAppSrc* asrc = GST_APP_SRC(src);
    while (g_running.load()) {
      GstSample* sample = gst_app_sink_try_pull_sample(asink, 20 * GST_MSECOND);
      if (!sample) continue;
      GstBuffer* buf = gst_sample_get_buffer(sample);
      if (!buf) { gst_sample_unref(sample); continue; }
      GstBuffer* out = gst_buffer_ref(buf);
      if (gst_app_src_push_buffer(asrc, out) != GST_FLOW_OK) gst_buffer_unref(out);
      gst_sample_unref(sample);
    }
    gst_app_src_end_of_stream(asrc);
  });

  GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
  g_main_loop_run(loop);

  g_main_loop_unref(loop);
  g_running = false;
  pump.join();

  gst_element_set_state(pipe_capture, GST_STATE_NULL);
  gst_element_set_state(pipe_encode, GST_STATE_NULL);
  gst_object_unref(pipe_capture);
  gst_object_unref(pipe_encode);
  return 0;
}
