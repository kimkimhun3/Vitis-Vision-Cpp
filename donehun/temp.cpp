  gchar *pipeline_str =
      g_strdup_printf("v4l2src device=/dev/video0 io-mode=4  ! "
                      "video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1 ! "
                      "videorate drop-only=true max-rate=%d/1 ! appsink "
                      "name=cv_sink emit-signals=true max-buffers=1 drop=true",
                      fps);
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
      "width=1920, height=1080, framerate=%s/1 ! "
      "queue ! omxh264enc target-bitrate=%u num-slices=1 "
      "control-rate=Constant qp-mode=fixed prefetch-buffer=true "
      "cpb-size=200 initial-delay=200 "
      "gdr-mode=disabled periodicity-idr=30 gop-length=30 filler-data=true ! "
      "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
      "rtph264pay mtu=1400 ! "
      "queue max-size-buffers=2  ! "
      "udpsink clients=192.168.25.69:5004 auto-multicast=false",
      fps, target_bitrate);
  app_src_pipeline = gst_parse_launch(pipeline_str, &error);
  g_free(pipeline_str);
  if (!app_src_pipeline) {
    g_printerr("Failed to create appsrc: %s\n", error->message);
    g_clear_error(&error);
    return -1;
  }