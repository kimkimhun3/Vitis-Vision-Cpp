// histequalize_host.c — appsink->appsrc bridge tuned for full target bitrate at 60fps
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

typedef struct {
    GstElement   *appsrc;
    GstElement   *appsink;
    gboolean      video_info_valid;
    GstVideoInfo  video_info;
    GMainLoop    *loop;
} CustomData;

static volatile sig_atomic_t g_stop = 0;
static guint64 g_callback_frames = 0;
static guint64 g_encoder_frames = 0;
static GTimer *g_timer = NULL;
static GMutex g_stats_mutex;

static void handle_sigint(int signum) { (void)signum; g_stop = 1; }

static gboolean on_bus_message(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus;
    GMainLoop *loop = (GMainLoop *)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError *err=NULL; gchar *dbg=NULL;
        gst_message_parse_error(msg, &err, &dbg);
        g_printerr("[BUS] ERROR: %s\n", err->message);
        if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
        g_clear_error(&err); g_free(dbg);
        if (loop) g_main_loop_quit(loop);
        break;
    }
    case GST_MESSAGE_EOS:
        g_printerr("[BUS] EOS\n");
        if (loop) g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_WARNING: {
        GError *err=NULL; gchar *dbg=NULL;
        gst_message_parse_warning(msg, &err, &dbg);
        g_printerr("[BUS] WARN: %s\n", err->message);
        if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
        g_clear_error(&err); g_free(dbg);
        break;
    }
    default: break;
    }
    return TRUE;
}

// Enhanced encoder sink pad probe with better timing info
static GstPadProbeReturn enc_sink_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    static guint64 cnt = 0;
    static GstClockTime first_pts = GST_CLOCK_TIME_NONE;
    static GstClockTime last_report_time = 0;
    
    (void)pad; (void)user_data;
    
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (buf) {
            cnt++;
            g_mutex_lock(&g_stats_mutex);
            g_encoder_frames++;
            g_mutex_unlock(&g_stats_mutex);
            
            GstClockTime pts = GST_BUFFER_PTS(buf);
            if (first_pts == GST_CLOCK_TIME_NONE) {
                first_pts = pts;
                last_report_time = pts;
            }
            
            // Report every 2 seconds instead of every 120 frames
            if (pts - last_report_time >= 2 * GST_SECOND) {
                gdouble elapsed_sec = (gdouble)(pts - first_pts) / GST_SECOND;
                gdouble actual_fps = elapsed_sec > 0 ? (gdouble)cnt / elapsed_sec : 0;
                
                g_mutex_lock(&g_stats_mutex);
                guint64 cb_frames = g_callback_frames;
                guint64 enc_frames = g_encoder_frames;
                g_mutex_unlock(&g_stats_mutex);
                
                g_print("[STATS] Encoder: %" G_GUINT64_FORMAT " frames, %.1f fps actual | "
                       "Callback: %" G_GUINT64_FORMAT " frames | "
                       "Drop rate: %.1f%%\n",
                       cnt, actual_fps, cb_frames,
                       cb_frames > 0 ? ((gdouble)(cb_frames - enc_frames) / cb_frames) * 100 : 0);
                       
                last_report_time = pts;
            }
        }
    }
    return GST_PAD_PROBE_PASS;
}

// Enhanced callback with frame counting
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Negotiated: %dx%d %s @ %d/%d\n",
                    data->video_info.width, data->video_info.height,
                    gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&data->video_info)),
                    data->video_info.fps_n, data->video_info.fps_d);
        } else {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Count frames received by callback
    g_mutex_lock(&g_stats_mutex);
    g_callback_frames++;
    g_mutex_unlock(&g_stats_mutex);

    // No timestamp edits: appsrc(do-timestamp=true) will stamp running-time.
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), gst_buffer_ref(buffer));
    
    gst_sample_unref(sample);
    return ret;
}

// Periodic stats reporting function
static gboolean print_stats_timeout(gpointer user_data) {
    (void)user_data;
    static guint64 last_callback = 0, last_encoder = 0;
    static gdouble last_time = 0;
    
    if (!g_timer) return TRUE;
    
    gdouble current_time = g_timer_elapsed(g_timer, NULL);
    
    g_mutex_lock(&g_stats_mutex);
    guint64 current_callback = g_callback_frames;
    guint64 current_encoder = g_encoder_frames;
    g_mutex_unlock(&g_stats_mutex);
    
    if (last_time > 0) {
        gdouble time_diff = current_time - last_time;
        if (time_diff >= 1.0) {  // Report every second
            gdouble callback_fps = (current_callback - last_callback) / time_diff;
            gdouble encoder_fps = (current_encoder - last_encoder) / time_diff;
            
            g_print("[REALTIME] Callback: %.1f fps | Encoder: %.1f fps | "
                   "Efficiency: %.1f%%\n",
                   callback_fps, encoder_fps,
                   callback_fps > 0 ? (encoder_fps / callback_fps) * 100 : 0);
                   
            last_callback = current_callback;
            last_encoder = current_encoder;
            last_time = current_time;
        }
    } else {
        last_time = current_time;
        last_callback = current_callback;
        last_encoder = current_encoder;
    }
    
    return TRUE;  // Continue timeout
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    
    // Initialize mutex for thread-safe stats
    g_mutex_init(&g_stats_mutex);

    // ---- CLI ----
    gint bitrate_kbps = 10000;
    gint fps          = 60;
    gint width        = 1280;  // Try 720p first
    gint height       = 720;
    const gchar *device   = "/dev/video0";
    const gchar *dst_host = "192.168.25.69";
    gint dst_port         = 5004;
    enum { CODEC_H264, CODEC_H265 } codec = CODEC_H264;

    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate="))       bitrate_kbps = atoi(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--fps="))      fps          = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--width="))    width        = atoi(argv[i] + 8);
        else if (g_str_has_prefix(argv[i], "--height="))   height       = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--device="))   device       = argv[i] + 9;
        else if (g_str_has_prefix(argv[i], "--host="))     dst_host     = argv[i] + 7;
        else if (g_str_has_prefix(argv[i], "--port="))     dst_port     = atoi(argv[i] + 7);
        else if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *c = argv[i] + 8;
            if (!g_ascii_strcasecmp(c, "h265") || !g_ascii_strcasecmp(c, "hevc")) codec = CODEC_H265;
            else codec = CODEC_H264;
        }
    }

    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d\n",
            device, width, height, fps, bitrate_kbps,
            (codec==CODEC_H264?"H.264":"H.265"), dst_host, dst_port);

    // ---- Pipelines ----
    CustomData data = {0};

    // OPTIMIZED Capture/appsink: Force specific pixel format and add more debugging
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=60/1, pixel-aspect-ratio=1/1 ! "
        "identity silent=false ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=cv_sink emit-signals=true max-buffers=6 drop=false sync=false",
        device, width, height, fps);

    GError *error = NULL;
    GstElement *app_sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!app_sink_pipeline) {
        g_printerr("Failed to create appsink pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        return -1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(app_sink_pipeline), "cv_sink");
    if (!data.appsink) {
        g_printerr("Failed to get appsink element\n");
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    // OPTIMIZED appsrc/encoder/UDP pipeline with larger queue sizes
    gchar *src_pipeline_str = NULL;
    int idr = fps * 4; if (idr < 60) idr = 60;

    if (codec == CODEC_H264) {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=12 leaky=no ! "
            "omxh264enc name=enc num-slices=8 periodicity-idr=%d "
            "cpb-size=500 gdr-mode=horizontal initial-delay=250 "
            "control-rate=low-latency prefetch-buffer=true "
            "target-bitrate=%d gop-mode=low-delay-p ! "
            "video/x-h264, alignment=nal, stream-format=byte-stream ! "
            "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            "queue max-size-buffers=12 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps, idr, bitrate_kbps, dst_host, dst_port);
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=12 leaky=no ! "
            "omxh265enc name=enc control-rate=constant target-bitrate=%d "
            "gop-mode=low-delay-p gop-length=%d periodicity-idr=%d "
            "num-slices=8 qp-mode=auto prefetch-buffer=true "
            "cpb-size=500 initial-delay=250 gdr-mode=horizontal filler-data=true ! "
            "video/x-h265, alignment=au, profile=main, stream-format=byte-stream ! "
            "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
            "queue max-size-buffers=12 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps, bitrate_kbps, fps, idr, dst_host, dst_port);
    }

    error = NULL;
    GstElement *app_src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!app_src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "cv_src");
    if (!data.appsrc) {
        g_printerr("Failed to get appsrc element\n");
        gst_object_unref(app_src_pipeline);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    // OPTIMIZED appsrc configuration with increased buffer capacity
    g_object_set(data.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "block", TRUE,              // block briefly instead of dropping
                 "max-bytes", 8*1024*1024,   // Doubled buffer size
                 "max-buffers", 10,          // Allow more queued buffers
                 NULL);

    // Set caps on appsrc directly for stable negotiation
    GstCaps *appsrc_caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width",  G_TYPE_INT,    width,
        "height", G_TYPE_INT,    height,
        "framerate", GST_TYPE_FRACTION, fps, 1, NULL);
    gst_app_src_set_caps(GST_APP_SRC(data.appsrc), appsrc_caps);
    gst_caps_unref(appsrc_caps);

    // Bridge callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Share the same clock across both top-level pipelines
    GstClock *sysclk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(app_sink_pipeline), sysclk);
    gst_pipeline_use_clock(GST_PIPELINE(app_src_pipeline),  sysclk);
    gst_object_unref(sysclk);

    // Bus watches
    data.loop = g_main_loop_new(NULL, FALSE);
    
    // Initialize timer and stats
    g_timer = g_timer_new();
    
    // Add periodic stats reporting (every 100ms)
    g_timeout_add(100, print_stats_timeout, NULL);
    
    GstBus *bus_sink = gst_element_get_bus(app_sink_pipeline);
    GstBus *bus_src  = gst_element_get_bus(app_src_pipeline);
    gst_bus_add_watch(bus_sink, on_bus_message, data.loop);
    gst_bus_add_watch(bus_src,  on_bus_message, data.loop);
    gst_object_unref(bus_sink);
    gst_object_unref(bus_src);

    // Attach probe to encoder sink
    GstElement *enc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "enc");
    if (enc) {
        GstPad *sinkpad = gst_element_get_static_pad(enc, "sink");
        if (sinkpad) {
            gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe, NULL, NULL);
            gst_object_unref(sinkpad);
        }
        gst_object_unref(enc);
    }

    // Start pipelines
    gst_element_set_state(app_src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);

    signal(SIGINT, handle_sigint);
    g_print("Running with enhanced diagnostics. Press Ctrl+C to exit.\n");
    g_print("Watch for [REALTIME] and [STATS] output to monitor frame rates.\n");

    while (!g_stop) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000); // 1 ms
    }

    g_print("Stopping...\n");
    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(app_src_pipeline,  GST_STATE_NULL);
    
    if (g_timer) {
        g_timer_destroy(g_timer);
    }
    
    // Clean up mutex
    g_mutex_clear(&g_stats_mutex);
    
    g_main_loop_unref(data.loop);
    gst_object_unref(app_sink_pipeline);
    gst_object_unref(app_src_pipeline);
    return 0;
}