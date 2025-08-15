// histequalize_host.c — appsink->appsrc bridge tuned for 1080p60, DMABuf AUTO/ON/OFF + memory logs
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
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

typedef enum { DMABUF_AUTO, DMABUF_ON, DMABUF_OFF } DmaBufMode;

static volatile sig_atomic_t g_stop = 0;
static guint64 g_callback_frames = 0;   // pulled from appsink
static guint64 g_pushed_ok = 0;         // successful appsrc pushes
static guint64 g_appsrc_out = 0;        // buffers leaving appsrc (src pad)
static guint64 g_encoder_frames = 0;    // buffers arriving at encoder sink
static GTimer *g_timer = NULL;
static GMutex g_stats_mutex;

// one-shot memory logs
static gboolean g_logged_in_mem  = FALSE;
static gboolean g_logged_out_mem = FALSE;

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

static gboolean buffer_has_dmabuf(GstBuffer *buf) {
    if (!buf) return FALSE;
    guint n = gst_buffer_n_memory(buf);
    for (guint i=0; i<n; ++i) {
        GstMemory *m = gst_buffer_peek_memory(buf, i);
        if (m && gst_is_dmabuf_memory(m)) return TRUE;
    }
    return FALSE;
}

static GstPadProbeReturn enc_sink_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    static guint64 cnt = 0;
    static GstClockTime first_pts = GST_CLOCK_TIME_NONE;
    static GstClockTime last_report_time = 0;
    static gboolean logged_enc_mem = FALSE; // log once
    (void)pad; (void)user_data;

    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (buf) {
            if (!logged_enc_mem) {
                g_print("[MEM] enc-sink: DMABuf=%s\n", buffer_has_dmabuf(buf) ? "YES" : "NO");
                logged_enc_mem = TRUE;
            }

            cnt++;
            g_mutex_lock(&g_stats_mutex);
            g_encoder_frames++;
            g_mutex_unlock(&g_stats_mutex);

            GstClockTime pts = GST_BUFFER_PTS(buf);
            if (first_pts == GST_CLOCK_TIME_NONE) { first_pts = pts; last_report_time = pts; }
            if (pts != GST_CLOCK_TIME_NONE && pts - last_report_time >= 2 * GST_SECOND) {
                gdouble elapsed_sec = (gdouble)(pts - first_pts) / GST_SECOND;
                gdouble actual_fps = elapsed_sec > 0 ? (gdouble)cnt / elapsed_sec : 0;

                g_mutex_lock(&g_stats_mutex);
                guint64 cb_frames = g_callback_frames;
                guint64 enc_frames = g_encoder_frames;
                g_mutex_unlock(&g_stats_mutex);

                g_print("[STATS] Encoder: %" G_GUINT64_FORMAT " frames, %.1f fps actual | "
                        "Callback: %" G_GUINT64_FORMAT " frames | Drop rate: %.1f%%\n",
                        cnt, actual_fps, cb_frames,
                        cb_frames > 0 ? ((gdouble)(cb_frames - enc_frames) / cb_frames) * 100 : 0);

                last_report_time = pts;
            }
        }
    }
    return GST_PAD_PROBE_PASS;
}

static GstPadProbeReturn appsrc_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    (void)pad; (void)user_data;
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!g_logged_out_mem) {
            g_print("[MEM] appsrc-out: DMABuf=%s\n", buffer_has_dmabuf(buf) ? "YES" : "NO");
            g_logged_out_mem = TRUE;
        }
        g_mutex_lock(&g_stats_mutex);
        g_appsrc_out++;
        g_mutex_unlock(&g_stats_mutex);
    }
    return GST_PAD_PROBE_PASS;
}

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

    if (!g_logged_in_mem) {
        g_print("[MEM] appsink: DMABuf=%s\n", buffer_has_dmabuf(buffer) ? "YES" : "NO");
        g_logged_in_mem = TRUE;
    }

    g_mutex_lock(&g_stats_mutex);
    g_callback_frames++;
    g_mutex_unlock(&g_stats_mutex);

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), gst_buffer_ref(buffer));
    if (ret == GST_FLOW_OK) {
        g_mutex_lock(&g_stats_mutex);
        g_pushed_ok++;
        g_mutex_unlock(&g_stats_mutex);
    }
    gst_sample_unref(sample);
    return ret;
}

static gboolean print_stats_timeout(gpointer user_data) {
    (void)user_data;
    static guint64 last_callback = 0, last_encoder = 0, last_pushok = 0, last_appsrc_out = 0;
    static gdouble last_time = 0;

    if (!g_timer) return TRUE;

    gdouble current_time = g_timer_elapsed(g_timer, NULL);

    g_mutex_lock(&g_stats_mutex);
    guint64 current_callback = g_callback_frames;
    guint64 current_encoder  = g_encoder_frames;
    guint64 current_pushok   = g_pushed_ok;
    guint64 current_appsrc_o = g_appsrc_out;
    g_mutex_unlock(&g_stats_mutex);

    if (last_time > 0) {
        gdouble time_diff = current_time - last_time;
        if (time_diff >= 1.0) {
            gdouble callback_fps = (current_callback - last_callback) / time_diff;
            gdouble pushok_fps   = (current_pushok   - last_pushok)   / time_diff;
            gdouble appsrcout_fps= (current_appsrc_o - last_appsrc_out)/ time_diff;
            gdouble encoder_fps  = (current_encoder  - last_encoder)  / time_diff;

            g_print("[REALTIME] cb %.1f | pushOK %.1f | appsrc-out %.1f | enc %.1f fps | eff(enc/cb)=%.0f%%\n",
                    callback_fps, pushok_fps, appsrcout_fps, encoder_fps,
                    callback_fps > 0 ? (encoder_fps / callback_fps) * 100 : 0);

            last_callback  = current_callback;
            last_encoder   = current_encoder;
            last_pushok    = current_pushok;
            last_appsrc_out= current_appsrc_o;
            last_time = current_time;
        }
    } else {
        last_time = current_time;
        last_callback  = current_callback;
        last_encoder   = current_encoder;
        last_pushok    = current_pushok;
        last_appsrc_out= current_appsrc_o;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    g_mutex_init(&g_stats_mutex);

    // ---- CLI ----
    gint bitrate_kbps = 10000;
    gint fps          = 60;          // focus on 1080p60
    gint width        = 1920;
    gint height       = 1080;
    const gchar *device   = "/dev/video0";
    const gchar *dst_host = "192.168.25.69";
    gint dst_port         = 5004;
    enum { CODEC_H264, CODEC_H265 } codec = CODEC_H264;
    DmaBufMode dmabuf_mode = DMABUF_AUTO;

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
        } else if (g_str_has_prefix(argv[i], "--dmabuf=")) {
            const char *v = argv[i] + 9;
            if (!g_ascii_strcasecmp(v, "1") || !g_ascii_strcasecmp(v, "on")) dmabuf_mode = DMABUF_ON;
            else if (!g_ascii_strcasecmp(v, "0") || !g_ascii_strcasecmp(v, "off")) dmabuf_mode = DMABUF_OFF;
            else dmabuf_mode = DMABUF_AUTO;
        }
    }

    if ((width & 1) || (height & 1)) {
        g_printerr("Error: NV12 requires even width/height. Got %dx%d\n", width, height);
        return -1;
    }

    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d  DMABuf=%s\n",
            device, width, height, fps, bitrate_kbps,
            (codec==CODEC_H264?"H.264":"H.265"), dst_host, dst_port,
            (dmabuf_mode==DMABUF_ON?"ON": dmabuf_mode==DMABUF_OFF?"OFF":"AUTO"));

    // ---- Pipelines ----
    CustomData data = {0};

    // Capture/appsink: live mode, keep latency low and avoid backpressure at 60 fps
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
        "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1, pixel-aspect-ratio=1/1 ! "
        "identity silent=false ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=cv_sink emit-signals=true max-buffers=3 drop=true sync=false",
        device, width, height, fps, fps);

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

    // Encoder pipeline — leaky queues for low latency; udpsink sync=false
    gchar *src_pipeline_str = NULL;
    int idr = fps;                    // 1s IDR for quicker recovery
    const char *queue_ll = "queue max-size-buffers=12 max-size-time=40000000 leaky=downstream";

    // Build capsfilter string right after appsrc (DMABuf AUTO/ON/OFF)
    gchar *caps_after_appsrc = NULL;
    if (dmabuf_mode == DMABUF_ON) {
        caps_after_appsrc = g_strdup_printf(
            "video/x-raw(memory:DMABuf),format=NV12,width=%d,height=%d,framerate=%d/1",
            width, height, fps);
    } else if (dmabuf_mode == DMABUF_OFF) {
        caps_after_appsrc = g_strdup_printf(
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1",
            width, height, fps);
    } else { // AUTO: offer both, downstream will pick
        caps_after_appsrc = g_strdup_printf(
            "video/x-raw(memory:DMABuf),format=NV12,width=%d,height=%d,framerate=%d/1; "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1",
            width, height, fps, width, height, fps);
    }

    if (codec == CODEC_H264) {
        // src_pipeline_str = g_strdup_printf(
        //     "appsrc name=cv_src format=GST_FORMAT_TIME is-live=true do-timestamp=false ! "
        //     "%s ! "                  // capsfilter immediately after appsrc
        //     "%s ! "
        //     "omxh264enc name=enc "
        //     "control-rate=constant target-bitrate=%d "
        //     "gop-mode=basic gop-length=%d periodicity-idr=%d "
        //     "num-slices=2 prefetch-buffer=false ! "
        //     "video/x-h264, alignment=au, stream-format=byte-stream ! "
        //     "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
        //     "%s ! "
        //     "udpsink host=%s port=%d sync=false",
        //     caps_after_appsrc, queue_ll, bitrate_kbps, fps, idr, queue_ll, dst_host, dst_port);
        src_pipeline_str = g_strdup_printf(
        "appsrc name=cv_src format=GST_FORMAT_TIME is-live=true do-timestamp=false ! "
        "%s ! "                  /* capsfilter immediately after appsrc (DMABuf AUTO/ON/OFF) */
        "%s ! "
        "omxh264enc name=enc "
        "control-rate=variable "          /* was: constant */
        "target-bitrate=%d "
        "gop-mode=low-delay-p gop-length=%d periodicity-idr=%d "
        "num-slices=1 prefetch-buffer=false ! "
        /* Request a high-profile bitstream; many encoders switch entropy/profile from this */
        "video/x-h264,profile=high,alignment=au,stream-format=byte-stream ! "
        "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
        "%s ! "
        "udpsink host=%s port=%d sync=false",
        caps_after_appsrc, queue_ll, bitrate_kbps, fps, fps, queue_ll, dst_host, dst_port);
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME is-live=true do-timestamp=false ! "
            "%s ! "
            "%s ! "
            "omxh265enc name=enc control-rate=constant target-bitrate=%d "
            "gop-mode=low-delay-p gop-length=%d periodicity-idr=%d "
            "num-slices=2 qp-mode=auto prefetch-buffer=false "
            "cpb-size=500 initial-delay=250 filler-data=false ! "
            "video/x-h265, alignment=au, profile=main, stream-format=byte-stream ! "
            "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
            "%s ! "
            "udpsink host=%s port=%d sync=false",
            caps_after_appsrc, queue_ll, bitrate_kbps, fps, idr, queue_ll, dst_host, dst_port);
    }
    g_free(caps_after_appsrc);

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

    // appsrc runtime config (preserve source PTS)
    g_object_set(data.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", FALSE,
                 "block", TRUE,
                 "max-bytes", 8*1024*1024,
                 NULL);

    // Bridge callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Share the same clock
    GstClock *sysclk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(app_sink_pipeline), sysclk);
    gst_pipeline_use_clock(GST_PIPELINE(app_src_pipeline),  sysclk);
    gst_object_unref(sysclk);

    // Bus watches & loop
    data.loop = g_main_loop_new(NULL, FALSE);

    g_timer = g_timer_new();
    g_timeout_add(100, print_stats_timeout, NULL);

    GstBus *bus_sink = gst_element_get_bus(app_sink_pipeline);
    GstBus *bus_src  = gst_element_get_bus(app_src_pipeline);
    gst_bus_add_watch(bus_sink, on_bus_message, data.loop);
    gst_bus_add_watch(bus_src,  on_bus_message, data.loop);
    gst_object_unref(bus_sink);
    gst_object_unref(bus_src);

    // Encoder sink probe
    GstElement *enc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "enc");
    if (enc) {
        GstPad *sinkpad = gst_element_get_static_pad(enc, "sink");
        if (sinkpad) {
            gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe, NULL, NULL);
            gst_object_unref(sinkpad);
        }
        gst_object_unref(enc);
    }

    // appsrc src-pad probe
    GstPad *appsrc_srcpad = gst_element_get_static_pad(data.appsrc, "src");
    if (appsrc_srcpad) {
        gst_pad_add_probe(appsrc_srcpad, GST_PAD_PROBE_TYPE_BUFFER, appsrc_src_probe, NULL, NULL);
        gst_object_unref(appsrc_srcpad);
    }

    // Start
    gst_element_set_state(app_src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);

    signal(SIGINT, handle_sigint);
    g_print("Running. Ctrl+C to exit.\n");

    while (!g_stop) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000);
    }

    g_print("Stopping...\n");
    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(app_src_pipeline,  GST_STATE_NULL);

    if (g_timer) g_timer_destroy(g_timer);
    g_mutex_clear(&g_stats_mutex);

    g_main_loop_unref(data.loop);
    gst_object_unref(app_sink_pipeline);
    gst_object_unref(app_src_pipeline);
    return 0;
}
