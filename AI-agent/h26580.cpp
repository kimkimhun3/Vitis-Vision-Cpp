// histequalize_host_dynamic.c
// appsink -> appsrc bridge with fixed device/host/port, but encoder pipeline parameters are dynamic.
// Focus: make properties/caps *after* the encoder configurable via CLI.

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

//
// -------- Fixed endpoints (per your requirement) --------
//
#define FIXED_DEVICE "/dev/video0"
#define FIXED_HOST   "192.168.25.69"
#define FIXED_PORT   5004

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

    g_mutex_lock(&g_stats_mutex);
    g_callback_frames++;
    g_mutex_unlock(&g_stats_mutex);

    // Zero-copy handoff: just ref and push.
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), gst_buffer_ref(buffer));
    gst_sample_unref(sample);
    return ret;
}

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
        if (time_diff >= 1.0) {
            gdouble callback_fps = (current_callback - last_callback) / time_diff;
            gdouble encoder_fps = (current_encoder - last_encoder) / time_diff;

            g_print("[REALTIME] Callback: %.1f fps | Encoder: %.1f fps | Efficiency: %.1f%%\n",
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
    return TRUE;
}

//
// --------------------- CLI helpers ---------------------
//
static gboolean parse_bool(const char *s, int *out) {
    if (!s) return FALSE;
    if (!g_ascii_strcasecmp(s, "1") || !g_ascii_strcasecmp(s, "true") || !g_ascii_strcasecmp(s, "yes")) { *out=1; return TRUE; }
    if (!g_ascii_strcasecmp(s, "0") || !g_ascii_strcasecmp(s, "false")|| !g_ascii_strcasecmp(s, "no"))  { *out=0; return TRUE; }
    return FALSE;
}

// Map strings to omx rc values: "constant|variable|low-latency|disable"
// We pass the string through as-is to the pipeline.
static const char* norm_rc(const char *s){
    if (!s) return NULL;
    if (!g_ascii_strcasecmp(s,"constant"))     return "constant";
    if (!g_ascii_strcasecmp(s,"variable"))     return "variable";
    if (!g_ascii_strcasecmp(s,"disable"))      return "disable";
    if (!g_ascii_strcasecmp(s,"low-latency"))  return "low-latency";
    return s;
}

// entropy-mode for H.264: default|cabac|cavlc
static const char* norm_entropy(const char *s){
    if (!s) return NULL;
    if (!g_ascii_strcasecmp(s,"cabac")) return "CABAC";
    if (!g_ascii_strcasecmp(s,"cavlc")) return "CAVLC";
    if (!g_ascii_strcasecmp(s,"default")) return "default";
    return s;
}

// gop-mode per gst-inspect list (we pass string through)
static const char* norm_gop_mode(const char *s){ return s; }

// gdr-mode: disabled|horizontal|vertical (pass through if present)
static const char* norm_gdr(const char *s){ return s; }

// qp-mode: encoder-dependent; pass through if provided
static const char* norm_qp_mode(const char *s){ return s; }

// alignment: au|nal
static const char* norm_align(const char *s){
    if (!s) return NULL;
    if (!g_ascii_strcasecmp(s,"au"))  return "au";
    if (!g_ascii_strcasecmp(s,"nal")) return "nal";
    return s;
}

// stream-format: byte-stream|avc
static const char* norm_streamfmt(const char *s){
    if (!s) return NULL;
    if (!g_ascii_strcasecmp(s,"byte-stream")) return "byte-stream";
    if (!g_ascii_strcasecmp(s,"avc"))         return "avc";
    return s;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    g_mutex_init(&g_stats_mutex);

    // ---- user-adjustable capture basics (width/height/fps), bitrate, codec ----
    gint bitrate_kbps = 10000;
    gint fps          = 60;
    gint width        = 1920;
    gint height       = 1080;
    enum { CODEC_H264, CODEC_H265 } codec = CODEC_H264;

    // ---- encoder dynamic params (all optional; only set if provided) ----
    const char *rc_mode        = NULL;    // "constant|variable|low-latency|disable"
    const char *entropy_mode   = NULL;    // CABAC|CAVLC|default
    const char *gop_mode       = "low-delay-p"; // default safe
    const char *gdr_mode       = NULL;    // "disabled|horizontal|vertical"
    const char *qp_mode        = NULL;    // e.g. "auto" (omxh265enc), vendor-specific
    gint        cpb_ms         = -1;      // ms
    gint        initial_delay  = -1;      // ms (only if present on your BSP)
    gint        gop_length     = -1;      // frames
    gint        idr_period     = -1;      // frames
    gint        num_slices     = -1;      // >=1
    gint        slice_size     = -1;      // bytes
    gint        dependent_slice= -1;      // bool
    gint        filler_data    = -1;      // bool
    gint        skip_frame     = -1;      // bool
    gint        low_bandwidth  = -1;      // bool

    // caps after encoder
    const char *profile_caps   = NULL;    // e.g., "high" (h264), "main" (h265)
    const char *level_caps     = NULL;    // e.g., "4.2" (h264), "5.1" (h265)
    const char *alignment_caps = "nal";   // default; user may set
    const char *stream_format  = "byte-stream"; // default

    // RTP/Payloader params
    gint rtp_pt = 96;
    gint mtu    = 1400;
    gint cfg_int= 1;

    // Stabilizer
    gint stabilize = 1; // 1 = insert videorate

    // ---- parse CLI ----
    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate="))        bitrate_kbps = atoi(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--fps="))       fps          = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--width="))     width        = atoi(argv[i] + 8);
        else if (g_str_has_prefix(argv[i], "--height="))    height       = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *c = argv[i] + 8;
            if (!g_ascii_strcasecmp(c, "h265") || !g_ascii_strcasecmp(c, "hevc")) codec = CODEC_H265;
            else codec = CODEC_H264;
        }
        else if (g_str_has_prefix(argv[i], "--rc="))        rc_mode      = norm_rc(argv[i] + 5);
        else if (g_str_has_prefix(argv[i], "--entropy="))   entropy_mode = norm_entropy(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--gop-mode="))  gop_mode     = norm_gop_mode(argv[i] + 11);
        else if (g_str_has_prefix(argv[i], "--gop-length="))gop_length   = atoi(argv[i] + 13);
        else if (g_str_has_prefix(argv[i], "--idr="))       idr_period   = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--slices="))    num_slices   = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--slice-size="))slice_size   = atoi(argv[i] + 13);
        else if (g_str_has_prefix(argv[i], "--dependent-slice=")) dependent_slice = atoi(argv[i] + 18);
        else if (g_str_has_prefix(argv[i], "--gdr="))       gdr_mode     = norm_gdr(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--qp-mode="))   qp_mode      = norm_qp_mode(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--cpb-ms="))    cpb_ms       = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--initial-delay-ms=")) initial_delay = atoi(argv[i] + 20);
        else if (g_str_has_prefix(argv[i], "--filler-data=")) { int b; if (parse_bool(argv[i]+14,&b)) filler_data=b; }
        else if (g_str_has_prefix(argv[i], "--skip-frame="))   { int b; if (parse_bool(argv[i]+13,&b)) skip_frame=b; }
        else if (g_str_has_prefix(argv[i], "--low-bandwidth=")){ int b; if (parse_bool(argv[i]+16,&b)) low_bandwidth=b; }
        else if (g_str_has_prefix(argv[i], "--profile="))   profile_caps = argv[i] + 10;
        else if (g_str_has_prefix(argv[i], "--level="))     level_caps   = argv[i] + 8;
        else if (g_str_has_prefix(argv[i], "--align="))     alignment_caps = norm_align(argv[i] + 8);
        else if (g_str_has_prefix(argv[i], "--stream-format=")) stream_format = norm_streamfmt(argv[i] + 16);
        else if (g_str_has_prefix(argv[i], "--pt="))        rtp_pt       = atoi(argv[i] + 5);
        else if (g_str_has_prefix(argv[i], "--mtu="))       mtu          = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--config-interval=")) cfg_int = atoi(argv[i] + 18);
        else if (g_str_has_prefix(argv[i], "--stabilize=")) stabilize    = atoi(argv[i] + 12);
        // (intentionally ignoring device/host/port per your request—they are fixed)
    }

    if ((width & 1) || (height & 1)) {
        g_printerr("Error: NV12 requires even width/height. Got %dx%d\n", width, height);
        return -1;
    }

    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d  stabilizer=%s\n",
            FIXED_DEVICE, width, height, fps, bitrate_kbps,
            (codec==CODEC_H264?"H.264":"H.265"), FIXED_HOST, FIXED_PORT,
            stabilize?"on":"off");

    // ---- Pipelines ----
    CustomData data = {0};

    // Capture/appsink (device fixed)
    gchar *sink_pipeline_str = NULL;
    if (stabilize) {
        sink_pipeline_str = g_strdup_printf(
            "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1, pixel-aspect-ratio=1/1 ! "
            "videorate drop-only=true max-rate=%d ! "
            "appsink name=cv_sink emit-signals=true max-buffers=6 drop=false sync=false",
            FIXED_DEVICE, width, height, fps, fps);
    } else {
        sink_pipeline_str = g_strdup_printf(
            "v4l2src device=%s io-mode=dmabuf do-timestamp=true ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1, pixel-aspect-ratio=1/1 ! "
            "appsink name=cv_sink emit-signals=true max-buffers=6 drop=false sync=false",
            FIXED_DEVICE, width, height, fps);
    }

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

    // Build encoder property string dynamically
    GString *enc_props = g_string_new("");

    // Common properties
    if (rc_mode)         g_string_append_printf(enc_props, " control-rate=%s", rc_mode);
    g_string_append_printf(enc_props, " target-bitrate=%d", bitrate_kbps);
    if (entropy_mode)    g_string_append_printf(enc_props, " entropy-mode=%s", entropy_mode);
    if (gop_mode)        g_string_append_printf(enc_props, " gop-mode=%s", gop_mode);
    if (gop_length>0)    g_string_append_printf(enc_props, " gop-length=%d", gop_length);
    if (idr_period>0)    g_string_append_printf(enc_props, " periodicity-idr=%d", idr_period);
    if (num_slices>0)    g_string_append_printf(enc_props, " num-slices=%d", num_slices);
    if (slice_size>0)    g_string_append_printf(enc_props, " slice-size=%d", slice_size);
    if (dependent_slice>=0) g_string_append_printf(enc_props, " dependent-slice=%s", dependent_slice?"true":"false");
    if (gdr_mode)        g_string_append_printf(enc_props, " gdr-mode=%s", gdr_mode);
    if (qp_mode)         g_string_append_printf(enc_props, " qp-mode=%s", qp_mode);
    if (cpb_ms>=0)       g_string_append_printf(enc_props, " cpb-size=%d", cpb_ms);
    if (initial_delay>=0)g_string_append_printf(enc_props, " initial-delay=%d", initial_delay);
    if (filler_data>=0)  g_string_append_printf(enc_props, " filler-data=%s", filler_data?"true":"false");
    if (skip_frame>=0)   g_string_append_printf(enc_props, " skip-frame=%s", skip_frame?"true":"false");
    if (low_bandwidth>=0)g_string_append_printf(enc_props, " low-bandwidth=%s", low_bandwidth?"true":"false");

    // After-encoder caps (profile/level/alignment/stream-format)
    GString *post_caps = g_string_new("");
    if (codec == CODEC_H264) {
        g_string_append(post_caps, "video/x-h264,");
        if (profile_caps) g_string_append_printf(post_caps, "profile=%s,", profile_caps);
        if (level_caps)   g_string_append_printf(post_caps, "level=(string)%s,", level_caps);
    } else {
        g_string_append(post_caps, "video/x-h265,");
        if (profile_caps) g_string_append_printf(post_caps, "profile=%s,", profile_caps);
        if (level_caps)   g_string_append_printf(post_caps, "level=(string)%s,", level_caps);
    }
    g_string_append_printf(post_caps, "alignment=%s,stream-format=%s", alignment_caps, stream_format);

    // Encoder pipeline (host/port fixed)
    gchar *src_pipeline_str = NULL;
    if (codec == CODEC_H264) {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=12 leaky=no ! "
            "omxh264enc name=enc%s ! "
            "%s ! "
            "rtph264pay pt=%d mtu=%d config-interval=%d ! "
            "queue max-size-buffers=12 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps,
            enc_props->len ? enc_props->str : "",
            post_caps->str,
            rtp_pt, mtu, cfg_int,
            FIXED_HOST, FIXED_PORT);
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=cv_src format=GST_FORMAT_TIME ! "
            "video/x-raw, format=NV12, width=%d, height=%d, framerate=%d/1 ! "
            "queue max-size-buffers=12 leaky=no ! "
            "omxh265enc name=enc%s ! "
            "%s ! "
            "rtph265pay pt=%d mtu=%d config-interval=%d ! "
            "queue max-size-buffers=12 leaky=no ! "
            "udpsink host=%s port=%d sync=true",
            width, height, fps,
            enc_props->len ? enc_props->str : "",
            post_caps->str,
            rtp_pt, mtu, cfg_int,
            FIXED_HOST, FIXED_PORT);
    }

    g_string_free(enc_props, TRUE);
    g_string_free(post_caps, TRUE);

    error = NULL;
    GstElement *app_src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    if (!app_src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        g_free(src_pipeline_str);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }
    g_free(src_pipeline_str);

    data.appsrc = gst_bin_get_by_name(GST_BIN(app_src_pipeline), "cv_src");
    if (!data.appsrc) {
        g_printerr("Failed to get appsrc element\n");
        gst_object_unref(app_src_pipeline);
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    // appsrc configuration
    g_object_set(data.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,   // leave PTS to upstream; simple and low overhead
                 "block", TRUE,
                 "max-bytes", 8*1024*1024,
                 NULL);

    // Explicit caps on appsrc
    GstCaps *appsrc_caps = gst_caps_new_simple("video/x-raw",
        "format",  G_TYPE_STRING, "NV12",
        "width",   G_TYPE_INT,    width,
        "height",  G_TYPE_INT,    height,
        "framerate", GST_TYPE_FRACTION, fps, 1, NULL);
    gst_app_src_set_caps(GST_APP_SRC(data.appsrc), appsrc_caps);
    gst_caps_unref(appsrc_caps);

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
