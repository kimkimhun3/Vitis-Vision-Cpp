// host_h265_1080p60.c — appsink->appsrc passthrough (MMAP, non-DMABuf) tuned for 1080p60 via omxh265enc
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    GstElement   *appsrc;
    GstElement   *appsink;
    gboolean      video_info_valid;
    GstVideoInfo  video_info;
    GMainLoop    *loop;
} CustomData;

static volatile sig_atomic_t g_stop = 0;
static GTimer *g_timer = NULL;
static GMutex g_stats_mutex;
static guint64 g_cb_frames=0, g_push_ok=0, g_appsrc_out=0, g_enc_in=0;

static void handle_sigint(int sig){ (void)sig; g_stop=1; }

static gboolean on_bus(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus;
    GMainLoop *loop = (GMainLoop*)user_data;
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

static GstPadProbeReturn appsrc_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u) {
    (void)pad; (void)u;
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        g_mutex_lock(&g_stats_mutex); g_appsrc_out++; g_mutex_unlock(&g_stats_mutex);
    }
    return GST_PAD_PROBE_PASS;
}

static GstPadProbeReturn enc_sink_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u) {
    (void)pad; (void)u;
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        g_mutex_lock(&g_stats_mutex); g_enc_in++; g_mutex_unlock(&g_stats_mutex);
    }
    return GST_PAD_PROBE_PASS;
}

// appsink -> appsrc passthrough (no copy; keep PTS/DTS)
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData*)user_data;
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    if (!inbuf) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            // lock appsrc caps to camera caps so we preserve format/fps exactly
            gst_app_src_set_caps(GST_APP_SRC(data->appsrc), caps);
            g_print("Negotiated: %dx%d %s @ %d/%d\n",
                    data->video_info.width, data->video_info.height,
                    gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&data->video_info)),
                    data->video_info.fps_n, data->video_info.fps_d);
        } else { gst_sample_unref(sample); return GST_FLOW_ERROR; }
    }

    // push a ref (no memcpy); timestamps preserved because do-timestamp=false on appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), gst_buffer_ref(inbuf));

    g_mutex_lock(&g_stats_mutex);
    g_cb_frames++;
    if (ret == GST_FLOW_OK) g_push_ok++;
    g_mutex_unlock(&g_stats_mutex);

    gst_sample_unref(sample);
    return ret;
}

static gboolean stats_tick(gpointer u) {
    (void)u;
    static gdouble last_t=0; static guint64 c0=0,c1=0,c2=0,c3=0;
    if (!g_timer) return TRUE;
    gdouble now = g_timer_elapsed(g_timer,NULL);
    if (last_t==0){ last_t=now; c0=g_cb_frames; c1=g_push_ok; c2=g_appsrc_out; c3=g_enc_in; return TRUE; }
    gdouble dt = now-last_t; if (dt<1.0) return TRUE;

    g_mutex_lock(&g_stats_mutex);
    guint64 n0=g_cb_frames, n1=g_push_ok, n2=g_appsrc_out, n3=g_enc_in;
    g_mutex_unlock(&g_stats_mutex);

    gdouble cb = (n0-c0)/dt, push=(n1-c1)/dt, out=(n2-c2)/dt, enc=(n3-c3)/dt;
    g_print("[REALTIME] cb %.1f | pushOK %.1f | appsrc-out %.1f | enc %.1f fps | eff(enc/cb)=%.0f%%\n",
            cb, push, out, enc, cb>0?(enc/cb)*100:0);

    last_t=now; c0=n0; c1=n1; c2=n2; c3=n3;
    return TRUE;
}

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    g_mutex_init(&g_stats_mutex);

    // ---- CLI ----
    gint bitrate_kbps = 10000;
    gint fps=60, width=1920, height=1080;
    const gchar *host="192.168.25.69"; gint port=5004;

    for (int i=1;i<argc;i++){
        if (g_str_has_prefix(argv[i],"--bitrate=")) bitrate_kbps=atoi(argv[i]+10);
        else if (g_str_has_prefix(argv[i],"--fps=")) fps=atoi(argv[i]+6);
        else if (g_str_has_prefix(argv[i],"--width=")) width=atoi(argv[i]+8);
        else if (g_str_has_prefix(argv[i],"--height=")) height=atoi(argv[i]+9);
        else if (g_str_has_prefix(argv[i],"--host=")) host=argv[i]+7;
        else if (g_str_has_prefix(argv[i],"--port=")) port=atoi(argv[i]+7);
    }

    if ((width&1)||(height&1)) { g_printerr("NV12 requires even WxH\n"); return -1; }

    g_print("Device=/dev/video0  %dx%d@%dfps  Bitrate=%d kbps  dst=%s:%d (H.265/omxh265enc)\n",
            width,height,fps,bitrate_kbps,host,port);

    CustomData data={0};
    GError *err=NULL;

    // ---- Pipeline 1: camera -> appsink (MMAP, non-DMABuf), drop under pressure
    gchar *p1 = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=mmap do-timestamp=true ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=my_sink emit-signals=true max-buffers=3 drop=true sync=false",
        width,height,fps,fps);

    GstElement *sink_pipe = gst_parse_launch(p1,&err); g_free(p1);
    if (!sink_pipe){ g_printerr("sink pipeline: %s\n", err?err->message:"(unknown)"); g_clear_error(&err); return -1; }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipe),"my_sink");
    if (!data.appsink){ g_printerr("get appsink failed\n"); gst_object_unref(sink_pipe); return -1; }

    // ---- Pipeline 2: appsrc -> omxh265enc -> RTP -> UDP
    const char *qll = "queue max-size-buffers=12 max-size-time=40000000 leaky=downstream";
    const int   idr = fps; // ~1s IDR

    gchar *p2 = g_strdup_printf(
        "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=false block=true max-bytes=%d ! "
        // keep caps explicit; we also set exact caps at runtime from the first sample
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
        "%s ! "
        "omxh265enc name=enc "
          "control-rate=constant target-bitrate=%d "
          "gop-mode=low-delay-p gop-length=%d periodicity-idr=%d "
          "num-slices=2 prefetch-buffer=false filler-data=false ! "
        "video/x-h265,alignment=au,profile=main,stream-format=byte-stream ! "
        "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
        "%s ! "
        "udpsink buffer-size=60000000 host=%s port=%d sync=false",
        8*1024*1024, width,height,fps,
        qll,
        bitrate_kbps, fps, idr,
        qll,
        host, port);

    GstElement *src_pipe = gst_parse_launch(p2,&err); g_free(p2);
    if (!src_pipe){ g_printerr("src pipeline: %s\n", err?err->message:"(unknown)"); g_clear_error(&err); gst_object_unref(sink_pipe); return -1; }
    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipe),"my_src");
    if (!data.appsrc){ g_printerr("get appsrc failed\n"); gst_object_unref(src_pipe); gst_object_unref(sink_pipe); return -1; }

    // share the same clock across both pipelines
    GstClock *clk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(sink_pipe), clk);
    gst_pipeline_use_clock(GST_PIPELINE(src_pipe),  clk);
    gst_object_unref(clk);

    // probes for stats
    GstElement *enc = gst_bin_get_by_name(GST_BIN(src_pipe),"enc");
    if (enc){
        GstPad *sinkpad = gst_element_get_static_pad(enc,"sink");
        if (sinkpad){ gst_pad_add_probe(sinkpad, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe, NULL, NULL); gst_object_unref(sinkpad); }
        gst_object_unref(enc);
    }
    GstPad *appsrc_srcpad = gst_element_get_static_pad(data.appsrc,"src");
    if (appsrc_srcpad){ gst_pad_add_probe(appsrc_srcpad, GST_PAD_PROBE_TYPE_BUFFER, appsrc_src_probe, NULL, NULL); gst_object_unref(appsrc_srcpad); }

    // callbacks
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // bus + loop
    data.loop = g_main_loop_new(NULL,FALSE);
    GstBus *b1=gst_element_get_bus(sink_pipe), *b2=gst_element_get_bus(src_pipe);
    gst_bus_add_watch(b1,on_bus,data.loop); gst_bus_add_watch(b2,on_bus,data.loop);
    gst_object_unref(b1); gst_object_unref(b2);

    // periodic stats
    g_timer = g_timer_new(); g_timeout_add(100, stats_tick, NULL);

    // run
    signal(SIGINT, handle_sigint);
    gst_element_set_state(src_pipe, GST_STATE_PLAYING);
    gst_element_set_state(sink_pipe, GST_STATE_PLAYING);
    g_print("Running. Ctrl+C to exit.\n");

    while (!g_stop) { g_main_context_iteration(NULL, FALSE); g_usleep(1000); }

    // teardown
    gst_element_set_state(sink_pipe, GST_STATE_NULL);
    gst_element_set_state(src_pipe,  GST_STATE_NULL);
    if (g_timer) g_timer_destroy(g_timer);
    g_mutex_clear(&g_stats_mutex);
    g_main_loop_unref(data.loop);
    gst_object_unref(sink_pipe);
    gst_object_unref(src_pipe);
    return 0;
}
