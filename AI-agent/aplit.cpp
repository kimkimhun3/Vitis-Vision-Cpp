// h265_bridge_split_dual.c
// appsink -> (relay) -> two appsrc's (even/odd), each to its own omxh265enc @30fps
// DMABuf zero-copy relay; appsrc retimes; per-encoder stats.

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
#include <glib.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    GstElement *appsink;
    GstElement *appsrc_even;
    GstElement *appsrc_odd;
    gboolean    video_info_valid;
    GstVideoInfo vi;
    GMainLoop  *loop;
    gint width, height, fps;
    guint64 frame_idx;
} Ctx;

static volatile sig_atomic_t g_stop=0;
static GTimer *g_timer=NULL;
static GMutex g_mu;
static guint64 g_cb=0, g_push_even=0, g_push_odd=0, g_out_even=0, g_out_odd=0, g_enc_even=0, g_enc_odd=0;
static gboolean g_log_in=FALSE, g_log_out_even=FALSE, g_log_out_odd=FALSE, g_log_enc_even=FALSE, g_log_enc_odd=FALSE;

static void on_sigint(int s){ (void)s; g_stop=1; }

static gboolean is_dmabuf(GstBuffer *b){
    if (!b) return FALSE;
    for (guint i=0, n=gst_buffer_n_memory(b); i<n; ++i){
        GstMemory *m = gst_buffer_peek_memory(b,i);
        if (m && gst_is_dmabuf_memory(m)) return TRUE;
    }
    return FALSE;
}

static GstBuffer* zero_copy_retime(GstBuffer *in){
    GstBuffer *out = gst_buffer_new();
    gst_buffer_copy_into(out, in, GST_BUFFER_COPY_METADATA, 0, -1);
    for (guint i=0, n=gst_buffer_n_memory(in); i<n; ++i)
        gst_buffer_append_memory(out, gst_memory_ref(gst_buffer_peek_memory(in,i)));
    GST_BUFFER_PTS(out)=GST_CLOCK_TIME_NONE;
    GST_BUFFER_DTS(out)=GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(out)=GST_CLOCK_TIME_NONE;
    return out;
}

static gboolean on_bus(GstBus *bus, GstMessage *m, gpointer user){
    (void)bus; GMainLoop *loop=(GMainLoop*)user;
    switch (GST_MESSAGE_TYPE(m)){
    case GST_MESSAGE_ERROR: {
        GError *e=NULL; gchar *dbg=NULL;
        gst_message_parse_error(m,&e,&dbg);
        g_printerr("[BUS] ERROR: %s\n", e->message);
        if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
        g_clear_error(&e); g_free(dbg);
        if (loop) g_main_loop_quit(loop);
        break; }
    case GST_MESSAGE_EOS:
        g_printerr("[BUS] EOS\n"); if (loop) g_main_loop_quit(loop); break;
    default: break;
    }
    return TRUE;
}

/* probes for stats */
static GstPadProbeReturn appsrc_out_probe_even(GstPad *p, GstPadProbeInfo *i, gpointer u){
    (void)p; (void)u; if (GST_PAD_PROBE_INFO_TYPE(i)&GST_PAD_PROBE_TYPE_BUFFER){
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(i);
        if (!g_log_out_even){ g_print("[MEM] appsrc-even out: DMABuf=%s\n", is_dmabuf(b)?"YES":"NO"); g_log_out_even=TRUE; }
        g_mutex_lock(&g_mu); g_out_even++; g_mutex_unlock(&g_mu);
    } return GST_PAD_PROBE_PASS;
}
static GstPadProbeReturn appsrc_out_probe_odd(GstPad *p, GstPadProbeInfo *i, gpointer u){
    (void)p; (void)u; if (GST_PAD_PROBE_INFO_TYPE(i)&GST_PAD_PROBE_TYPE_BUFFER){
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(i);
        if (!g_log_out_odd){ g_print("[MEM] appsrc-odd out: DMABuf=%s\n", is_dmabuf(b)?"YES":"NO"); g_log_out_odd=TRUE; }
        g_mutex_lock(&g_mu); g_out_odd++; g_mutex_unlock(&g_mu);
    } return GST_PAD_PROBE_PASS;
}
static GstPadProbeReturn enc_sink_probe_even(GstPad *p, GstPadProbeInfo *i, gpointer u){
    (void)p; (void)u; if (GST_PAD_PROBE_INFO_TYPE(i)&GST_PAD_PROBE_TYPE_BUFFER){
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(i);
        if (!g_log_enc_even){ g_print("[MEM] enc-even sink: DMABuf=%s\n", is_dmabuf(b)?"YES":"NO"); g_log_enc_even=TRUE; }
        g_mutex_lock(&g_mu); g_enc_even++; g_mutex_unlock(&g_mu);
    } return GST_PAD_PROBE_PASS;
}
static GstPadProbeReturn enc_sink_probe_odd(GstPad *p, GstPadProbeInfo *i, gpointer u){
    (void)p; (void)u; if (GST_PAD_PROBE_INFO_TYPE(i)&GST_PAD_PROBE_TYPE_BUFFER){
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(i);
        if (!g_log_enc_odd){ g_print("[MEM] enc-odd sink: DMABuf=%s\n", is_dmabuf(b)?"YES":"NO"); g_log_enc_odd=TRUE; }
        g_mutex_lock(&g_mu); g_enc_odd++; g_mutex_unlock(&g_mu);
    } return GST_PAD_PROBE_PASS;
}

/* appsink -> relay -> appsrc_even/odd */
static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user){
    Ctx *c=(Ctx*)user;
    GstSample *s = gst_app_sink_pull_sample(sink);
    if (!s) return GST_FLOW_ERROR;
    GstBuffer *in = gst_sample_get_buffer(s);
    if (!in){ gst_sample_unref(s); return GST_FLOW_ERROR; }

    if (!c->video_info_valid){
        GstCaps *caps = gst_sample_get_caps(s);
        if (caps && gst_video_info_from_caps(&c->vi, caps)){
            c->video_info_valid = TRUE;
            g_print("Negotiated: %dx%d %s @ %d/%d\n",
                    c->vi.width, c->vi.height,
                    gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&c->vi)),
                    c->vi.fps_n, c->vi.fps_d);
            /* lock appsrc caps to 30/1 for each branch */
            GstCaps *caps30 = gst_caps_new_simple("video/x-raw",
                "format", G_TYPE_STRING, "NV12",
                "width",  G_TYPE_INT, c->width,
                "height", G_TYPE_INT, c->height,
                "framerate", GST_TYPE_FRACTION, 30, 1, NULL);
            gst_app_src_set_caps(GST_APP_SRC(c->appsrc_even), caps30);
            gst_app_src_set_caps(GST_APP_SRC(c->appsrc_odd),  caps30);
            gst_caps_unref(caps30);
        } else { gst_sample_unref(s); return GST_FLOW_ERROR; }
    }

    if (!g_log_in){ g_print("[MEM] appsink: DMABuf=%s\n", is_dmabuf(in)?"YES":"NO"); g_log_in=TRUE; }

    gboolean to_even = ((c->frame_idx++ & 1u)==0);
    GstBuffer *out = zero_copy_retime(in);      // no memcpy; timestamps cleared

    GstElement *which = to_even ? c->appsrc_even : c->appsrc_odd;
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(which), out);

    g_mutex_lock(&g_mu);
    g_cb++;
    if (ret==GST_FLOW_OK) {
        if (to_even) g_push_even++; else g_push_odd++;
    }
    g_mutex_unlock(&g_mu);

    gst_sample_unref(s);
    return ret;
}

static gboolean tick(gpointer u){
    (void)u;
    static gdouble last=0;
    static guint64 c0=0, c1=0, c2=0, c3=0, c4=0, c5=0, c6=0;
    if (!g_timer) return TRUE;
    gdouble now=g_timer_elapsed(g_timer,NULL);
    if (last==0){ last=now; c0=g_cb; c1=g_push_even; c2=g_push_odd; c3=g_out_even; c4=g_out_odd; c5=g_enc_even; c6=g_enc_odd; return TRUE; }
    gdouble dt=now-last; if (dt<1.0) return TRUE;

    g_mutex_lock(&g_mu);
    guint64 n0=g_cb, n1=g_push_even, n2=g_push_odd, n3=g_out_even, n4=g_out_odd, n5=g_enc_even, n6=g_enc_odd;
    g_mutex_unlock(&g_mu);

    g_print("[REALTIME] cb %.1f | push even %.1f / odd %.1f | out even %.1f / odd %.1f | enc even %.1f / odd %.1f fps\n",
        (n0-c0)/dt, (n1-c1)/dt, (n2-c2)/dt, (n3-c3)/dt, (n4-c4)/dt, (n5-c5)/dt, (n6-c6)/dt);

    last=now; c0=n0; c1=n1; c2=n2; c3=n3; c4=n4; c5=n5; c6=n6;
    return TRUE;
}

int main(int argc, char **argv){
    gst_init(&argc,&argv);
    g_mutex_init(&g_mu);

    gint bitrate_kbps=10000, fps=60, width=1920, height=1080;
    const gchar *host="192.168.25.69"; gint port=5004; // even on port, odd on port+2
    gint slices=8;
    for (int i=1;i<argc;i++){
        if (g_str_has_prefix(argv[i],"--bitrate=")) bitrate_kbps=atoi(argv[i]+10);
        else if (g_str_has_prefix(argv[i],"--fps=")) fps=atoi(argv[i]+6);
        else if (g_str_has_prefix(argv[i],"--width=")) width=atoi(argv[i]+8);
        else if (g_str_has_prefix(argv[i],"--height=")) height=atoi(argv[i]+9);
        else if (g_str_has_prefix(argv[i],"--host=")) host=argv[i]+7;
        else if (g_str_has_prefix(argv[i],"--port=")) port=atoi(argv[i]+7);
        else if (g_str_has_prefix(argv[i],"--slices=")) slices=atoi(argv[i]+9);
    }
    if ((width&1)||(height&1)){ g_printerr("NV12 requires even WxH\n"); return -1; }

    // We split bitrate between two encoders
    gint br_each = bitrate_kbps/2;
    g_print("Split mode: %dx%d@%dfps -> two HEVC encoders @30fps each, ports %d (even) and %d (odd), %d+%d kbps\n",
            width,height,fps, port, port+2, br_each, br_each);

    Ctx c={0}; c.width=width; c.height=height; c.fps=fps;

    // Pipeline 1: camera -> appsink
    GError *err=NULL;
    gchar *p1 = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=dmabuf do-timestamp=true ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=cap_sink emit-signals=true max-buffers=3 drop=true sync=false",
        width,height,fps,fps);
    GstElement *pipe1 = gst_parse_launch(p1,&err); g_free(p1);
    if (!pipe1){ g_printerr("pipe1: %s\n", err?err->message:"(unknown)"); g_clear_error(&err); return -1; }
    c.appsink = gst_bin_get_by_name(GST_BIN(pipe1),"cap_sink");
    if (!c.appsink){ g_printerr("get appsink failed\n"); gst_object_unref(pipe1); return -1; }

    // Pipeline 2A: even frames
    gchar *p2a = g_strdup_printf(
        "appsrc name=src_even is-live=true format=GST_FORMAT_TIME do-timestamp=true block=true max-bytes=%d ! "
        "video/x-raw(memory:DMABuf),format=NV12,width=%d,height=%d,framerate=30/1; "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=30/1 ! "
        "queue max-size-buffers=12 max-size-time=40000000 leaky=downstream ! "
        "omxh265enc name=enc_even control-rate=low-latency target-bitrate=%d "
        "gop-mode=low-delay-p gop-length=30 periodicity-idr=30 num-slices=%d prefetch-buffer=true filler-data=false ! "
        "video/x-h265,alignment=au,profile=main,stream-format=byte-stream ! "
        "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
        "udpsink host=%s port=%d sync=false",
        8*1024*1024, width,height, width,height, br_each, slices, host, port);

    GstElement *pipe2a = gst_parse_launch(p2a,&err); g_free(p2a);
    if (!pipe2a){ g_printerr("pipe2-even: %s\n", err?err->message:"(unknown)"); g_clear_error(&err); gst_object_unref(pipe1); return -1; }
    c.appsrc_even = gst_bin_get_by_name(GST_BIN(pipe2a),"src_even");
    if (!c.appsrc_even){ g_printerr("get appsrc_even failed\n"); gst_object_unref(pipe2a); gst_object_unref(pipe1); return -1; }

    // Pipeline 2B: odd frames
    gchar *p2b = g_strdup_printf(
        "appsrc name=src_odd is-live=true format=GST_FORMAT_TIME do-timestamp=true block=true max-bytes=%d ! "
        "video/x-raw(memory:DMABuf),format=NV12,width=%d,height=%d,framerate=30/1; "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=30/1 ! "
        "queue max-size-buffers=12 max-size-time=40000000 leaky=downstream ! "
        "omxh265enc name=enc_odd control-rate=low-latency target-bitrate=%d "
        "gop-mode=low-delay-p gop-length=30 periodicity-idr=30 num-slices=%d prefetch-buffer=true filler-data=false ! "
        "video/x-h265,alignment=au,profile=main,stream-format=byte-stream ! "
        "rtph265pay pt=96 mtu=1400 config-interval=1 ! "
        "udpsink host=%s port=%d sync=false",
        8*1024*1024, width,height, width,height, br_each, slices, host, port+2);

    GstElement *pipe2b = gst_parse_launch(p2b,&err); g_free(p2b);
    if (!pipe2b){ g_printerr("pipe2-odd: %s\n", err?err->message:"(unknown)"); g_clear_error(&err); gst_object_unref(pipe2a); gst_object_unref(pipe1); return -1; }
    c.appsrc_odd = gst_bin_get_by_name(GST_BIN(pipe2b),"src_odd");
    if (!c.appsrc_odd){ g_printerr("get appsrc_odd failed\n"); gst_object_unref(pipe2b); gst_object_unref(pipe2a); gst_object_unref(pipe1); return -1; }

    // shared clock across all pipelines (helps pacing)
    GstClock *clk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(pipe1), clk);
    gst_pipeline_use_clock(GST_PIPELINE(pipe2a), clk);
    gst_pipeline_use_clock(GST_PIPELINE(pipe2b), clk);
    gst_object_unref(clk);

    // probes
    GstElement *enc_e = gst_bin_get_by_name(GST_BIN(pipe2a),"enc_even");
    if (enc_e){
        GstPad *sp = gst_element_get_static_pad(enc_e,"sink");
        if (sp){ gst_pad_add_probe(sp, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe_even, NULL, NULL); gst_object_unref(sp); }
        gst_object_unref(enc_e);
    }
    GstElement *enc_o = gst_bin_get_by_name(GST_BIN(pipe2b),"enc_odd");
    if (enc_o){
        GstPad *sp = gst_element_get_static_pad(enc_o,"sink");
        if (sp){ gst_pad_add_probe(sp, GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe_odd, NULL, NULL); gst_object_unref(sp); }
        gst_object_unref(enc_o);
    }
    GstPad *asrc_e = gst_element_get_static_pad(c.appsrc_even,"src");
    if (asrc_e){ gst_pad_add_probe(asrc_e, GST_PAD_PROBE_TYPE_BUFFER, appsrc_out_probe_even, NULL, NULL); gst_object_unref(asrc_e); }
    GstPad *asrc_o = gst_element_get_static_pad(c.appsrc_odd,"src");
    if (asrc_o){ gst_pad_add_probe(asrc_o, GST_PAD_PROBE_TYPE_BUFFER, appsrc_out_probe_odd, NULL, NULL); gst_object_unref(asrc_o); }

    // connect callback, buses, loop, stats
    g_signal_connect(c.appsink, "new-sample", G_CALLBACK(on_new_sample), &c);
    GMainLoop *loop = g_main_loop_new(NULL,FALSE); c.loop=loop;
    GstBus *b1=gst_element_get_bus(pipe1), *b2=gst_element_get_bus(pipe2a), *b3=gst_element_get_bus(pipe2b);
    gst_bus_add_watch(b1,on_bus,loop); gst_bus_add_watch(b2,on_bus,loop); gst_bus_add_watch(b3,on_bus,loop);
    gst_object_unref(b1); gst_object_unref(b2); gst_object_unref(b3);

    g_timer=g_timer_new(); g_timeout_add(100, tick, NULL);
    signal(SIGINT, on_sigint);

    gst_element_set_state(pipe2a, GST_STATE_PLAYING);
    gst_element_set_state(pipe2b, GST_STATE_PLAYING);
    gst_element_set_state(pipe1,  GST_STATE_PLAYING);
    g_print("Running split encoders. Ctrl+C to exit.\n");

    while (!g_stop){ g_main_context_iteration(NULL,FALSE); g_usleep(1000); }

    gst_element_set_state(pipe1,  GST_STATE_NULL);
    gst_element_set_state(pipe2a, GST_STATE_NULL);
    gst_element_set_state(pipe2b, GST_STATE_NULL);

    if (g_timer) g_timer_destroy(g_timer);
    g_mutex_clear(&g_mu);
    g_main_loop_unref(loop);
    gst_object_unref(pipe1);
    gst_object_unref(pipe2a);
    gst_object_unref(pipe2b);
    return 0;
}
