// histequalize_host.cpp
// Two-pipeline bridge with monitors + fast CPU path
// Changes in this version:
//  - v4l2src io-mode=2 (MMAP) for CPU-friendly mapping
//  - Copy Y plane into a cached buffer (stride-aware) before equalizeHist
//  - Buffer pool for appsrc output, appsrc block=false
//  - Encoder set to Constant + filler-data for steadier bitrate
//
// Run examples:
//   ./histequalize_host --bitrate=10000 --codec=h265
//   ./histequalize_host --bitrate=10000            (H.264 default)

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <stdlib.h>

typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean video_info_valid;
    GstVideoInfo video_info;
    GTimer *processing_timer;
    double total_processing_time;
    int frame_count;

    // Monitoring elements
    GstElement *cap_q;     // queue before appsink
    GstElement *preenc_q;  // queue before encoder
    GstElement *encoder;   // omxh264enc/omxh265enc
    GstElement *pay;       // rtph26xpay

    // Counters
    GMutex mu;
    guint64 cnt_cam_src=0;       // v4l2src out
    guint64 cnt_sink_in=0;       // into appsink
    guint64 cnt_cb_processed=0;  // after OpenCV callback
    guint64 cnt_appsrc_out=0;    // appsrc out
    guint64 cnt_enc_out=0;       // encoder out
    guint64 cnt_pay_in=0;        // payloader in

    guint64 bytes_cam_src=0;     // raw bytes (NV12)
    guint64 bytes_enc_out=0;     // encoded bytes
    guint64 bytes_pay_in=0;      // encoded bytes into payloader

    // Queue events
    guint64 cap_overruns=0, cap_underruns=0;
    guint64 pre_overruns=0, pre_underruns=0;

    // Last snapshots (for 1-second deltas)
    guint64 last_cnt_cam_src=0, last_cnt_sink_in=0, last_cnt_cb=0;
    guint64 last_cnt_appsrc_out=0, last_cnt_enc_out=0, last_cnt_pay_in=0;
    guint64 last_bytes_cam_src=0, last_bytes_enc_out=0, last_bytes_pay_in=0;

    // Fast path resources
    GstBufferPool *pool = NULL;
    gboolean pool_ready = FALSE;
    cv::Mat y_plane_out_reuse; // persistent output Y
    cv::Mat y_cached;          // persistent cached copy of input Y
} CustomData;

// ---------- Helpers ----------
static inline gsize buf_size(GstBuffer *b) {
#if GST_CHECK_VERSION(1,14,0)
    return gst_buffer_get_size(b);
#else
    GstMapInfo mi;
    if (gst_buffer_map(b, &mi, GST_MAP_READ)) { gsize s = mi.size; gst_buffer_unmap(b,&mi); return s; }
    return 0;
#endif
}

typedef enum { STAGE_CAM_SRC, STAGE_SINK_IN, STAGE_APPSRC_OUT, STAGE_ENCODER_OUT, STAGE_PAY_IN } Stage;
typedef struct { CustomData *data; Stage stage; } ProbeCtx;

static GstPadProbeReturn on_buf_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    if (!(GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER)) return GST_PAD_PROBE_OK;
    GstBuffer *buf = gst_pad_probe_info_get_buffer(info);
    if (!buf) return GST_PAD_PROBE_OK;

    ProbeCtx *ctx = (ProbeCtx*)user_data;
    gsize sz = buf_size(buf);

    g_mutex_lock(&ctx->data->mu);
    switch (ctx->stage) {
        case STAGE_CAM_SRC:    ctx->data->cnt_cam_src++;    ctx->data->bytes_cam_src += sz; break;
        case STAGE_SINK_IN:    ctx->data->cnt_sink_in++;    break;
        case STAGE_APPSRC_OUT: ctx->data->cnt_appsrc_out++; break;
        case STAGE_ENCODER_OUT:ctx->data->cnt_enc_out++;    ctx->data->bytes_enc_out += sz; break;
        case STAGE_PAY_IN:     ctx->data->cnt_pay_in++;     ctx->data->bytes_pay_in += sz; break;
    }
    g_mutex_unlock(&ctx->data->mu);
    return GST_PAD_PROBE_OK;
}

static void on_overrun(GstElement *queue, gpointer user_data) {
    CustomData *d = (CustomData*)user_data;
    g_mutex_lock(&d->mu);
    if (queue == d->cap_q) d->cap_overruns++; else if (queue == d->preenc_q) d->pre_overruns++;
    g_mutex_unlock(&d->mu);
}
static void on_underrun(GstElement *queue, gpointer user_data) {
    CustomData *d = (CustomData*)user_data;
    g_mutex_lock(&d->mu);
    if (queue == d->cap_q) d->cap_underruns++; else if (queue == d->preenc_q) d->pre_underruns++;
    g_mutex_unlock(&d->mu);
}

static gboolean stats_tick(gpointer user_data) {
    CustomData *d = (CustomData*)user_data;

    // snapshot
    g_mutex_lock(&d->mu);
    guint64 cam  = d->cnt_cam_src,  sink = d->cnt_sink_in, cb = d->cnt_cb_processed;
    guint64 src  = d->cnt_appsrc_out, enc = d->cnt_enc_out, pay = d->cnt_pay_in;
    guint64 bcam = d->bytes_cam_src, benc = d->bytes_enc_out, bpay = d->bytes_pay_in;
    guint64 capov = d->cap_overruns, capun = d->cap_underruns;
    guint64 preov = d->pre_overruns, preun = d->pre_underruns;

    guint64 d_cam  = cam  - d->last_cnt_cam_src;      d->last_cnt_cam_src  = cam;
    guint64 d_sink = sink - d->last_cnt_sink_in;      d->last_cnt_sink_in  = sink;
    guint64 d_cb   = cb   - d->last_cnt_cb;           d->last_cnt_cb       = cb;
    guint64 d_src  = src  - d->last_cnt_appsrc_out;   d->last_cnt_appsrc_out = src;
    guint64 d_enc  = enc  - d->last_cnt_enc_out;      d->last_cnt_enc_out  = enc;
    guint64 d_pay  = pay  - d->last_cnt_pay_in;       d->last_cnt_pay_in   = pay;

    guint64 d_bcam = bcam - d->last_bytes_cam_src;    d->last_bytes_cam_src = bcam;
    guint64 d_benc = benc - d->last_bytes_enc_out;    d->last_bytes_enc_out = benc;
    guint64 d_bpay = bpay - d->last_bytes_pay_in;     d->last_bytes_pay_in  = bpay;

    int cap_lvl=0, cap_max=0, pre_lvl=0, pre_max=0;
    g_object_get(d->cap_q,   "current-level-buffers", &cap_lvl,  "max-size-buffers", &cap_max,  NULL);
    g_object_get(d->preenc_q,"current-level-buffers", &pre_lvl,  "max-size-buffers", &pre_max,  NULL);
    g_mutex_unlock(&d->mu);

    // per-second rates
    double fps_in   = (double)d_cam;
    double fps_sink = (double)d_sink;
    double fps_cb   = (double)d_cb;
    double fps_src  = (double)d_src;
    double fps_enc  = (double)d_enc;
    double fps_pay  = (double)d_pay;

    double mbps_raw = d_bcam * 8.0 / 1e6;
    double mbps_enc = d_benc * 8.0 / 1e6;
    double mbps_rtp = d_bpay * 8.0 / 1e6;

    g_print("[MON] FPS in/sink/cb/appsrc/enc/pay = %.1f / %.1f / %.1f / %.1f / %.1f / %.1f | "
            "Mbps raw/enc/rtp = %.2f / %.2f / %.2f | "
            "cap_q=%d/%d ov=%" G_GUINT64_FORMAT " un=%" G_GUINT64_FORMAT " | "
            "preenc_q=%d/%d ov=%" G_GUINT64_FORMAT " un=%" G_GUINT64_FORMAT "\n",
            fps_in, fps_sink, fps_cb, fps_src, fps_enc, fps_pay,
            mbps_raw, mbps_enc, mbps_rtp,
            cap_lvl, cap_max, capov, capun,
            pre_lvl, pre_max, preov, preun);

    if (d_sink > d_cb) {
        g_print("  -> Bottleneck likely in OpenCV callback or appsink drops (sink-cb = %" G_GUINT64_FORMAT ")\n", (d_sink - d_cb));
    } else if (d_cb > d_src) {
        g_print("  -> appsrc backpressure (cb-appsrc = %" G_GUINT64_FORMAT ")\n", (d_cb - d_src));
    } else if (d_src > d_enc) {
        g_print("  -> Encoder lagging (appsrc-enc = %" G_GUINT64_FORMAT ")\n", (d_src - d_enc));
    }
    return TRUE;
}

// ---------- FAST processing callback ----------
GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) { g_printerr("Failed to pull sample from appsink\n"); return GST_FLOW_ERROR; }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) { g_printerr("Failed to get buffer from sample\n"); gst_sample_unref(sample); return GST_FLOW_ERROR; }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Video info: %dx%d\n", data->video_info.width, data->video_info.height);
        } else {
            g_printerr("Failed to extract video info\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Stride-safe mapping
    GstVideoFrame vframe;
    if (!gst_video_frame_map(&vframe, &data->video_info, buffer, GST_MAP_READ)) {
        g_printerr("Failed to map GstVideoFrame\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    int width  = data->video_info.width;
    int height = data->video_info.height;
    size_t y_size  = (size_t)width * (size_t)height;
    size_t uv_size = y_size / 2;

    guint8 *y_ptr = (guint8*)GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0);
    gint    y_stride_g = GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0);
    size_t  y_stride = (y_stride_g > 0) ? (size_t)y_stride_g : (size_t)width;

    // Copy Y into cached CPU buffer (row-wise) — MUCH faster than reading DMABUF in equalizeHist
    data->y_cached.create(height, width, CV_8UC1);
    if (y_stride == (size_t)width) {
        memcpy(data->y_cached.data, y_ptr, y_size);
    } else {
        for (int r = 0; r < height; ++r) {
            memcpy(data->y_cached.ptr(r), y_ptr + (size_t)r * y_stride, (size_t)width);
        }
    }

    // Reuse output mat
    data->y_plane_out_reuse.create(height, width, CV_8UC1);

    // Process
    g_timer_start(data->processing_timer);
    cv::equalizeHist(data->y_cached, data->y_plane_out_reuse);
    g_timer_stop(data->processing_timer);

    double ms = g_timer_elapsed(data->processing_timer, NULL) * 1000.0;
    data->total_processing_time += ms;
    data->frame_count++;
    if (data->frame_count % 100 == 0) {
        double avg = data->total_processing_time / data->frame_count;
        g_print("Stats - Frame %d: %.2f ms, avg: %.2f ms, FPS: %.1f\n",
            data->frame_count, ms, avg, 1000.0 / avg);
    }

    // Lazy-create NV12 output buffer pool (1.5 bytes per pixel)
    if (!data->pool_ready) {
        GstCaps *pool_caps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "NV12",
            "width",  G_TYPE_INT,    width,
            "height", G_TYPE_INT,    height,
            NULL);
        data->pool = gst_buffer_pool_new();
        GstStructure *cfg = gst_buffer_pool_get_config(data->pool);
        gst_buffer_pool_config_set_params(cfg, pool_caps, y_size + uv_size, 4, 8);
        gst_buffer_pool_set_config(data->pool, cfg);
        gst_buffer_pool_set_active(data->pool, TRUE);
        gst_caps_unref(pool_caps);
        data->pool_ready = TRUE;
    }

    // Acquire output buffer from pool
    GstBuffer *processed_buffer = NULL;
    if (gst_buffer_pool_acquire_buffer(data->pool, &processed_buffer, NULL) != GST_FLOW_OK || !processed_buffer) {
        g_printerr("Failed to acquire buffer from pool\n");
        gst_video_frame_unmap(&vframe);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Write NV12: copy Y; UV neutral (or copy original UV if you want color/bitrate↑)
    GstMapInfo outmap;
    if (!gst_buffer_map(processed_buffer, &outmap, GST_MAP_WRITE)) {
        g_printerr("Failed to map output buffer\n");
        gst_buffer_unref(processed_buffer);
        gst_video_frame_unmap(&vframe);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Y copy (contiguous)
    if ((size_t)data->y_plane_out_reuse.step == (size_t)width) {
        memcpy(outmap.data, data->y_plane_out_reuse.data, y_size);
    } else {
        for (int r = 0; r < height; ++r)
            memcpy(outmap.data + (size_t)r * width, data->y_plane_out_reuse.ptr(r), (size_t)width);
    }

    // If you want color + higher bitrate, copy UV from input instead of neutral 128:
    // memcpy(outmap.data + y_size, (guint8*)GST_VIDEO_FRAME_PLANE_DATA(&vframe, 1), uv_size);
    memset(outmap.data + y_size, 128, uv_size);

    gst_buffer_unmap(processed_buffer, &outmap);

    // carry timestamps
    gst_buffer_copy_into(processed_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

    // push downstream
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), processed_buffer);
    if (ret != GST_FLOW_OK) g_printerr("Failed to push buffer to appsrc: %d\n", ret);

    g_mutex_lock(&data->mu);
    data->cnt_cb_processed++;
    g_mutex_unlock(&data->mu);

    gst_video_frame_unmap(&vframe);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// ---------- Bus watch ----------
static gboolean on_bus_msg(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus;
    GMainLoop *loop = (GMainLoop*)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err=NULL; gchar *dbg=NULL;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("[BUS] ERROR: %s\n", err?err->message:"(unknown)");
            if (dbg) g_printerr("[BUS] debug: %s\n", dbg);
            if (err) g_error_free(err);
            g_free(dbg);
            g_main_loop_quit(loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("[BUS] EOS\n");
            g_main_loop_quit(loop);
            break;
        default: break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // Parse only --codec and --bitrate (kbps)
    gboolean use_h265 = FALSE;
    int bitrate_kbps = 20000; // default

    for (int i = 1; i < argc; ++i) {
        if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *val = strchr(argv[i], '=');
            if (val && g_ascii_strcasecmp(val + 1, "h265") == 0) use_h265 = TRUE;
        } else if (g_strcmp0(argv[i], "--codec") == 0 && i + 1 < argc) {
            if (g_ascii_strcasecmp(argv[i + 1], "h265") == 0) use_h265 = TRUE;
        } else if (g_str_has_prefix(argv[i], "--bitrate=")) {
            const char *val = strchr(argv[i], '=');
            if (val) { int v = atoi(val + 1); if (v > 0) bitrate_kbps = v; }
        } else if (g_strcmp0(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            int v = atoi(argv[i + 1]); if (v > 0) bitrate_kbps = v;
        }
    }

    g_print("Encoder: %s, target-bitrate: %d kbps\n", use_h265 ? "H.265" : "H.264", bitrate_kbps);

    CustomData data = {};
    data.video_info_valid = FALSE;
    data.processing_timer = g_timer_new();
    g_mutex_init(&data.mu);

    GError *error = NULL;

    // 1) Capture pipeline — switch to io-mode=2 (MMAP) for CPU
    gchar *sink_pipeline_str =
        g_strdup_printf("v4l2src name=cam device=/dev/video0 io-mode=2 ! "
                        "video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1 ! "
                        "queue name=cap_q max-size-buffers=8 leaky=downstream ! "
                        "appsink name=my_sink emit-signals=true max-buffers=2 drop=true sync=false");
    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) { g_printerr("Failed to create sink pipeline: %s\n", error->message); g_clear_error(&error); return -1; }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");
    data.cap_q   = gst_bin_get_by_name(GST_BIN(sink_pipeline), "cap_q");

    // 2) Streaming pipeline — appsrc block=false; encoder Constant + filler-data
    gchar *src_pipeline_str = NULL;
    if (use_h265) {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true block=false ! "
            "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
            "queue name=preenc_q max-size-buffers=8 leaky=downstream ! "
            "omxh265enc name=enc num-slices=1 periodicity-idr=60 cpb-size=200 gdr-mode=disabled "
            "initial-delay=200 control-rate=Constant prefetch-buffer=true filler-data=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h265, alignment=au ! "
            "rtph265pay name=pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            bitrate_kbps
        );
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true block=false ! "
            "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
            "queue name=preenc_q max-size-buffers=8 leaky=downstream ! "
            "omxh264enc name=enc num-slices=1 periodicity-idr=60 cpb-size=200 gdr-mode=disabled "
            "initial-delay=200 control-rate=Constant qp-mode=fixed prefetch-buffer=true filler-data=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h264, alignment=nal ! "
            "rtph264pay name=pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            bitrate_kbps
        );
    }
    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) { g_printerr("Failed to create src pipeline: %s\n", error->message); g_clear_error(&error); gst_object_unref(sink_pipeline); return -1; }
    data.appsrc  = gst_bin_get_by_name(GST_BIN(src_pipeline),  "my_src");
    data.preenc_q= gst_bin_get_by_name(GST_BIN(src_pipeline),  "preenc_q");
    data.encoder = gst_bin_get_by_name(GST_BIN(src_pipeline),  "enc");
    data.pay     = gst_bin_get_by_name(GST_BIN(src_pipeline),  "pay");

    // ---- Attach pad probes ----
    { // cam src pad
        GstElement *cam = gst_bin_get_by_name(GST_BIN(sink_pipeline), "cam");
        GstPad *p = gst_element_get_static_pad(cam, "src");
        ProbeCtx *ctx = (ProbeCtx*)g_malloc0(sizeof(ProbeCtx)); ctx->data=&data; ctx->stage=STAGE_CAM_SRC;
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, on_buf_probe, ctx, (GDestroyNotify)g_free);
        gst_object_unref(p); gst_object_unref(cam);
    }
    { // appsink sink pad
        GstPad *p = gst_element_get_static_pad(data.appsink, "sink");
        ProbeCtx *ctx = (ProbeCtx*)g_malloc0(sizeof(ProbeCtx)); ctx->data=&data; ctx->stage=STAGE_SINK_IN;
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, on_buf_probe, ctx, (GDestroyNotify)g_free);
        gst_object_unref(p);
    }
    { // appsrc src pad
        GstPad *p = gst_element_get_static_pad(data.appsrc, "src");
        ProbeCtx *ctx = (ProbeCtx*)g_malloc0(sizeof(ProbeCtx)); ctx->data=&data; ctx->stage=STAGE_APPSRC_OUT;
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, on_buf_probe, ctx, (GDestroyNotify)g_free);
        gst_object_unref(p);
    }
    { // encoder src pad
        GstPad *p = gst_element_get_static_pad(data.encoder, "src");
        ProbeCtx *ctx = (ProbeCtx*)g_malloc0(sizeof(ProbeCtx)); ctx->data=&data; ctx->stage=STAGE_ENCODER_OUT;
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, on_buf_probe, ctx, (GDestroyNotify)g_free);
        gst_object_unref(p);
    }
    { // payloader sink pad
        GstPad *p = gst_element_get_static_pad(data.pay, "sink");
        ProbeCtx *ctx = (ProbeCtx*)g_malloc0(sizeof(ProbeCtx)); ctx->data=&data; ctx->stage=STAGE_PAY_IN;
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, on_buf_probe, ctx, (GDestroyNotify)g_free);
        gst_object_unref(p);
    }

    // Queue over/under-run callbacks (may be unavailable on some builds)
    g_signal_connect(data.cap_q,   "overrun",  G_CALLBACK(on_overrun),  &data);
    g_signal_connect(data.cap_q,   "underrun", G_CALLBACK(on_underrun), &data);
    g_signal_connect(data.preenc_q,"overrun",  G_CALLBACK(on_overrun),  &data);
    g_signal_connect(data.preenc_q,"underrun", G_CALLBACK(on_underrun), &data);

    // Appsink callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // --- Start pipelines ---
    gst_element_set_state(src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    // --- MAIN LOOP for timers/probes ---
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus1 = gst_element_get_bus(sink_pipeline);
    GstBus *bus2 = gst_element_get_bus(src_pipeline);
    gst_bus_add_watch(bus1, on_bus_msg, loop);
    gst_bus_add_watch(bus2, on_bus_msg, loop);
    g_object_unref(bus1);
    g_object_unref(bus2);

    g_timeout_add_seconds(1, stats_tick, &data);

    g_print("Running. Press Ctrl+C to stop.\n");
    g_main_loop_run(loop);

    // Cleanup
    g_main_loop_unref(loop);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline,  GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    if (data.processing_timer) g_timer_destroy(data.processing_timer);
    if (data.pool) {
        gst_buffer_pool_set_active(data.pool, FALSE);
        g_object_unref(data.pool);
    }
    g_mutex_clear(&data.mu);
    return 0;
}
