#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>

static const guint RES_2K_W = 1920;
static const guint RES_2K_H = 1089;   // per your requirement
static const guint RES_4K_W = 3840;
static const guint RES_4K_H = 2160;

struct CustomData {
    GstElement   *appsink = nullptr;
    GstElement   *appsrc  = nullptr;

    gboolean      video_info_valid = FALSE;
    GstVideoInfo  video_info{};

    gboolean      appsrc_caps_set = FALSE;
    GstClockTime  t0 = GST_CLOCK_TIME_NONE; // for fallback timestamping

    guint         fps = 60;
    gboolean      verbose = TRUE;
};

static GMainLoop *g_loop = nullptr;

static void handle_sigint(int) {
    if (g_loop) g_main_loop_quit(g_loop);
}

static void bus_watch(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus; (void)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = nullptr; gchar *dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            if (dbg) g_printerr("Debug: %s\n", dbg);
            g_clear_error(&err); g_free(dbg);
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("EOS from %s\n", GST_OBJECT_NAME(msg->src));
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        default: break;
    }
}

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *d = static_cast<CustomData*>(user_data);

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buf = gst_sample_get_buffer(sample);
    if (!buf) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    // Discover/propagate caps on first frame
    GstCaps *caps = gst_sample_get_caps(sample);
    if (G_UNLIKELY(!d->video_info_valid)) {
        if (caps && gst_video_info_from_caps(&d->video_info, caps)) {
            d->video_info_valid = TRUE;

            // Push exact upstream caps (size, framerate, format, memory features) to appsrc
            GstCaps *copy = gst_caps_copy(caps);
            gst_app_src_set_caps(GST_APP_SRC(d->appsrc), copy);
            gst_caps_unref(copy);
            d->appsrc_caps_set = TRUE;

            // Scale appsrc budget to resolution (≈ 4 frames)
            guint64 frame_bytes = d->video_info.size;
            guint64 budget = frame_bytes ? frame_bytes * 4ull : (16ull * 1024 * 1024);
            g_object_set(d->appsrc, "max-bytes", budget, NULL);

            if (d->verbose) {
                gint w = GST_VIDEO_INFO_WIDTH(&d->video_info);
                gint h = GST_VIDEO_INFO_HEIGHT(&d->video_info);
                g_print("[caps] %dx%d @ %u/1, format=%s, frame_bytes=%" G_GUINT64_FORMAT ", budget=%" G_GUINT64_FORMAT "\n",
                        w, h, d->fps,
                        gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&d->video_info)),
                        frame_bytes, budget);
            }
        } else {
            if (d->verbose) g_printerr("Failed to parse upstream caps on first sample.\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Ensure valid PTS
    if (!GST_BUFFER_PTS_IS_VALID(buf)) {
        if (G_UNLIKELY(d->t0 == GST_CLOCK_TIME_NONE))
            d->t0 = gst_util_get_timestamp();
        GST_BUFFER_PTS(buf) = gst_util_get_timestamp() - d->t0;
    }

    // Zero-copy: pass buffer by ref
    gst_buffer_ref(buf);
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), buf);

    gst_sample_unref(sample);
    return ret;
}

static gboolean on_appsrc_need_data(GstAppSrc*, guint, gpointer) {
    // Pushed from appsink callback
    return TRUE;
}

int main(int argc, char **argv) {
    // Defaults for your typical run
    std::string device = "/dev/video0";
    std::string host   = "192.168.25.69";
    gint       port    = 5004;
    guint      width   = RES_4K_W;   // default 4K
    guint      height  = RES_4K_H;
    guint      fps     = 60;
    guint      bitrate_kbps = 30000; // kbps (do NOT multiply by 1000)
    std::string codec  = "h264";     // "h264" or "h265"

    // Only 2K and 4K presets per your spec
    bool preset_set = false;

    // Simple CLI
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--device") && i+1 < argc) device = argv[++i];
        else if (!strcmp(argv[i], "--host") && i+1 < argc) host = argv[++i];
        else if (!strcmp(argv[i], "--port") && i+1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--codec") && i+1 < argc) codec = argv[++i];
        else if (!strcmp(argv[i], "--bitrate") && i+1 < argc) bitrate_kbps = (guint)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--2k30")) { width=RES_2K_W; height=RES_2K_H; fps=30; preset_set=true; }
        else if (!strcmp(argv[i], "--2k60")) { width=RES_2K_W; height=RES_2K_H; fps=60; preset_set=true; }
        else if (!strcmp(argv[i], "--4k30")) { width=RES_4K_W; height=RES_4K_H; fps=30; preset_set=true; }
        else if (!strcmp(argv[i], "--4k60")) { width=RES_4K_W; height=RES_4K_H; fps=60; preset_set=true; }
    }
    // If no preset specified, default remains 4K60

    // Normalize codec
    if (codec != "h264" && codec != "h265") codec = "h264";

    gst_init(&argc, &argv);
    signal(SIGINT, handle_sigint);

    CustomData data{};
    data.fps = fps;

    // --- appsink (ingest) pipeline ---
    // Explicit caps + FPS shaping. sync=false to reduce jitter.
    gchar *sink_desc = g_strdup_printf(
        "v4l2src device=%s io-mode=4 ! "
        "video/x-raw,format=NV12,width=%u,height=%u,framerate=%u/1 ! "
        "videorate drop-only=true max-rate=%u ! "
        "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true sync=false",
        device.c_str(), width, height, fps, fps);

    GError *error = nullptr;
    GstElement *sink_pipeline = gst_parse_launch(sink_desc, &error);
    g_free(sink_desc);
    if (!sink_pipeline) {
        g_printerr("Failed to create appsink pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        return 1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "cv_sink");
    if (!data.appsink) { g_printerr("appsink not found\n"); return 1; }

    // --- appsrc (egress) pipeline ---
    // Caps will be set programmatically from the first sample (dynamic; includes memory features).
    std::string enc_line;
    if (codec == "h265") {
        enc_line =
            "omxh265enc gop-mode=low-delay target-bitrate=%u control-rate=low-latency num-slices=4 "
            "periodicity-idr=60 gop-length=60 prefetch-buffer=true ! "
            "video/x-h265,alignment=au ! "
            "rtph265pay mtu=1400 ! ";
    } else {
        enc_line =
            "omxh264enc target-bitrate=%u control-rate=low-latency num-slices=4 "
            "periodicity-idr=60 gop-length=60 filler-data=false prefetch-buffer=true ! "
            "video/x-h264,alignment=au,stream-format=byte-stream,profile=main ! "
            "rtph264pay mtu=1400 ! ";
    }

    gchar *src_desc = g_strdup_printf(
        ("appsrc name=cv_src is-live=true block=true format=GST_FORMAT_TIME do-timestamp=true ! "
         "queue max-size-buffers=2 leaky=downstream ! " // keep latency bounded
         + enc_line +
         "udpsink host=%s port=%d buffer-size=60000000 async=false").c_str(),
        bitrate_kbps,
        host.c_str(), port);

    GstElement *src_pipeline = gst_parse_launch(src_desc, &error);
    g_free(src_desc);
    if (!src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return 1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "cv_src");
    if (!data.appsrc) { g_printerr("appsrc not found\n"); return 1; }

    g_signal_connect(data.appsrc, "need-data", G_CALLBACK(on_appsrc_need_data), &data);

    GstAppSinkCallbacks cbs{};
    cbs.new_sample = new_sample_cb;
    gst_app_sink_set_callbacks(GST_APP_SINK(data.appsink), &cbs, &data, NULL);

    // Buses
    GstBus *bus_sink = gst_element_get_bus(sink_pipeline);
    GstBus *bus_src  = gst_element_get_bus(src_pipeline);
    gst_bus_add_signal_watch(bus_sink);
    gst_bus_add_signal_watch(bus_src);
    g_signal_connect(bus_sink, "message", G_CALLBACK(bus_watch), NULL);
    g_signal_connect(bus_src,  "message", G_CALLBACK(bus_watch), NULL);

    // Start
    gst_element_set_state(src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    g_print("Running zero-copy: %ux%u @ %u fps, codec=%s, bitrate=%u kbps\n",
            width, height, fps, codec.c_str(), bitrate_kbps);
    g_print("Source: %s  →  %s:%d\n", device.c_str(), host.c_str(), port);

    g_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(g_loop);

    // Cleanup
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline,  GST_STATE_NULL);
    if (bus_sink) gst_object_unref(bus_sink);
    if (bus_src)  gst_object_unref(bus_src);
    if (data.appsink) gst_object_unref(data.appsink);
    if (data.appsrc)  gst_object_unref(data.appsrc);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    g_main_loop_unref(g_loop);
    return 0;
}
