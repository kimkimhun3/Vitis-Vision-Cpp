// bridge.c - appsink -> appsrc zero-copy bridge with DMABuf, tuned for 60fps
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

// ---------- Signal handling to quit cleanly ----------
static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int signum) {
    (void)signum;
    g_stop = 1;
}

// ---------- Bus watch ----------
static gboolean on_bus_message(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus;
    GMainLoop *loop = (GMainLoop *)user_data;

    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError *err = NULL;
        gchar *debug = NULL;
        gst_message_parse_error(msg, &err, &debug);
        g_printerr("[BUS] ERROR: %s\n", err->message);
        if (debug) g_printerr("[BUS] DEBUG: %s\n", debug);
        g_clear_error(&err);
        g_free(debug);
        if (loop) g_main_loop_quit(loop);
        break;
    }
    case GST_MESSAGE_EOS:
        g_printerr("[BUS] EOS\n");
        if (loop) g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_WARNING: {
        GError *err = NULL;
        gchar *debug = NULL;
        gst_message_parse_warning(msg, &err, &debug);
        g_printerr("[BUS] WARNING: %s\n", err->message);
        if (debug) g_printerr("[BUS] DEBUG: %s\n", debug);
        g_clear_error(&err);
        g_free(debug);
        break;
    }
    default:
        break;
    }
    return TRUE; // keep watching
}

// ---------- appsink -> appsrc bridge callback ----------
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("Failed to pull sample from appsink\n");
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("Failed to get buffer from sample\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Cache video info once
    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Negotiated: %dx%d, format=%s, fps=%d/%d\n",
                    data->video_info.width, data->video_info.height,
                    gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&data->video_info)),
                    data->video_info.fps_n, data->video_info.fps_d);
        } else {
            g_printerr("Failed to parse video info from caps\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Zero-copy pass-through: ref and push the same buffer into appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), gst_buffer_ref(buffer));

    gst_sample_unref(sample);
    return ret;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // ---- Parse CLI ----
    gint bitrate_kbps = 8000; // default kbps
    gint fps          = 60;   // default fps
    const gchar *device = "/dev/video0";
    const gchar *dst_host = "192.168.25.69";
    gint dst_port = 5004;

    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate=")) {
            bitrate_kbps = atoi(argv[i] + strlen("--bitrate="));
            if (bitrate_kbps <= 0) {
                g_printerr("Invalid --bitrate value.\n");
                return -1;
            }
        } else if (g_str_has_prefix(argv[i], "--fps=")) {
            fps = atoi(argv[i] + strlen("--fps="));
            if (fps <= 0 || fps > 240) {
                g_printerr("Invalid --fps value.\n");
                return -1;
            }
        } else if (g_str_has_prefix(argv[i], "--device=")) {
            device = argv[i] + strlen("--device=");
        } else if (g_str_has_prefix(argv[i], "--host=")) {
            dst_host = argv[i] + strlen("--host=");
        } else if (g_str_has_prefix(argv[i], "--port=")) {
            dst_port = atoi(argv[i] + strlen("--port="));
        }
    }

    g_print("Using device=%s, FPS=%d, Bitrate=%d kbps, dst=%s:%d\n",
            device, fps, bitrate_kbps, dst_host, dst_port);

    // ---- Build pipelines ----
    CustomData data = {0};
    GError *error = NULL;

    // appsink (capture) pipeline
    // Force DMABuf all the way to appsink; videorate drops to requested fps
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=4 do-timestamp=true ! "
        "video/x-raw(memory:DMABuf), format=NV12, width=1920, height=1080, framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true sync=false",
        device, fps);

    GstElement *app_sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!app_sink_pipeline) {
        g_printerr("Failed to create appsink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    data.appsink = gst_bin_get_by_name(GST_BIN(app_sink_pipeline), "cv_sink");
    if (!data.appsink) {
        g_printerr("Failed to get appsink element\n");
        gst_object_unref(app_sink_pipeline);
        return -1;
    }

    // appsrc (encode/stream) pipeline
    // Caps include memory:DMABuf; small leaky queue to keep latency down
    gchar *src_pipeline_str = g_strdup_printf(
        "appsrc name=cv_src format=GST_FORMAT_TIME ! "
        "video/x-raw(memory:DMABuf), format=NV12, width=1920, height=1080, framerate=%d/1 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "omxh265enc control-rate=constant target-bitrate=%d "
        "gop-mode=low-delay-p gop-length=%d periodicity-idr=%d "
        "num-slices=8 qp-mode=auto "
        "prefetch-buffer=true cpb-size=500 initial-delay=250 gdr-mode=horizontal "
        "! video/x-h265, alignment=au, profile=main, stream-format=byte-stream "
        "! rtph265pay pt=96 mtu=1400 config-interval=1 "
        "! queue max-size-buffers=2 leaky=downstream "
        "! udpsink host=%s port=%d sync=true",
        fps, bitrate_kbps, fps, fps, dst_host, dst_port);

    GstElement *app_src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!app_src_pipeline) {
        g_printerr("Failed to create appsrc pipeline: %s\n", error->message);
        g_clear_error(&error);
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

    // ---- Configure appsrc as a live source and set caps directly (matches capsfilter) ----
    g_object_set(data.appsrc,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,   // appsrc stamps running-time; stable pacing at 60fps
                 "block", TRUE,          // backpressure if downstream is slow
                 NULL);

    // Also set caps on appsrc itself (without features) for clean negotiation
    GstCaps *appsrc_caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width",  G_TYPE_INT,    1920,
        "height", G_TYPE_INT,    1080,
        "framerate", GST_TYPE_FRACTION, fps, 1, NULL);
    gst_app_src_set_caps(GST_APP_SRC(data.appsrc), appsrc_caps);
    gst_caps_unref(appsrc_caps);

    // ---- Hook the callback bridge ----
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // ---- Share the same clock across both pipelines (reduces drift) ----
    GstClock *sysclk = gst_system_clock_obtain();
    gst_pipeline_use_clock(GST_PIPELINE(app_sink_pipeline), sysclk);
    gst_pipeline_use_clock(GST_PIPELINE(app_src_pipeline),  sysclk);
    gst_object_unref(sysclk);

    // ---- Bus watches on both pipelines ----
    data.loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus_sink = gst_element_get_bus(app_sink_pipeline);
    GstBus *bus_src  = gst_element_get_bus(app_src_pipeline);
    gst_bus_add_watch(bus_sink, on_bus_message, data.loop);
    gst_bus_add_watch(bus_src,  on_bus_message, data.loop);
    gst_object_unref(bus_sink);
    gst_object_unref(bus_src);

    // ---- Start pipelines (PAUSED -> PLAYING) ----
    gst_element_set_state(app_src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(app_sink_pipeline, GST_STATE_PLAYING);

    // Ctrl+C handling
    signal(SIGINT, handle_sigint);

    g_print("Running. Press Ctrl+C to exit.\n");
    // Run main loop until SIGINT or bus error/EOS
    while (!g_stop) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000); // 1ms
    }

    g_print("Stopping...\n");
    gst_element_set_state(app_sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(app_src_pipeline,  GST_STATE_NULL);
    g_main_loop_unref(data.loop);

    gst_object_unref(app_sink_pipeline);
    gst_object_unref(app_src_pipeline);

    return 0;
}
