// histequalize_host_single_pipeline.c — Low-latency, single-pipeline streaming.
#include <gst/gst.h>
#include <glib.h>
#include <stdlib.h>
#include <signal.h>

static GMainLoop *g_loop = NULL;
static void handle_sigint(int signum) {
    (void)signum;
    if (g_loop) {
        g_main_loop_quit(g_loop);
    }
}

static gboolean on_bus_message(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus; (void)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("[BUS] ERROR from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
            g_clear_error(&err); g_free(dbg);
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("[BUS] EOS\n");
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        case GST_MESSAGE_WARNING: {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_warning(msg, &err, &dbg);
            g_printerr("[BUS] WARN from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            if (dbg) g_printerr("[BUS] DEBUG: %s\n", dbg);
            g_clear_error(&err); g_free(dbg);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // ---- CLI Parameters ----
    gint bitrate_kbps = 10000;
    gint fps = 60;
    gint width = 1920;
    gint height = 1080;
    const gchar *device = "/dev/video0";
    const gchar *dst_host = "192.168.25.69";
    gint dst_port = 5004;
    gboolean use_h265 = FALSE;

    // Basic CLI parsing
    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate=")) bitrate_kbps = atoi(argv[i] + 10);
        else if (g_str_has_prefix(argv[i], "--fps=")) fps = atoi(argv[i] + 6);
        else if (g_str_has_prefix(argv[i], "--width=")) width = atoi(argv[i] + 8);
        else if (g_str_has_prefix(argv[i], "--height=")) height = atoi(argv[i] + 9);
        else if (g_str_has_prefix(argv[i], "--device=")) device = argv[i] + 9;
        else if (g_str_has_prefix(argv[i], "--host=")) dst_host = argv[i] + 7;
        else if (g_str_has_prefix(argv[i], "--port=")) dst_port = atoi(argv[i] + 7);
        else if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *c = argv[i] + 8;
            use_h265 = (!g_ascii_strcasecmp(c, "h265") || !g_ascii_strcasecmp(c, "hevc"));
        }
    }

    g_print("Using single-pipeline model for low latency.\n");
    g_print("Device=%s  %dx%d@%dfps  Bitrate=%dkbps  Codec=%s  dst=%s:%d\n",
            device, width, height, fps, bitrate_kbps,
            (use_h265 ? "H.265" : "H.264"), dst_host, dst_port);

    // ---- Build the single, unified pipeline ----
    GString *pipeline_str = g_string_new("");
    gint idr_interval = fps * 2; // Keyframe every 2 seconds

    // 1. V4L2 Source
    g_string_append_printf(pipeline_str, "v4l2src device=%s io-mode=dmabuf ! "
                           "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! ",
                           device, width, height, fps);

    // 2. Queue: A small buffer to decouple the source from the encoder.
    // This prevents v4l2src from dropping frames if the encoder briefly stalls.
    g_string_append(pipeline_str, "queue max-size-buffers=5 leaky=no ! ");

    // 3. Encoder (H.264 or H.265)
    if (!use_h265) {
        g_string_append_printf(pipeline_str,
            "omxh264enc control-rate=low-latency target-bitrate=%d periodicity-idr=%d "
            "gop-mode=low-delay-p ! video/x-h264,stream-format=byte-stream,alignment=nal,profile=high ! ",
            bitrate_kbps * 1000, idr_interval);
        // 4. RTP Payloader for H.264
        g_string_append(pipeline_str, "rtph264pay pt=96 config-interval=-1 ! ");
    } else {
        g_string_append_printf(pipeline_str,
            "omxh265enc control-rate=low-latency target-bitrate=%d periodicity-idr=%d "
            "gop-mode=low-delay-p ! video/x-h265,stream-format=byte-stream,alignment=au,profile=main ! ",
            bitrate_kbps * 1000, idr_interval);
        // 4. RTP Payloader for H.265
        g_string_append(pipeline_str, "rtph265pay pt=96 config-interval=-1 ! ");
    }

    // 5. Queue: A small buffer to decouple the encoder from the network.
    g_string_append(pipeline_str, "queue max-size-buffers=5 leaky=no ! ");
    
    // 6. UDP Sink: Send the data. sync=false is crucial for low-latency live streams.
    g_string_append_printf(pipeline_str, "udpsink host=%s port=%d sync=false", dst_host, dst_port);

    // ---- Run the Pipeline ----
    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(pipeline_str->str, &error);
    g_string_free(pipeline_str, TRUE);

    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n", error ? error->message : "unknown error");
        if (error) g_clear_error(&error);
        return -1;
    }

    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, on_bus_message, NULL);
    gst_object_unref(bus);

    g_loop = g_main_loop_new(NULL, FALSE);
    signal(SIGINT, handle_sigint);

    g_print("Starting pipeline. Press Ctrl+C to exit.\n");
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_main_loop_run(g_loop);

    // ---- Teardown ----
    g_print("Stopping pipeline...\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);
    g_main_loop_unref(g_loop);
    gst_object_unref(pipeline);
    
    return 0;
}