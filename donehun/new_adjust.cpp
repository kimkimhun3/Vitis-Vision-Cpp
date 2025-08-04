#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]) {
    GstElement *pipeline;
    GError *error = nullptr;

    // Default values
    guint input_bitrate_kbps = 2000;
    gchar *resolution_str = (gchar *)"2K";
    guint framerate = 30;

    // Command-line options
    GOptionEntry entries[] = {
        { "bitrate",  0, 0, G_OPTION_ARG_INT,  &input_bitrate_kbps, "Bitrate in kbps (e.g. 5000)", "BITRATE_KBPS" },
        { "in",       0, 0, G_OPTION_ARG_STRING, &resolution_str,   "Resolution (2K or 4K)", "RES" },
        { "fps",      0, 0, G_OPTION_ARG_INT,  &framerate,          "Framerate (e.g. 30, 60)", "FPS" },
        { nullptr }
    };

    GOptionContext *context = g_option_context_new("- GStreamer H.264 RTP Streamer");
    g_option_context_add_main_entries(context, entries, nullptr);

    if (!g_option_context_parse(context, &argc, &argv, &error)) {
        g_printerr("Option parsing failed: %s\n", error->message);
        g_clear_error(&error);
        g_option_context_free(context);
        return -1;
    }

    g_option_context_free(context);
    gst_init(&argc, &argv);

    // Hardcoded input camera maximum capabilities (modify if needed)
    const guint MAX_WIDTH = 3840;
    const guint MAX_HEIGHT = 2160;
    const guint MAX_FPS = 60;

    // Resolution mapping
    guint width = 1920, height = 1080;
    if (g_strcmp0(resolution_str, "4K") == 0) {
        width = 3840;
        height = 2160;
    } else if (g_strcmp0(resolution_str, "2K") == 0) {
        width = 1920;
        height = 1080;
    } else {
        g_printerr("Invalid resolution: %s. Only '2K' or '4K' are supported.\n", resolution_str);
        return -1;
    }

    // Validation
    if (width > MAX_WIDTH || height > MAX_HEIGHT || framerate > MAX_FPS) {
        g_printerr("Requested resolution %ux%u@%u exceeds camera capabilities (%ux%u@%u).\n",
                   width, height, framerate, MAX_WIDTH, MAX_HEIGHT, MAX_FPS);
        return -1;
    }

    // Bitrate settings
    guint target_bitrate_kbps = input_bitrate_kbps * 90 / 100;
    guint max_bitrate_bps = input_bitrate_kbps * 95 / 100 * 1000;

    // Pipeline string generation
    gchar *pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 do-timestamp=false ! "
        "video/x-raw, format=NV12, width=%u, height=%u, framerate=%u/1 ! "
        "videorate drop-only=true max-rate=%u ! "
        "queue ! "
        "omxh264enc target-bitrate=%u num-slices=1 control-rate=Constant "
        "qp-mode=auto prefetch-buffer=true cpb-size=200 initial-delay=200 "
        "gdr-mode=disabled periodicity-idr=90 gop-length=90 filler-data=false ! "
        "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
        "queue ! rtph264pay mtu=1400 ! queue ! "
        "udpsink clients=192.168.25.69:5004 auto-multicast=false max-bitrate=%u",
        width, height, framerate, framerate,
        target_bitrate_kbps,
        max_bitrate_bps
    );

    g_print("Running pipeline:\n%s\n", pipeline_str);

    pipeline = gst_parse_launch(pipeline_str, &error);
    g_free(pipeline_str);

    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (msg != nullptr) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug_info);
                g_printerr("Error received: %s\n", err->message);
                g_free(debug_info);
                g_error_free(err);
                break;
            case GST_MESSAGE_EOS:
                g_print("End-Of-Stream reached.\n");
                break;
            default:
                break;
        }
        gst_message_unref(msg);
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
