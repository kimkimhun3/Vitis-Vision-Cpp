// build: g++ -std=c++17 -O2 pass_through_nv12.cpp -o pass_nv12 `pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0`
// run:   ./pass_nv12            (defaults to 8000 kbps)
//    or: ./pass_nv12 12000      (sets 12,000 kbps)

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>

struct Bridge {
    GstElement *appsrc = nullptr;
    gboolean caps_set = FALSE;
};

static GMainLoop *loop = nullptr;

static gboolean bus_cb(GstBus *bus, GstMessage *msg, gpointer) {
    (void)bus;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = nullptr; gchar *dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            if (dbg) { g_printerr("  debug: %s\n", dbg); g_free(dbg); }
            g_error_free(err);
            if (loop) g_main_loop_quit(loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("EOS\n");
            if (loop) g_main_loop_quit(loop);
            break;
        default: break;
    }
    return TRUE;
}

// appsink -> appsrc zero-copy forward
static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data) {
    Bridge *bridge = static_cast<Bridge*>(user_data);

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    GstCaps   *caps  = gst_sample_get_caps(sample);
    if (!inbuf || !caps) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (!bridge->caps_set) {
        g_object_set(bridge->appsrc, "caps", caps, NULL);
        bridge->caps_set = TRUE;
        gchar *s = gst_caps_to_string(caps);
        g_print("Appsrc caps set: %s\n", s);
        g_free(s);
    }

    gst_buffer_ref(inbuf); // zero-copy: forward the same buffer
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(bridge->appsrc), inbuf);
    gst_sample_unref(sample);

    if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
        g_printerr("appsrc push returned %d\n", ret);
    }
    return GST_FLOW_OK;
}

static void handle_sigint(int) { if (loop) g_main_loop_quit(loop); }

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    signal(SIGINT, handle_sigint);

    // Bitrate: only CLI parameter (kbps). Default 8000 kbps
    guint bitrate_kbps = 8000;
    if (argc >= 2) {
        long v = std::strtol(argv[1], nullptr, 10);
        if (v > 0) bitrate_kbps = static_cast<guint>(v);
    }
    guint bitrate_bps = bitrate_kbps * 1000;
    g_print("Using bitrate: %u kbps (%u bps)\n", bitrate_kbps, bitrate_bps);

    // ---- Pipeline A: /dev/video0 → appsink (NV12 1920x1080@60) ----
    std::string pipeA_desc =
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "appsink name=cv_sink emit-signals=true sync=false max-buffers=1 drop=true";

    GError *err = nullptr;
    GstElement *pipeA = gst_parse_launch(pipeA_desc.c_str(), &err);
    if (!pipeA) {
        g_printerr("Failed to create capture pipeline: %s\n", err ? err->message : "unknown");
        if (err) g_clear_error(&err);
        return 1;
    }
    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeA), "cv_sink");
    if (!appsink) { g_printerr("appsink not found\n"); return 1; }

    // ---- Pipeline B: appsrc → omxh264enc → RTP/UDP ----
    // target-bitrate expects bits per second.
    char pipeB_desc[2048];
    snprintf(pipeB_desc, sizeof(pipeB_desc),
        "appsrc name=cv_src is-live=true do-timestamp=true format=time block=true "
        "min-latency=0 max-latency=0 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "omxh264enc control-rate=low-latency gop-mode=basic periodicity-idr=120 "
        "target-bitrate=%u prefetch-buffer=true "
        "! video/x-h264,stream-format=byte-stream,alignment=au "
        "! h264parse config-interval=-1 "
        "! rtph264pay pt=96 config-interval=-1 "
        "! udpsink clients=192.168.25.69:5004 sync=false async=false",
        bitrate_bps
    );

    GstElement *pipeB = gst_parse_launch(pipeB_desc, &err);
    if (!pipeB) {
        g_printerr("Failed to create encoder pipeline: %s\n", err ? err->message : "unknown");
        if (err) g_clear_error(&err);
        gst_object_unref(appsink);
        gst_object_unref(pipeA);
        return 1;
    }
    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(pipeB), "cv_src");
    if (!appsrc) {
        g_printerr("appsrc not found\n");
        gst_object_unref(pipeB);
        gst_object_unref(appsink);
        gst_object_unref(pipeA);
        return 1;
    }

    // Pre-set appsrc caps (first sample will confirm/overwrite)
    {
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "format",    G_TYPE_STRING, "NV12",
            "width",     G_TYPE_INT,     1920,
            "height",    G_TYPE_INT,     1080,
            "framerate", GST_TYPE_FRACTION, 60, 1,
            NULL);
        g_object_set(appsrc, "caps", caps, NULL);
        gst_caps_unref(caps);
    }

    Bridge bridge{appsrc, FALSE};
    GstAppSinkCallbacks cbs{};
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &cbs, &bridge, nullptr);

    // Bus watches
    GstBus *busA = gst_element_get_bus(pipeA);
    GstBus *busB = gst_element_get_bus(pipeB);
    gst_bus_add_watch(busA, bus_cb, nullptr);
    gst_bus_add_watch(busB, bus_cb, nullptr);

    // Start downstream first
    gst_element_set_state(pipeB, GST_STATE_PLAYING);
    gst_element_set_state(pipeA, GST_STATE_PLAYING);

    g_print("Running… Ctrl+C to stop.\n");
    loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    // Shutdown
    gst_element_send_event(pipeB, gst_event_new_eos());
    gst_element_set_state(pipeA, GST_STATE_NULL);
    gst_element_set_state(pipeB, GST_STATE_NULL);
    gst_object_unref(busA);
    gst_object_unref(busB);
    gst_object_unref(appsink);
    gst_object_unref(appsrc);
    gst_object_unref(pipeA);
    gst_object_unref(pipeB);
    g_main_loop_unref(loop);
    return 0;
}
