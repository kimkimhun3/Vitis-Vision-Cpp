// build: g++ -std=c++17 -Wall -O2 pass_through_nv12.cpp -o pass_nv12 `pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0`
// run:   ./pass_nv12 /dev/video0

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <csignal>
#include <cstdio>
#include <string>

struct Bridge {
    GstElement *appsrc = nullptr;
    gboolean caps_set = FALSE;
};

static GMainLoop *loop = nullptr;

static gboolean bus_cb(GstBus *bus, GstMessage *msg, gpointer user_data) {
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
            g_print("EOS received\n");
            if (loop) g_main_loop_quit(loop);
            break;
        default: break;
    }
    return TRUE;
}

// appsink "new-sample" callback: forward buffer to appsrc with minimal overhead
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

    // Set appsrc caps once (exactly match upstream caps)
    if (!bridge->caps_set) {
        g_object_set(bridge->appsrc, "caps", caps, NULL);
        bridge->caps_set = TRUE;
        // Optional: print negotiated caps
        gchar *s = gst_caps_to_string(caps);
        g_print("Appsrc caps set to: %s\n", s);
        g_free(s);
    }

    // Forward timestamps to preserve pacing across the bridge
    // We will reuse the same buffer (zero-copy) by ref'ing it.
    gst_buffer_ref(inbuf);

    // Push straight into appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(bridge->appsrc), inbuf);

    // Drop our sample reference
    gst_sample_unref(sample);

    if (ret != GST_FLOW_OK) {
        // If downstream is not ready, you might see FLUSHING during state changes; treat as non-fatal.
        if (ret == GST_FLOW_FLUSHING) return GST_FLOW_OK;
        g_printerr("appsrc push returned %d\n", ret);
    }
    return GST_FLOW_OK;
}

static void handle_sigint(int) {
    if (loop) g_main_loop_quit(loop);
}

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    signal(SIGINT, handle_sigint);

    const char *device = (argc > 1) ? argv[1] : "/dev/video0";

    // ---- Pipeline A: capture → appsink (NV12, 1920x1080@60) ----
    // io-mode=4 (DMABuf) if supported by your v4l2 driver; safe to keep even if it falls back.
    std::string pipeA_desc =
        "v4l2src name=cam io-mode=4 ! "
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
    if (!appsink) {
        g_printerr("appsink not found\n");
        return 1;
    }

    // ---- Pipeline B: appsrc → omxh264enc → RTP → UDP ----
    // Note: use your platform’s encoder element (omxh264enc assumed).
    // `clients=192.168.25.69:5004` sends to the requested destination.
    std::string pipeB_desc =
        "appsrc name=cv_src is-live=true do-timestamp=true format=time block=true "
        "min-latency=0 max-latency=0 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "omxh264enc "
            "control-rate=low-latency "
            "gop-mode=basic "
            "periodicity-idr=120 "
            "target-bitrate=10000 "
            "prefetch-buffer=true "
        " ! video/x-h264,stream-format=byte-stream,alignment=au "
        " ! h264parse config-interval=-1 "
        " ! rtph264pay pt=96 config-interval=-1 "
        " ! udpsink clients=192.168.25.69:5004 sync=false async=false";

    GstElement *pipeB = gst_parse_launch(pipeB_desc.c_str(), &err);
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

    // (Optional) Pre-set appsrc caps to match our target; they will be overwritten by exact caps from first sample
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

    // Wire callback
    Bridge bridge;
    bridge.appsrc = appsrc;
    GstAppSinkCallbacks cbs = {};
    cbs.eos = nullptr;
    cbs.new_preroll = nullptr;
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &cbs, &bridge, nullptr);

    // Buses for both pipelines
    GstBus *busA = gst_element_get_bus(pipeA);
    GstBus *busB = gst_element_get_bus(pipeB);
    gst_bus_add_watch(busA, bus_cb, nullptr);
    gst_bus_add_watch(busB, bus_cb, nullptr);

    // Set both to PLAYING (encoder first so appsrc has a consumer)
    gst_element_set_state(pipeB, GST_STATE_PLAYING);
    gst_element_set_state(pipeA, GST_STATE_PLAYING);

    g_print("Running… Ctrl+C to stop.\n");
    loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    // Tear down
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
