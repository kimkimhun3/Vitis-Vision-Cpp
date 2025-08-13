// build: g++ -std=c++17 -O2 appbridge_like_gstlaunch.cpp -o appbridge_like_gstlaunch \
/* */ `pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0`
//
// run:   ./appbridge_like_gstlaunch            (defaults to 8000 kbps)
//     or ./appbridge_like_gstlaunch 10000      (10 Mbps)

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <csignal>
#include <cstdlib>

struct Bridge {
    GstElement *appsrc = nullptr;
    gboolean caps_set = FALSE;
};

static GMainLoop *loop = nullptr;

static gboolean bus_cb(GstBus*, GstMessage *msg, gpointer) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err=nullptr; gchar *dbg=nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            if (dbg) { g_printerr("  debug: %s\n", dbg); g_free(dbg); }
            g_error_free(err);
            if (loop) g_main_loop_quit(loop);
            break;
        }
        case GST_MESSAGE_EOS:
            if (loop) g_main_loop_quit(loop);
            break;
        default: break;
    }
    return TRUE;
}

// Zero-copy forward: ref incoming buffer and push to appsrc
static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data) {
    Bridge *b = static_cast<Bridge*>(user_data);
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    GstCaps   *caps  = gst_sample_get_caps(sample);
    if (!inbuf || !caps) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    if (!b->caps_set) {
        // Use the exact upstream caps (includes NV12/size/fps and memory type if present)
        g_object_set(b->appsrc, "caps", caps, NULL);
        b->caps_set = TRUE;
        gchar *s = gst_caps_to_string(caps);
        g_print("Appsrc caps set: %s\n", s);
        g_free(s);
    }

    gst_buffer_ref(inbuf); // zero-copy reuse
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(b->appsrc), inbuf);
    gst_sample_unref(sample);

    if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING)
        g_printerr("appsrc push returned %d\n", ret);
    return GST_FLOW_OK;
}

static void on_sigint(int){ if (loop) g_main_loop_quit(loop); }

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    std::signal(SIGINT, on_sigint);

    // Only CLI arg: bitrate (kbps). Default 8000 kbps.
    guint bitrate_kbps = 8000;
    if (argc >= 2) {
        long v = std::strtol(argv[1], nullptr, 10);
        if (v > 0) bitrate_kbps = (guint)v;
    }
    guint bitrate_bps = bitrate_kbps * 1000;
    g_print("Bitrate target: %u kbps\n", bitrate_kbps);

    // Elements (single pipeline like your gst-launch line)
    GstElement *pipeline = gst_pipeline_new("p");
    GstElement *v4l2src  = gst_element_factory_make("v4l2src", "cam");
    GstElement *cap_raw  = gst_element_factory_make("capsfilter", "cap_raw");
    GstElement *appsink  = gst_element_factory_make("appsink", "cv_sink");

    GstElement *appsrc   = gst_element_factory_make("appsrc", "cv_src");
    GstElement *q_after  = gst_element_factory_make("queue", "q_after_src");
    GstElement *enc      = gst_element_factory_make("omxh264enc", "enc");
    GstElement *cap_h264 = gst_element_factory_make("capsfilter", "cap_h264");
    GstElement *pay      = gst_element_factory_make("rtph264pay", "pay");
    GstElement *udp      = gst_element_factory_make("udpsink", "udp");

    if (!pipeline||!v4l2src||!cap_raw||!appsink||!appsrc||!q_after||!enc||!cap_h264||!pay||!udp) {
        g_printerr("Failed to create elements\n");
        return 1;
    }

    // --- Configure capture like your gst-launch ---
    g_object_set(v4l2src, "device", "/dev/video0", "io-mode", 4, NULL);
    GstCaps *caps = gst_caps_from_string("video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1");
    g_object_set(cap_raw, "caps", caps, NULL);
    gst_caps_unref(caps);

    // appsink: non-blocking handoff
    g_object_set(appsink,
        "emit-signals", TRUE,
        "sync", FALSE,
        "max-buffers", 1,
        "drop", TRUE,
        "enable-last-sample", FALSE,
        NULL);

    // --- Appsrc & encoder chain set to mirror gst-launch behavior ---
    g_object_set(appsrc,
        "is-live", TRUE,
        "block", TRUE,              // let encoder pacing backpressure here
        "format", GST_FORMAT_TIME,
        "do-timestamp", FALSE,      // preserve upstream PTS/DTS
        "stream-type", GST_APP_STREAM_TYPE_STREAM,
        "max-bytes", (guint64)0,    // unlimited (like direct link)
        NULL);

    // small leaky queue to avoid HOL blocking if encoder jitters
    g_object_set(q_after,
        "max-size-buffers", 8,
        "max-size-bytes", 0,
        "max-size-time", 0,
        "leaky", 2, /* downstream */,
        NULL);

    // omxh264enc: copy *exactly* your knobs
    g_object_set(enc,
        "num-slices", 8,
        "periodicity-idr", 240,
        "cpb-size", 500,
        "gdr-mode", 1 /* horizontal (enum value on many omx builds) */,
        "initial-delay", 250,
        "control-rate", 2 /* low-latency */,
        "prefetch-buffer", TRUE,
        "target-bitrate", bitrate_bps,
        "gop-mode", 2 /* low-delay-p on many omx builds */,
        NULL);

    // h264 caps alignment=nal (as per your line)
    GstCaps *c264 = gst_caps_from_string("video/x-h264, alignment=nal");
    g_object_set(cap_h264, "caps", c264, NULL);
    gst_caps_unref(c264);

    // rtph264pay: defaults are okay (same as your gst-launch)
    // udpsink: mirror your socket/buffering/QoS params
    g_object_set(udp,
        "host", "192.168.25.69",
        "port", 5004,
        "buffer-size", 60000000,
        "async", FALSE,
        "sync", FALSE,
        "qos-dscp", 60,
        NULL);

    // Assemble (capture → appsink) and (appsrc → enc → pay → udp)
    gst_bin_add_many(GST_BIN(pipeline),
        v4l2src, cap_raw, appsink,
        appsrc, q_after, enc, cap_h264, pay, udp, NULL);

    if (!gst_element_link_many(v4l2src, cap_raw, appsink, NULL)) {
        g_printerr("Failed to link capture chain\n"); return 1;
    }
    if (!gst_element_link_many(appsrc, q_after, enc, cap_h264, pay, udp, NULL)) {
        g_printerr("Failed to link encode chain\n"); return 1;
    }

    // Wire callback
    Bridge bridge{appsrc, FALSE};
    GstAppSinkCallbacks cbs{}; cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &cbs, &bridge, nullptr);

    // Bus watch & go
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, bus_cb, nullptr);

    // (Optional) pre-set appsrc caps to match capture; exact caps will be applied on first sample anyway
    {
        GstCaps *pre = gst_caps_new_simple("video/x-raw",
            "format",    G_TYPE_STRING, "NV12",
            "width",     G_TYPE_INT,     1920,
            "height",    G_TYPE_INT,     1080,
            "framerate", GST_TYPE_FRACTION, 60, 1, NULL);
        g_object_set(appsrc, "caps", pre, NULL);
        gst_caps_unref(pre);
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("Running… 1920x1080@60 NV12 -> omxh264enc (low-delay-p) -> RTP/UDP\n");
    loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    gst_element_send_event(pipeline, gst_event_new_eos());
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    return 0;
}
