#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    GstElement  *appsrc;
    GstElement  *appsink;
    gboolean     video_info_valid;
    GstVideoInfo video_info;
    guint64      frame_idx;     // <— add
    GstClockTime base_pts;      // <— add
} CustomData;

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *d = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *in = gst_sample_get_buffer(sample);
    if (!in) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    if (!d->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&d->video_info, caps)) {
            d->video_info_valid = TRUE;
            d->frame_idx = 0;
            d->base_pts  = 0;

            // propagate exact caps to appsrc
            gst_app_src_set_caps(GST_APP_SRC(d->appsrc), caps);

            // allow ~4 frames in flight
            guint64 budget = (d->video_info.size ? d->video_info.size : (4*1024*1024)) * 4ull;
            g_object_set(d->appsrc, "max-bytes", budget, NULL);
        } else {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    // Zero-copy pass-through: add a ref for appsrc
    GstBuffer *out = gst_buffer_ref(in);

    // Exact 60fps pacing from frame counter
    guint fps_n = d->video_info.fps_n ? d->video_info.fps_n : 60;
    guint fps_d = d->video_info.fps_d ? d->video_info.fps_d : 1;
    GstClockTime dur = gst_util_uint64_scale_int(GST_SECOND, fps_d, fps_n);

    if (d->frame_idx == 0) d->base_pts = 0;  // start at 0 for a clean timeline

    GstClockTime pts = d->base_pts + d->frame_idx * dur;

    GST_BUFFER_PTS(out) = pts;
    GST_BUFFER_DTS(out) = pts;
    GST_BUFFER_DURATION(out) = dur;

    d->frame_idx++;

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), out);
    gst_sample_unref(sample);
    return ret;
}

static void on_bus_msg(GstBus *bus, GstMessage *msg, gpointer user_data) {
    (void)bus; (void)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
            if (dbg) g_printerr("Debug: %s\n", dbg);
            g_clear_error(&err); g_free(dbg);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("EOS from %s\n", GST_OBJECT_NAME(msg->src));
            break;
        default: break;
    }
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // Default bitrate (kbps) OK
    gint target_bitrate = 8000;
    for (int i = 1; i < argc; i++) {
        if (g_str_has_prefix(argv[i], "--bitrate=")) {
            const gchar *value = argv[i] + strlen("--bitrate=");
            target_bitrate = atoi(value);
            if (target_bitrate <= 0) {
                g_printerr("Invalid bitrate value: %s\n", value);
                return -1;
            }
        }
    }
    g_print("Using target bitrate: %d kbps\n", target_bitrate);

    CustomData data = {};
    data.video_info_valid = FALSE;
    data.t0 = GST_CLOCK_TIME_NONE;

    GError *error = NULL;

    // Ingest pipeline: camera → appsink (keep 4K60 as you had it)
const gchar *sink_pipeline_str =
  "v4l2src device=/dev/video0 io-mode=4 ! "
  "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
  "videorate drop-only=true max-rate=60"
  "appsink name=my_sink emit-signals=true max-buffers=4 drop=false sync=true";


    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");

    // Egress pipeline: appsrc → encoder → pay → UDP
    // NOTE: Keep your encoder knobs. Consider AU alignment for low-latency receivers (see commented line).
    gchar *src_pipeline_str = g_strdup_printf(
    "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=false stream-type=0 ! "
    "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
    "queue max-size-buffers=8 max-size-time=0 max-size-bytes=0 ! "
    "omxh264enc control-rate=constant target-bitrate=%d "
    "gop-mode=low-delay-p periodicity-idr=60 gop-length=60 "
    "num-slices=4 insert-sps-pps=true filler-data=true prefetch-buffer=true ! "
    "video/x-h264,alignment=au,profile=main,stream-format=byte-stream ! "
    "rtph264pay config-interval=1 pt=96 ! "
    "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
    target_bitrate);


    // pipeline_str = g_strdup_printf(
    //     "appsrc name=cv_src format=GST_FORMAT_TIME ! "
    //     "video/x-raw, format=NV12, width=1920, height=1080, framerate=%d/1 ! "
    //     "queue ! "
    //     "omxh264enc skip-frame=true max-consecutive-skip=5 gop-mode=low-delay-p target-bitrate=%d num-slices=8 control-rate=Constant qp-mode=auto "
    //     "prefetch-buffer=true cpb-size=200 initial-delay=200 gdr-mode=horizontal "
    //     "periodicity-idr=30 gop-length=30 filler-data=true ! "
    //     "video/x-h264, alignment=au, profile=main, stream-format=byte-stream ! "
    //     "rtph264pay mtu=1400 ! "
    //     "queue max-size-buffers=2 ! "
    //     "udpsink clients=192.168.25.69:5004 auto-multicast=false",
    //     fps, bitrate);


    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error->message);
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return -1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");

    // Hook callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Watch both buses
    GstBus *bus_sink = gst_element_get_bus(sink_pipeline);
    GstBus *bus_src  = gst_element_get_bus(src_pipeline);
    gst_bus_add_signal_watch(bus_sink);
    gst_bus_add_signal_watch(bus_src);
    g_signal_connect(bus_sink, "message", G_CALLBACK(on_bus_msg), NULL);
    g_signal_connect(bus_src,  "message", G_CALLBACK(on_bus_msg), NULL);

    // Start pipelines
    gst_element_set_state(src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);
    g_print("Running. Press Ctrl+C to exit.\n");

    // Main loop
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);

    // Cleanup
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline,  GST_STATE_NULL);
    if (bus_sink) gst_object_unref(bus_sink);
    if (bus_src)  gst_object_unref(bus_src);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    return 0;
}
