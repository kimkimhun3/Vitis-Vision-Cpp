// histequalize_host_opencv_nv12.cpp
// OpenCV global histogram equalization on NV12 (Y-only) with GStreamer appsink/appsrc
// - Stride-correct using GstVideoFrame
// - Zero-clone, minimal copies
// - v4l2src io-mode=2 (mmap) for reliable CPU mapping

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <opencv2/opencv.hpp>
#include <string.h>  // argv parsing
#include <stdlib.h>  // atoi

// ---- Tunables --------------------------------------------------------------
#define PRESERVE_COLOR_UV 1   // 1: copy UV (keep color), 0: set UV=128 (gray)
#define MAP_Y_TO_LIMITED  0   // 1: map Y 0..255 -> 16..235 (video/limited range)

// ---- State -----------------------------------------------------------------
typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean    video_info_valid;
    GstVideoInfo video_info;
    GTimer     *processing_timer;
    double      total_processing_time;
    int         frame_count;
} CustomData;

// Optional LUT for 0..255 -> 16..235 mapping
static cv::Mat make_limited_range_lut() {
    static cv::Mat lut;
    if (lut.empty()) {
        lut.create(1, 256, CV_8U);
        for (int i = 0; i < 256; ++i) {
            lut.ptr<uchar>(0)[i] = (uchar)(16 + (219 * i + 127) / 255);
        }
    }
    return lut;
}

// ---- Callback: stride-correct, zero-clone ----------------------------------
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("appsink: null sample\n");
        return GST_FLOW_ERROR;
    }
    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    if (!inbuf) {
        g_printerr("appsink: null buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Cache GstVideoInfo once
    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (!caps || !gst_video_info_from_caps(&data->video_info, caps)) {
            g_printerr("appsink: failed to get video info from caps\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
        data->video_info_valid = TRUE;
        g_print("Video info: %dx%d NV12\n", data->video_info.width, data->video_info.height);
    }

    // Map input as GstVideoFrame (stride-correct). READ only.
    GstVideoFrame vf_in;
    if (!gst_video_frame_map(&vf_in, &data->video_info, inbuf, GST_MAP_READ)) {
        g_printerr("appsink: gst_video_frame_map READ failed\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    const int   W         = GST_VIDEO_INFO_WIDTH(&data->video_info);
    const int   H         = GST_VIDEO_INFO_HEIGHT(&data->video_info);
    const gsize y_size    = (gsize)W * H;
    const gsize uv_size   = (gsize)W * H / 2;
    const gsize out_bytes = y_size + uv_size;

    // Allocate destination buffer (tight NV12: Y stride=W, UV stride=W)
    GstBuffer *outbuf = gst_buffer_new_allocate(NULL, out_bytes, NULL);
    if (!outbuf) {
        g_printerr("appsrc: allocate out buffer failed\n");
        gst_video_frame_unmap(&vf_in);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstMapInfo outmap;
    if (!gst_buffer_map(outbuf, &outmap, GST_MAP_WRITE)) {
        g_printerr("appsrc: map out buffer failed\n");
        gst_buffer_unref(outbuf);
        gst_video_frame_unmap(&vf_in);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Plane pointers & strides from input
    const uint8_t *y_in      = GST_VIDEO_FRAME_PLANE_DATA(&vf_in, 0);
    const int      y_stride  = GST_VIDEO_FRAME_PLANE_STRIDE(&vf_in, 0);
    const uint8_t *uv_in     = GST_VIDEO_FRAME_PLANE_DATA(&vf_in, 1);
    const int      uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vf_in, 1);

    uint8_t *dst_y  = outmap.data;
    uint8_t *dst_uv = outmap.data + y_size;

    // Timing: include map -> equalize -> pack
    g_timer_start(data->processing_timer);

    // Wrap input Y with stride, and destination Y with tight stride=W
    cv::Mat srcY(H, W, CV_8UC1, const_cast<uint8_t*>(y_in), y_stride);
    cv::Mat dstY(H, W, CV_8UC1, dst_y, W);

    // Equalize directly into destination (no clone, no temp)
    cv::equalizeHist(srcY, dstY);

#if MAP_Y_TO_LIMITED
    // Optional: map 0..255 -> 16..235 if encoder expects limited/TV range
    cv::LUT(dstY, make_limited_range_lut(), dstY);
#endif

#if PRESERVE_COLOR_UV
    // Keep color: copy UV row-by-row (input stride -> tight W)
    for (int r = 0; r < H / 2; ++r) {
        memcpy(dst_uv + r * W, uv_in + r * uv_stride, (size_t)W);
    }
#else
    // Grayscale output: neutral chroma
    memset(dst_uv, 128, uv_size);
#endif

    g_timer_stop(data->processing_timer);
    const double ms = g_timer_elapsed(data->processing_timer, NULL) * 1000.0;
    data->total_processing_time += ms;
    data->frame_count++;
    if ((data->frame_count % 100) == 0) {
        const double avg = data->total_processing_time / data->frame_count;
        g_print("[PROC] frame=%d time=%.2f ms avg=%.2f ms fps≈%.1f\n",
                data->frame_count, ms, avg, 1000.0 / avg);
    }

    // Done writing out buffer
    gst_buffer_unmap(outbuf, &outmap);

    // Preserve timestamps from input
    gst_buffer_copy_into(outbuf, inbuf, GST_BUFFER_COPY_TIMESTAMPS, 0, (gssize)-1);

    // Push downstream (appsrc takes ownership on success)
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), outbuf);
    if (ret != GST_FLOW_OK) {
        g_printerr("appsrc: push failed (%d)\n", ret);
        gst_buffer_unref(outbuf); // not consumed
        gst_video_frame_unmap(&vf_in);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Cleanup input map/sample
    gst_video_frame_unmap(&vf_in);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// ---- Main ------------------------------------------------------------------
int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // Parse only --codec and --bitrate (kbps)
    gboolean use_h265 = FALSE;
    int bitrate_kbps = 20000; // default 20 Mbps

    for (int i = 1; i < argc; ++i) {
        if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *val = strchr(argv[i], '=');
            if (val && g_ascii_strcasecmp(val + 1, "h265") == 0) use_h265 = TRUE;
        } else if (g_strcmp0(argv[i], "--codec") == 0 && i + 1 < argc) {
            if (g_ascii_strcasecmp(argv[i + 1], "h265") == 0) use_h265 = TRUE;
        } else if (g_str_has_prefix(argv[i], "--bitrate=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                int v = atoi(val + 1);
                if (v > 0) bitrate_kbps = v;
            }
        } else if (g_strcmp0(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            int v = atoi(argv[i + 1]);
            if (v > 0) bitrate_kbps = v;
        }
    }

    g_print("Encoder: %s, target-bitrate: %d kbps\n",
            use_h265 ? "H.265" : "H.264", bitrate_kbps);

    CustomData data = {};
    data.video_info_valid = FALSE;
    data.processing_timer = g_timer_new();
    data.total_processing_time = 0.0;
    data.frame_count = 0;

    GError *error = NULL;

    // 1) Capture pipeline (appsink) — use io-mode=2 (mmap) for CPU processing
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=2 ! " // <-- mmap
        "video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1 ! "
        "appsink name=my_sink emit-signals=true max-buffers=2 drop=true sync=false"
    );
    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error ? error->message : "(unknown)");
        g_clear_error(&error);
        return -1;
    }
    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");

    // 2) Streaming pipeline (appsrc → encoder → pay → udpsink)
    gchar *src_pipeline_str = NULL;
    if (use_h265) {
        // H.265 path (low-latency knobs; you can tweak to match your board)
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
            "omxh265enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h265, alignment=au ! "
            "rtph265pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            bitrate_kbps
        );
    } else {
        // H.264 path
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
            "omxh264enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h264, alignment=nal ! "
            "rtph264pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            bitrate_kbps
        );
    }

    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error ? error->message : "(unknown)");
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return -1;
    }
    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");

    // Connect callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Start pipelines
    gst_element_set_state(src_pipeline,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    g_print("Running. Press Ctrl+C to exit.\n");

    // Minimal bus loop (capture pipeline). You can add a second loop for src_pipeline if desired.
    GstBus *bus = gst_element_get_bus(sink_pipeline);
    gboolean running = TRUE;
    while (running) {
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus, GST_CLOCK_TIME_NONE,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg) {
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = NULL;
                gchar *debug_info = NULL;
                gst_message_parse_error(msg, &err, &debug_info);
                g_printerr("Error: %s\n", err->message);
                g_error_free(err);
                g_free(debug_info);
                running = FALSE;
                break;
            }
            case GST_MESSAGE_EOS:
                g_print("End-Of-Stream reached.\n");
                running = FALSE;
                break;
            default:
                break;
            }
            gst_message_unref(msg);
        }
    }

    // Cleanup
    gst_object_unref(bus);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline,  GST_STATE_NULL);
    gst_object_unref(data.appsink);
    gst_object_unref(data.appsrc);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    g_timer_destroy(data.processing_timer);
    return 0;
}
