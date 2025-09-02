// relay_debug_nv12_main_thread_opencv.cpp
// Main thread processing with OpenCV histogram equalization.
//
// Build:
// g++ -O3 -DNDEBUG -std=c++17 relay_debug_nv12_main_thread_opencv.cpp -o relay_debug_main_thread_opencv \
//   $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 opencv4)

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>  // needed for high_resolution_clock timing

struct Counters {
    // Camera queue (q_cam)
    std::atomic<uint64_t> cam_out_frames{0},     cam_out_bytes{0};
    std::atomic<uint64_t> qcam_out_frames{0},    qcam_out_bytes{0};

    // Appsink pad
    std::atomic<uint64_t> appsink_in_frames{0},  appsink_in_bytes{0};

    // Main thread processed / pushed
    std::atomic<uint64_t> processed_frames{0},   processed_bytes{0};
    std::atomic<uint64_t> after_src_frames{0},   after_src_bytes{0};
    std::atomic<uint64_t> encoder_in_frames{0},  encoder_in_bytes{0};

    std::atomic<uint64_t> push_failures{0};
    std::atomic<uint64_t> processing_errors{0};

    // OpenCV processing timing
    std::atomic<uint64_t> total_processing_time_us{0};
};

struct CustomData {
    GstElement  *appsrc{nullptr};
    GstElement  *appsink{nullptr};
    gboolean     video_info_valid{FALSE};
    GstVideoInfo video_info{};

    Counters     ctr{};
    GMainLoop   *loop{nullptr};
};

/* ---------- Pad probes ---------- */

static GstPadProbeReturn probe_cam_out(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.cam_out_frames.fetch_add(1, std::memory_order_relaxed);
        d->ctr.cam_out_bytes .fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_qcam_out(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.qcam_out_frames.fetch_add(1, std::memory_order_relaxed);
        d->ctr.qcam_out_bytes .fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_apps_sink(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.appsink_in_frames.fetch_add(1, std::memory_order_relaxed);
        d->ctr.appsink_in_bytes .fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_after_appsrc_queue(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.after_src_frames.fetch_add(1, std::memory_order_relaxed);
        d->ctr.after_src_bytes .fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_encoder_sink(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
        d->ctr.encoder_in_frames.fetch_add(1, std::memory_order_relaxed);
        d->ctr.encoder_in_bytes .fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

/* ---------- appsink callback: Direct processing in main thread ---------- */

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    auto *d = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;
    
    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    if (!inbuf) { 
        gst_sample_unref(sample); 
        return GST_FLOW_ERROR; 
    }

    // Cache caps once (diagnostic only)
    if (!d->video_info_valid) {
        if (GstCaps *caps = gst_sample_get_caps(sample)) {
            if (gst_video_info_from_caps(&d->video_info, caps)) {
                d->video_info_valid = TRUE;
                g_print("Video info: %dx%d\n", d->video_info.width, d->video_info.height);
            }
        }
    }

    GstFlowReturn result = GST_FLOW_OK;

    try {
        // Map input buffer for reading
        GstMapInfo map_info;
        if (!gst_buffer_map(inbuf, &map_info, GST_MAP_READ)) {
            gst_sample_unref(sample);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return GST_FLOW_ERROR;
        }

        if (!d->video_info_valid) {
            gst_buffer_unmap(inbuf, &map_info);
            gst_sample_unref(sample);
            return GST_FLOW_OK; // Skip this frame, wait for video info
        }

        int width = d->video_info.width;
        int height = d->video_info.height;
        size_t y_size = (size_t)width * (size_t)height;
        size_t uv_size = (size_t)width * (size_t)height / 2;

        if (map_info.size < y_size + uv_size) {
            gst_buffer_unmap(inbuf, &map_info);
            gst_sample_unref(sample);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return GST_FLOW_ERROR;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        // Extract Y plane from NV12 and apply histogram equalization (using clone as requested)
        cv::Mat nv12_input(height * 3 / 2, width, CV_8UC1, map_info.data);
        cv::Mat y_plane_in = nv12_input(cv::Rect(0, 0, width, height)).clone();
        cv::Mat y_plane_out(height, width, CV_8UC1);

        // Histogram Equalization on Y channel
        cv::equalizeHist(y_plane_in, y_plane_out);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        d->ctr.total_processing_time_us.fetch_add(duration.count(), std::memory_order_relaxed);

        // Create output buffer
        GstBuffer *outbuf = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
        if (!outbuf) {
            gst_buffer_unmap(inbuf, &map_info);
            gst_sample_unref(sample);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return GST_FLOW_ERROR;
        }

        // Map output buffer and reconstruct NV12
        GstMapInfo out_map_info;
        if (gst_buffer_map(outbuf, &out_map_info, GST_MAP_WRITE)) {
            // Copy processed Y plane
            memcpy(out_map_info.data, y_plane_out.data, y_size);
            // Fill UV with neutral value 128
            memset(out_map_info.data + y_size, 128, uv_size);
            gst_buffer_unmap(outbuf, &out_map_info);
        } else {
            gst_buffer_unref(outbuf);
            gst_buffer_unmap(inbuf, &map_info);
            gst_sample_unref(sample);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            return GST_FLOW_ERROR;
        }

        gst_buffer_unmap(inbuf, &map_info);

        // Fresh timestamps in appsrc pipeline
        GST_BUFFER_PTS(outbuf)      = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DTS(outbuf)      = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DURATION(outbuf) = GST_CLOCK_TIME_NONE;

        d->ctr.processed_frames.fetch_add(1, std::memory_order_relaxed);
        d->ctr.processed_bytes .fetch_add(gst_buffer_get_size(outbuf), std::memory_order_relaxed);

        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), outbuf);
        if (ret != GST_FLOW_OK) {
            d->ctr.push_failures.fetch_add(1, std::memory_order_relaxed);
            result = ret; // Return the actual error from appsrc
        }

    } catch (const std::exception& e) {
        d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
        g_printerr("OpenCV error: %s\n", e.what());
        result = GST_FLOW_ERROR;
    }

    gst_sample_unref(sample);
    return result;
}

/* ---------- periodic status ---------- */

static gboolean status_tick(gpointer user_data) {
    auto *d = (CustomData*)user_data;

    const uint64_t cam_in   = d->ctr.cam_out_frames.load();
    const uint64_t qcam_out = d->ctr.qcam_out_frames.load();
    const uint64_t apps_in  = d->ctr.appsink_in_frames.load();
    const uint64_t proc     = d->ctr.processed_frames.load();
    const uint64_t aft      = d->ctr.after_src_frames.load();
    const uint64_t enc      = d->ctr.encoder_in_frames.load();

    const uint64_t cam_in_b   = d->ctr.cam_out_bytes.load();
    const uint64_t qcam_out_b = d->ctr.qcam_out_bytes.load();
    const uint64_t apps_in_b  = d->ctr.appsink_in_bytes.load();
    const uint64_t proc_b     = d->ctr.processed_bytes.load();
    const uint64_t aft_b      = d->ctr.after_src_bytes.load();
    const uint64_t enc_b      = d->ctr.encoder_in_bytes.load();

    const uint64_t proc_errors = d->ctr.processing_errors.load();
    const uint64_t total_proc_time = d->ctr.total_processing_time_us.load();

    double avg_proc_time_ms = 0.0;
    if (proc > 0) {
        avg_proc_time_ms = (double)total_proc_time / (double)proc / 1000.0; // convert µs to ms
    }

    g_print(
        "\n=== DEBUG STATUS (every 2s) ===  (push_fail=%" G_GUINT64_FORMAT ", proc_err=%" G_GUINT64_FORMAT ")\n"
        "cam -> q_cam.sink:   %8" G_GUINT64_FORMAT " frames, %8.2f MB\n"
        "q_cam.src -> appsink:%8" G_GUINT64_FORMAT " frames, %8.2f MB\n"
        "appsink in:          %8" G_GUINT64_FORMAT " frames, %8.2f MB\n"
        "processed/pushed:    %8" G_GUINT64_FORMAT " frames, %8.2f MB (avg proc: %.2f ms)\n"
        "after appsrc queue:  %8" G_GUINT64_FORMAT " frames, %8.2f MB\n"
        "encoder sink:        %8" G_GUINT64_FORMAT " frames, %8.2f MB\n",
        d->ctr.push_failures.load(), proc_errors,
        cam_in,   cam_in_b /(1024.0*1024.0),
        qcam_out, qcam_out_b/(1024.0*1024.0),
        apps_in,  apps_in_b /(1024.0*1024.0),
        proc,     proc_b    /(1024.0*1024.0), avg_proc_time_ms,
        aft,      aft_b     /(1024.0*1024.0),
        enc,      enc_b     /(1024.0*1024.0)
    );

    if (cam_in != qcam_out) g_print("LOSS in q_cam (queue dropping):           %" G_GUINT64_FORMAT "\n", cam_in - qcam_out);
    if (qcam_out != apps_in) g_print("LOSS between q_cam.src and appsink.sink:  %" G_GUINT64_FORMAT "\n", qcam_out - apps_in);
    if (apps_in != proc)     g_print("LOSS between appsink.sink and process:    %" G_GUINT64_FORMAT "\n", apps_in - proc);
    if (proc != aft)         g_print("LOSS between appsrc push and queue out:   %" G_GUINT64_FORMAT "\n", proc - aft);
    if (aft != enc)          g_print("LOSS between queue out and encoder sink:  %" G_GUINT64_FORMAT "\n", aft - enc);

    return TRUE;
}

/* ---------- bus watch (quit main loop) ---------- */

static gboolean bus_cb(GstBus *bus, GstMessage *msg, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *e=NULL; gchar *dbg=NULL;
            gst_message_parse_error(msg, &e, &dbg);
            g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), e->message);
            g_error_free(e); g_free(dbg);
            if (d->loop) g_main_loop_quit(d->loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("EOS from %s\n", GST_OBJECT_NAME(msg->src));
            if (d->loop) g_main_loop_quit(d->loop);
            break;
        default: break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);
    gst_init(&argc, &argv);

    gboolean use_h265 = FALSE;
    int bitrate_kbps = 20000; // Match basic.cpp default (20 Mbps)
    int v_width = 1920, v_height = 1080, fps = 60; // defaults

    // --- extend argv parsing with width/height/fps ---
    for (int i=1;i<argc;++i){
        if (g_str_has_prefix(argv[i],"--codec=")) { const char* v=strchr(argv[i],'='); if(v&&g_ascii_strcasecmp(v+1,"h265")==0) use_h265=TRUE; }
        else if (g_strcmp0(argv[i],"--codec")==0 && i+1<argc){ if (g_ascii_strcasecmp(argv[i+1],"h265")==0) use_h265=TRUE; }
        else if (g_str_has_prefix(argv[i],"--bitrate=")) { const char* v=strchr(argv[i],'='); if(v){ int b=atoi(v+1); if(b>0) bitrate_kbps=b; } }
        else if (g_strcmp0(argv[i],"--bitrate")==0 && i+1<argc){ int b=atoi(argv[i+1]); if(b>0) bitrate_kbps=b; }
        else if (g_str_has_prefix(argv[i],"--width=")) { const char* v=strchr(argv[i],'='); if(v){ int w=atoi(v+1); if(w>0) v_width=w; } }
        else if (g_strcmp0(argv[i],"--width")==0 && i+1<argc){ int w=atoi(argv[i+1]); if(w>0) v_width=w; }
        else if (g_str_has_prefix(argv[i],"--height=")) { const char* v=strchr(argv[i],'='); if(v){ int h=atoi(v+1); if(h>0) v_height=h; } }
        else if (g_strcmp0(argv[i],"--height")==0 && i+1<argc){ int h=atoi(argv[i+1]); if(h>0) v_height=h; }
        else if (g_str_has_prefix(argv[i],"--fps=")) { const char* v=strchr(argv[i],'='); if(v){ int f=atoi(v+1); if(f>0) fps=f; } }
        else if (g_strcmp0(argv[i],"--fps")==0 && i+1<argc){ int f=atoi(argv[i+1]); if(f>0) fps=f; }
    }
    g_print("Encoder: %s, target-bitrate: %d kbps, %dx%d@%dfps (Main Thread Processing)\n",
            use_h265 ? "H.265" : "H.264", bitrate_kbps, v_width, v_height, fps);

    CustomData d{};

    // Capture pipeline (bump queue to 8 for smoothing; add videorate dropper)
    GError *err=NULL;
    gchar *sink_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=60/1 ! "
        "videorate drop-only=true max-rate=%d ! "
        "queue name=q_cam leaky=downstream max-size-buffers=8 max-size-time=0 max-size-bytes=0 ! "
        "appsink name=cv_sink emit-signals=true max-buffers=1 drop=true sync=false",
        v_width, v_height, fps
    );
    GstElement *sink_pipe = gst_parse_launch(sink_str, &err);
    g_free(sink_str);
    if (!sink_pipe) { g_printerr("Create sink pipeline failed: %s\n", err?err->message:"?"); g_clear_error(&err); return -1; }
    d.appsink = gst_bin_get_by_name(GST_BIN(sink_pipe), "cv_sink");
    if (!d.appsink) { g_printerr("Failed to find appsink 'cv_sink'\n"); gst_object_unref(sink_pipe); return -1; }

    // Streaming pipeline (dynamic caps derived from CLI)
    gchar *src_str=NULL;
    if (use_h265) {
        src_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "queue name=q_after_src leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 ! "
            "omxh265enc name=enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h265,alignment=au ! "
            "rtph265pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            v_width, v_height, fps, bitrate_kbps
        );
    } else {
        src_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "queue name=q_after_src leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 ! "
            "omxh264enc name=enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h264,alignment=nal ! "
            "rtph264pay ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60",
            v_width, v_height, fps, bitrate_kbps
        );
    }
    GstElement *src_pipe = gst_parse_launch(src_str, &err);
    g_free(src_str);
    if (!src_pipe) {
        g_printerr("Create src pipeline failed: %s\n", err?err->message:"?");
        g_clear_error(&err);
        gst_object_unref(sink_pipe);
        return -1;
    }
    d.appsrc = gst_bin_get_by_name(GST_BIN(src_pipe), "my_src");
    if (!d.appsrc) {
        g_printerr("Failed to find appsrc 'my_src'\n");
        gst_object_unref(src_pipe);
        gst_object_unref(sink_pipe);
        return -1;
    }

    // Probes: q_cam.sink, q_cam.src, appsink.sink, q_after_src.src, enc.sink
    {
        GstElement *q_cam = gst_bin_get_by_name(GST_BIN(sink_pipe), "q_cam");
        if (q_cam) {
            if (GstPad *p = gst_element_get_static_pad(q_cam, "sink")) { gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_cam_out, &d, NULL); gst_object_unref(p); }
            if (GstPad *p = gst_element_get_static_pad(q_cam, "src"))  { gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_qcam_out, &d, NULL); gst_object_unref(p); }
            gst_object_unref(q_cam);
        }
    }
    { if (GstPad *p = gst_element_get_static_pad(d.appsink, "sink")) { gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_apps_sink, &d, NULL); gst_object_unref(p); } }
    {
        GstElement *q = gst_bin_get_by_name(GST_BIN(src_pipe), "q_after_src");
        if (q) { if (GstPad *p = gst_element_get_static_pad(q, "src")) { gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_after_appsrc_queue, &d, NULL); gst_object_unref(p); } gst_object_unref(q); }
    }
    {
        GstElement *enc = gst_bin_get_by_name(GST_BIN(src_pipe), "enc");
        if (enc) { if (GstPad *p = gst_element_get_static_pad(enc, "sink")) { gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_encoder_sink, &d, NULL); gst_object_unref(p); } gst_object_unref(enc); }
    }

    // Set up appsink callback - now processes directly in main thread
    g_signal_connect(d.appsink, "new-sample", G_CALLBACK(new_sample_cb), &d);

    // GLib loop + watches + status timer
    d.loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus_sink = gst_element_get_bus(sink_pipe);
    GstBus *bus_src  = gst_element_get_bus(src_pipe);
    gst_bus_add_watch(bus_sink, bus_cb, &d);
    gst_bus_add_watch(bus_src,  bus_cb, &d);
    g_timeout_add_seconds(2, status_tick, &d);

    // Start & run
    gst_element_set_state(src_pipe,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipe, GST_STATE_PLAYING);
    g_print("Single-threaded OpenCV histogram equalization. Press Ctrl+C to exit.\n");
    g_main_loop_run(d.loop);

    // Shutdown - much simpler now!
    gst_element_set_state(sink_pipe, GST_STATE_NULL);
    gst_element_set_state(src_pipe,  GST_STATE_NULL);
    gst_object_unref(bus_sink);
    gst_object_unref(bus_src);
    gst_object_unref(d.appsink);
    gst_object_unref(d.appsrc);
    gst_object_unref(sink_pipe);
    gst_object_unref(src_pipe);
    g_main_loop_unref(d.loop);
    return 0;
}