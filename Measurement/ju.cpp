// relay_debug_nv12_worker_opencv.cpp
// Worker-based processing with OpenCV histogram equalization + frame rate monitoring.
//
// Build:
// g++ -O3 -DNDEBUG -std=c++17 relay_debug_nv12_worker_opencv.cpp -o relay_debug_worker_opencv \
//   $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 opencv4) -lpthread

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
#include <chrono>

struct FrameRateCounters {
    // Frame counters for rate calculation (reset every 2 seconds)
    std::atomic<uint64_t> camera_frames{0};          // Frames captured from camera
    std::atomic<uint64_t> opencv_input_frames{0};    // Frames sent to OpenCV processing  
    std::atomic<uint64_t> opencv_output_frames{0};   // Frames processed by OpenCV
    std::atomic<uint64_t> encoder_frames{0};         // Frames sent to encoder

    // For error tracking
    std::atomic<uint64_t> processing_errors{0};
    std::atomic<uint64_t> push_failures{0};
};

struct CustomData {
    GstElement  *appsrc{nullptr};
    GstElement  *appsink{nullptr};
    gboolean     video_info_valid{FALSE};
    GstVideoInfo video_info{};

    // Worker decoupling
    GAsyncQueue *work_q{nullptr};   // carries GstBuffer* from callback to worker
    GThread     *worker{nullptr};
    std::atomic<bool> stop{false};

    // Multi-threading support
    int num_workers{1};
    GThread     **workers{nullptr};

    FrameRateCounters ctr{};
    GMainLoop   *loop{nullptr};
};

/* ---------- Pad probes for frame rate counting ---------- */

static GstPadProbeReturn probe_camera_output(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        d->ctr.camera_frames.fetch_add(1, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn probe_encoder_input(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto *d = (CustomData*)user_data;
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) && GST_PAD_PROBE_INFO_BUFFER(info)) {
        d->ctr.encoder_frames.fetch_add(1, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

/* ---------- appsink callback: O(1) enqueue ---------- */

static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    auto *d = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;
    GstBuffer *inbuf = gst_sample_get_buffer(sample);
    if (!inbuf) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

    // Cache caps once (diagnostic only)
    if (!d->video_info_valid) {
        if (GstCaps *caps = gst_sample_get_caps(sample)) {
            if (gst_video_info_from_caps(&d->video_info, caps)) {
                d->video_info_valid = TRUE;
                g_print("Video info: %dx%d\n", d->video_info.width, d->video_info.height);
            }
        }
    }

    // Count frame sent to OpenCV processing
    d->ctr.opencv_input_frames.fetch_add(1, std::memory_order_relaxed);

    // O(1): ref buffer, queue to worker, unref sample (returns one ref to pool)
    gst_buffer_ref(inbuf);
    g_async_queue_push(d->work_q, inbuf);

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

/* ---------- worker thread: OpenCV histogram equalization + push ---------- */

static gpointer worker_thread_fn(gpointer user_data) {
    auto *d = (CustomData*)user_data;

    while (!d->stop.load(std::memory_order_acquire)) {
        // Pop with timeout to allow graceful exit
        gpointer item = g_async_queue_timeout_pop(d->work_q, 50 * G_TIME_SPAN_MILLISECOND);
        if (!item) continue;

        GstBuffer *inbuf = (GstBuffer*)item;

        try {
            // Map input buffer for reading
            GstMapInfo map_info;
            if (!gst_buffer_map(inbuf, &map_info, GST_MAP_READ)) {
                gst_buffer_unref(inbuf);
                d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            if (!d->video_info_valid) {
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                continue;
            }

            int width = d->video_info.width;
            int height = d->video_info.height;
            size_t y_size = (size_t)width * (size_t)height;
            size_t uv_size = (size_t)width * (size_t)height / 2;

            if (map_info.size < y_size + uv_size) {
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            // Extract Y plane from NV12 and apply histogram equalization (using clone as requested)
            cv::Mat nv12_input(height * 3 / 2, width, CV_8UC1, map_info.data);
            cv::Mat y_plane_in = nv12_input(cv::Rect(0, 0, width, height)).clone();
            cv::Mat y_plane_out(height, width, CV_8UC1);

            // Histogram Equalization on Y channel
            cv::equalizeHist(y_plane_in, y_plane_out);

            // Create output buffer
            GstBuffer *outbuf = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
            if (!outbuf) {
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            // Map output buffer and reconstruct NV12
            GstMapInfo out_map_info;
            if (gst_buffer_map(outbuf, &out_map_info, GST_MAP_WRITE)) {
                // Copy processed Y plane
                memcpy(out_map_info.data, y_plane_out.data, y_size);
                // Fill UV with neutral value 128 (like basic.cpp)
                memset(out_map_info.data + y_size, 128, uv_size);
                gst_buffer_unmap(outbuf, &out_map_info);
            } else {
                gst_buffer_unref(outbuf);
                gst_buffer_unmap(inbuf, &map_info);
                gst_buffer_unref(inbuf);
                d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            gst_buffer_unmap(inbuf, &map_info);
            gst_buffer_unref(inbuf); // release camera buffer

            // Fresh timestamps in appsrc pipeline
            GST_BUFFER_PTS(outbuf)      = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DTS(outbuf)      = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DURATION(outbuf) = GST_CLOCK_TIME_NONE;

            // Count frame processed by OpenCV
            d->ctr.opencv_output_frames.fetch_add(1, std::memory_order_relaxed);

            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), outbuf);
            if (ret != GST_FLOW_OK) {
                d->ctr.push_failures.fetch_add(1, std::memory_order_relaxed);
                gst_buffer_unref(outbuf);
            }

        } catch (const std::exception& e) {
            gst_buffer_unref(inbuf);
            d->ctr.processing_errors.fetch_add(1, std::memory_order_relaxed);
            g_printerr("OpenCV error: %s\n", e.what());
        }
    }
    return nullptr;
}

/* ---------- periodic frame rate status ---------- */

static gboolean framerate_status_tick(gpointer user_data) {
    auto *d = (CustomData*)user_data;

    // Get current counts
    uint64_t camera_fps = d->ctr.camera_frames.load();
    uint64_t opencv_input_fps = d->ctr.opencv_input_frames.load();
    uint64_t opencv_output_fps = d->ctr.opencv_output_frames.load();
    uint64_t encoder_fps = d->ctr.encoder_frames.load();
    
    uint64_t processing_errors = d->ctr.processing_errors.load();
    uint64_t push_failures = d->ctr.push_failures.load();
    int queue_length = g_async_queue_length(d->work_q);

    // Reset counters for next interval (divide by 2 since we're measuring over 2 seconds)
    d->ctr.camera_frames.store(0);
    d->ctr.opencv_input_frames.store(0);
    d->ctr.opencv_output_frames.store(0);
    d->ctr.encoder_frames.store(0);

    g_print(
        "\n=== FRAME RATE STATUS (2s interval) ===\n"
        "Camera capture rate:     %.1f fps\n"
        "OpenCV input rate:       %.1f fps\n"
        "OpenCV output rate:      %.1f fps\n"
        "Encoder input rate:      %.1f fps\n"
        "Queue length: %d | Processing errors: %" G_GUINT64_FORMAT " | Push failures: %" G_GUINT64_FORMAT "\n",
        camera_fps / 2.0,
        opencv_input_fps / 2.0,
        opencv_output_fps / 2.0,
        encoder_fps / 2.0,
        queue_length, processing_errors, push_failures
    );

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
    int num_workers = 2; // Default to 2 workers for better performance

    int v_width = 1920, v_height = 1080, fps = 60; // defaults

    // --- extend argv parsing with width/height/fps ---
    for (int i=1;i<argc;++i){
        if (g_str_has_prefix(argv[i],"--codec=")) { const char* v=strchr(argv[i],'='); if(v&&g_ascii_strcasecmp(v+1,"h265")==0) use_h265=TRUE; }
        else if (g_strcmp0(argv[i],"--codec")==0 && i+1<argc){ if (g_ascii_strcasecmp(argv[i+1],"h265")==0) use_h265=TRUE; }
        else if (g_str_has_prefix(argv[i],"--bitrate=")) { const char* v=strchr(argv[i],'='); if(v){ int b=atoi(v+1); if(b>0) bitrate_kbps=b; } }
        else if (g_strcmp0(argv[i],"--bitrate")==0 && i+1<argc){ int b=atoi(argv[i+1]); if(b>0) bitrate_kbps=b; }
        else if (g_str_has_prefix(argv[i],"--workers=")) { const char* v=strchr(argv[i],'='); if(v){ int w=atoi(v+1); if(w>0 && w<=8) num_workers=w; } }
        else if (g_strcmp0(argv[i],"--workers")==0 && i+1<argc){ int w=atoi(argv[i+1]); if(w>0 && w<=8) num_workers=w; }
        else if (g_str_has_prefix(argv[i],"--width=")) { const char* v=strchr(argv[i],'='); if(v){ int w=atoi(v+1); if(w>0) v_width=w; } }
        else if (g_strcmp0(argv[i],"--width")==0 && i+1<argc){ int w=atoi(argv[i+1]); if(w>0) v_width=w; }
        else if (g_str_has_prefix(argv[i],"--height=")) { const char* v=strchr(argv[i],'='); if(v){ int h=atoi(v+1); if(h>0) v_height=h; } }
        else if (g_strcmp0(argv[i],"--height")==0 && i+1<argc){ int h=atoi(argv[i+1]); if(h>0) v_height=h; }
        else if (g_str_has_prefix(argv[i],"--fps=")) { const char* v=strchr(argv[i],'='); if(v){ int f=atoi(v+1); if(f>0) fps=f; } }
        else if (g_strcmp0(argv[i],"--fps")==0 && i+1<argc){ int f=atoi(argv[i+1]); if(f>0) fps=f; }
    }
    g_print("Encoder: %s, target-bitrate: %d kbps, workers: %d, %dx%d@%dfps\n",
            use_h265 ? "H.265" : "H.264", bitrate_kbps, num_workers, v_width, v_height, fps);

    CustomData d{};
    d.work_q = g_async_queue_new();
    d.num_workers = num_workers;

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

    // Add probes for frame rate monitoring
    {
        // Camera output probe (after videorate, before queue)
        GstElement *videorate = gst_bin_get_by_name(GST_BIN(sink_pipe), "videorate0");
        if (!videorate) {
            // Try to get videorate element by iterating through elements
            GstIterator *iter = gst_bin_iterate_elements(GST_BIN(sink_pipe));
            GValue item = G_VALUE_INIT;
            while (gst_iterator_next(iter, &item) == GST_ITERATOR_OK) {
                GstElement *element = GST_ELEMENT(g_value_get_object(&item));
                gchar *name = gst_element_get_name(element);
                if (g_str_has_prefix(name, "videorate")) {
                    videorate = GST_ELEMENT(gst_object_ref(element));
                    g_free(name);
                    g_value_reset(&item);
                    break;
                }
                g_free(name);
                g_value_reset(&item);
            }
            gst_iterator_free(iter);
        }
        
        if (videorate) {
            if (GstPad *p = gst_element_get_static_pad(videorate, "src")) {
                gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_camera_output, &d, NULL);
                gst_object_unref(p);
            }
            gst_object_unref(videorate);
        }
    }
    
    {
        // Encoder input probe
        GstElement *enc = gst_bin_get_by_name(GST_BIN(src_pipe), "enc");
        if (enc) {
            if (GstPad *p = gst_element_get_static_pad(enc, "sink")) {
                gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, probe_encoder_input, &d, NULL);
                gst_object_unref(p);
            }
            gst_object_unref(enc);
        }
    }

    // Callback + workers
    g_signal_connect(d.appsink, "new-sample", G_CALLBACK(new_sample_cb), &d);

    // Start multiple worker threads
    d.workers = g_new(GThread*, d.num_workers);
    for (int i = 0; i < d.num_workers; i++) {
        gchar *thread_name = g_strdup_printf("opencv-worker-%d", i);
        d.workers[i] = g_thread_new(thread_name, worker_thread_fn, &d);
        g_free(thread_name);
    }

    // GLib loop + watches + status timer
    d.loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus_sink = gst_element_get_bus(sink_pipe);
    GstBus *bus_src  = gst_element_get_bus(src_pipe);
    gst_bus_add_watch(bus_sink, bus_cb, &d);
    gst_bus_add_watch(bus_src,  bus_cb, &d);
    g_timeout_add_seconds(2, framerate_status_tick, &d);

    // Start & run
    gst_element_set_state(src_pipe,  GST_STATE_PLAYING);
    gst_element_set_state(sink_pipe, GST_STATE_PLAYING);
    g_print("OpenCV histogram equalization pipeline started. Press Ctrl+C to exit.\n");
    g_main_loop_run(d.loop);

    // Shutdown
    d.stop.store(true, std::memory_order_release);
    // Drain worker queue
    while (GstBuffer *b = (GstBuffer*)g_async_queue_try_pop(d.work_q)) gst_buffer_unref(b);

    // Join all worker threads
    if (d.workers) {
        for (int i = 0; i < d.num_workers; i++) {
            if (d.workers[i]) {
                g_thread_join(d.workers[i]);
            }
        }
        g_free(d.workers);
        d.workers = nullptr;
    }

    if (d.work_q) { g_async_queue_unref(d.work_q); d.work_q = nullptr; }

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