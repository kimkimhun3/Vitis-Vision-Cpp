#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <opencv2/opencv.hpp>
#include <string.h>  // argv parsing
#include <stdlib.h>  // atoi

typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean video_info_valid;
    GstVideoInfo video_info;
    GTimer *processing_timer;
    double total_processing_time;
    int frame_count;
    gchar *input_file;
    gboolean loop_playback;

    // NEW: keep loop and pipelines here so the bus callback can use them safely
    GMainLoop *loop;
    GstElement *sink_pipeline;
    GstElement *src_pipeline;
} CustomData;

// Called when appsink has a new sample
static GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("Failed to pull sample from appsink\n");
        return GST_FLOW_ERROR;
    }

    if (data->frame_count == 0) {
        g_print("First frame received! Processing started.\n");
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("Failed to get buffer from sample\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (!data->video_info_valid) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (caps && gst_video_info_from_caps(&data->video_info, caps)) {
            data->video_info_valid = TRUE;
            g_print("Video info: %dx%d, format: %s\n",
                data->video_info.width, data->video_info.height,
                gst_video_format_to_string(data->video_info.finfo->format));
        } else {
            g_printerr("Failed to extract video info\n");
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }
    }

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
        g_printerr("Failed to map buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    int width = data->video_info.width;
    int height = data->video_info.height;

    size_t y_size = (size_t)width * (size_t)height;
    size_t uv_size = (size_t)width * (size_t)height / 2;
    if (map_info.size < y_size + uv_size) {
        g_printerr("Buffer size mismatch: expected %zu, got %zu\n", y_size + uv_size, map_info.size);
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Step 1: Extract Y plane from NV12
    cv::Mat nv12_input(height * 3 / 2, width, CV_8UC1, map_info.data);
    cv::Mat y_plane_in = nv12_input(cv::Rect(0, 0, width, height)).clone();
    cv::Mat y_plane_out(height, width, CV_8UC1);

    try {
        g_timer_start(data->processing_timer);

        // Histogram Equalization on Y
        cv::equalizeHist(y_plane_in, y_plane_out);

        g_timer_stop(data->processing_timer);
        double frame_processing_time = g_timer_elapsed(data->processing_timer, NULL) * 1000.0;
        data->total_processing_time += frame_processing_time;
        data->frame_count++;

        if (data->frame_count % 100 == 0) {
            double avg = data->total_processing_time / data->frame_count;
            g_print("Stats - Frame %d: %.2f ms, avg: %.2f ms, FPS: %.1f\n",
                data->frame_count, frame_processing_time, avg, (avg > 0.0 ? 1000.0 / avg : 0.0));
        }

        // Step 2: Reconstruct NV12
        cv::Mat nv12_output(height * 3 / 2, width, CV_8UC1);
        memcpy(nv12_output.data, y_plane_out.data, y_size);
        // Fill UV with neutral value 128
        memset(nv12_output.data + y_size, 128, uv_size);

        // Step 3: Allocate new GstBuffer
        GstBuffer *processed_buffer = gst_buffer_new_allocate(NULL, y_size + uv_size, NULL);
        if (!processed_buffer) {
            g_printerr("Failed to allocate processed buffer\n");
            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        GstMapInfo processed_map_info;
        if (gst_buffer_map(processed_buffer, &processed_map_info, GST_MAP_WRITE)) {
            memcpy(processed_map_info.data, nv12_output.data, y_size + uv_size);
            gst_buffer_unmap(processed_buffer, &processed_map_info);

            // copy timestamps from input buffer to keep timing coherent
            gst_buffer_copy_into(processed_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), processed_buffer);
            if (ret != GST_FLOW_OK) {
                g_printerr("Failed to push buffer to appsrc: %d\n", ret);
                // processed_buffer ownership passed to appsrc on success; if failure, unref
                gst_buffer_unref(processed_buffer);
            }
        } else {
            g_printerr("Failed to map processed buffer\n");
            gst_buffer_unref(processed_buffer);
        }
    } catch (const std::exception& e) {
        g_printerr("OpenCV error: %s\n", e.what());
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// Handle pipeline messages including errors
static gboolean bus_message_cb(GstBus *bus, GstMessage *message, gpointer user_data) {
    (void)bus;
    CustomData *data = (CustomData *)user_data;

    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_EOS:
            g_print("End of stream reached.\n");
            if (data->loop_playback && data->sink_pipeline) {
                g_print("Restarting playback...\n");
                // Seek back to start of file pipeline
                if (!gst_element_seek_simple(
                        data->sink_pipeline, GST_FORMAT_TIME,
                        (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_KEY_UNIT), 0)) {
                    g_printerr("Seek failed; stopping.\n");
                    if (data->loop) g_main_loop_quit(data->loop);
                }
            } else {
                if (data->loop) g_main_loop_quit(data->loop);
            }
            break;

        case GST_MESSAGE_ERROR: {
            GError *err = NULL;
            gchar *debug_info = NULL;
            gst_message_parse_error(message, &err, &debug_info);
            g_printerr("Error from element %s: %s\n",
                       GST_OBJECT_NAME(message->src), err ? err->message : "unknown");
            g_printerr("Debug info: %s\n", debug_info ? debug_info : "none");
            if (err) g_error_free(err);
            if (debug_info) g_free(debug_info);
            if (data->loop) g_main_loop_quit(data->loop);
            break;
        }

        case GST_MESSAGE_WARNING: {
            GError *err = NULL;
            gchar *debug_info = NULL;
            gst_message_parse_warning(message, &err, &debug_info);
            g_print("Warning from element %s: %s\n",
                    GST_OBJECT_NAME(message->src), err ? err->message : "unknown");
            g_print("Debug info: %s\n", debug_info ? debug_info : "none");
            if (err) g_error_free(err);
            if (debug_info) g_free(debug_info);
            break;
        }

        case GST_MESSAGE_STATE_CHANGED: {
            // Only print for our top-level pipelines to avoid spam
            if (GST_MESSAGE_SRC(message) == GST_OBJECT(data->sink_pipeline) ||
                GST_MESSAGE_SRC(message) == GST_OBJECT(data->src_pipeline)) {
                GstState old_state, new_state, pending_state;
                gst_message_parse_state_changed(message, &old_state, &new_state, &pending_state);
                g_print("Pipeline state changed from %s to %s\n",
                        gst_element_state_get_name(old_state),
                        gst_element_state_get_name(new_state));
            }
            break;
        }

        default:
            break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // Parse command line arguments
    gboolean use_h265 = FALSE;
    int bitrate_kbps = 25000; // default
    gchar *input_file = NULL;
    gboolean loop_playback = FALSE;
    int target_width = 1280;
    int target_height = 720;
    int target_fps_num = 30;
    int target_fps_den = 1;

    for (int i = 1; i < argc; ++i) {
        if (g_str_has_prefix(argv[i], "--codec=")) {
            const char *val = strchr(argv[i], '=');
            if (val && g_ascii_strcasecmp(val + 1, "h265") == 0) use_h265 = TRUE;
        } else if (g_strcmp0(argv[i], "--codec") == 0 && i + 1 < argc) {
            if (g_ascii_strcasecmp(argv[i + 1], "h265") == 0) use_h265 = TRUE;
            i++;
        } else if (g_str_has_prefix(argv[i], "--bitrate=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                int v = atoi(val + 1);
                if (v > 0) bitrate_kbps = v;
            }
        } else if (g_strcmp0(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            int v = atoi(argv[i + 1]);
            if (v > 0) bitrate_kbps = v;
            i++;
        } else if (g_str_has_prefix(argv[i], "--input=")) {
            const char *val = strchr(argv[i], '=');
            if (val) input_file = g_strdup(val + 1);
        } else if (g_strcmp0(argv[i], "--input") == 0 && i + 1 < argc) {
            input_file = g_strdup(argv[i + 1]);
            i++;
        } else if (g_strcmp0(argv[i], "--loop") == 0) {
            loop_playback = TRUE;
        } else if (g_str_has_prefix(argv[i], "--resolution=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                if (sscanf(val + 1, "%dx%d", &target_width, &target_height) != 2) {
                    g_printerr("Invalid resolution format. Use --resolution=WIDTHxHEIGHT\n");
                }
            }
        } else if (g_str_has_prefix(argv[i], "--fps=")) {
            const char *val = strchr(argv[i], '=');
            if (val) {
                if (sscanf(val + 1, "%d/%d", &target_fps_num, &target_fps_den) != 2) {
                    target_fps_num = atoi(val + 1);
                    target_fps_den = 1;
                }
            }
        }
    }

    if (!input_file) {
        g_printerr("Usage: %s --input=/path/to/video.mp4 [OPTIONS]\n", argv[0]);
        g_printerr("Options:\n");
        g_printerr("  --codec=h264|h265     Encoder codec (default: h264)\n");
        g_printerr("  --bitrate=N           Bitrate in kbps (default: 25000)\n");
        g_printerr("  --resolution=WxH      Target resolution (default: 1280x720)\n");
        g_printerr("  --fps=N or N/D        Target framerate (default: 30/1)\n");
        g_printerr("  --loop                Loop playback\n");
        return -1;
    }

    if (!g_file_test(input_file, G_FILE_TEST_EXISTS)) {
        g_printerr("Error: Input file '%s' does not exist\n", input_file);
        g_free(input_file);
        return -1;
    }

    g_print("Input: %s\n", input_file);
    g_print("Target resolution: %dx%d @ %d/%d fps\n",
            target_width, target_height, target_fps_num, target_fps_den);
    g_print("Encoder: %s, target-bitrate: %d kbps\n",
            use_h265 ? "H.265" : "H.264", bitrate_kbps);
    g_print("Loop playback: %s\n", loop_playback ? "enabled" : "disabled");

    CustomData data = {};
    data.video_info_valid = FALSE;
    data.processing_timer = g_timer_new();
    data.total_processing_time = 0.0;
    data.frame_count = 0;
    data.input_file = input_file;
    data.loop_playback = loop_playback;
    data.loop = NULL;
    data.sink_pipeline = NULL;
    data.src_pipeline = NULL;

    GError *error = NULL;

    // 1) File input pipeline → NV12@WxH@fps → appsink
    // If "omxh264dec" is unavailable, replace with "avdec_h264".
    gchar *sink_pipeline_str = g_strdup_printf(
        "filesrc location=%s ! "
        "qtdemux ! queue ! h264parse ! omxh264dec ! "
        "videoconvert ! videoscale ! videorate ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/%d ! "
        "appsink name=my_sink emit-signals=true max-buffers=2 drop=false sync=true",
        input_file, target_width, target_height, target_fps_num, target_fps_den
    );

    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error ? error->message : "unknown");
        g_clear_error(&error);
        g_free(input_file);
        return -1;
    }

    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");
    data.sink_pipeline = sink_pipeline;

    // 2) appsrc → encoder → pay → udpsink
    gchar *src_pipeline_str = NULL;
    if (use_h265) {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/%d ! "
            "omxh265enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h265, alignment=au ! "
            "rtph265pay pt=96 ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 sync=true async=false qos-dscp=60",
            target_width, target_height, target_fps_num, target_fps_den, bitrate_kbps
        );
    } else {
        src_pipeline_str = g_strdup_printf(
            "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/%d ! "
            "omxh264enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
            "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=%d "
            "gop-mode=low-delay-p ! video/x-h264, alignment=nal ! "
            "rtph264pay pt=96 ! "
            "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 sync=true async=false qos-dscp=60",
            target_width, target_height, target_fps_num, target_fps_den, bitrate_kbps
        );
    }

    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error ? error->message : "unknown");
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        g_free(input_file);
        return -1;
    }

    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");
    data.src_pipeline = src_pipeline;

    // Register callbacks
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    // Bus watches — pass CustomData*, not GMainLoop*
    GstBus *sink_bus = gst_element_get_bus(sink_pipeline);
    GstBus *src_bus  = gst_element_get_bus(src_pipeline);

    // Main loop
    GMainLoop *main_loop = g_main_loop_new(NULL, FALSE);
    data.loop = main_loop;

    gst_bus_add_watch(sink_bus, bus_message_cb, &data);
    gst_bus_add_watch(src_bus,  bus_message_cb, &data);

    g_print("Processing video file. Press Ctrl+C to exit.\n");

    // Start pipelines
    GstStateChangeReturn src_ret  = gst_element_set_state(src_pipeline,  GST_STATE_PLAYING);
    GstStateChangeReturn sink_ret = gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    if (src_ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Failed to start src pipeline\n");
        gst_object_unref(sink_bus);
        gst_object_unref(src_bus);
        gst_object_unref(sink_pipeline);
        gst_object_unref(src_pipeline);
        g_timer_destroy(data.processing_timer);
        g_free(input_file);
        g_main_loop_unref(main_loop);
        return -1;
    }

    if (sink_ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Failed to start sink pipeline\n");
        gst_object_unref(sink_bus);
        gst_object_unref(src_bus);
        gst_element_set_state(src_pipeline, GST_STATE_NULL);
        gst_object_unref(sink_pipeline);
        gst_object_unref(src_pipeline);
        g_timer_destroy(data.processing_timer);
        g_free(input_file);
        g_main_loop_unref(main_loop);
        return -1;
    }

    g_main_loop_run(main_loop);

    // Cleanup
    gst_object_unref(sink_bus);
    gst_object_unref(src_bus);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline,  GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    g_timer_destroy(data.processing_timer);
    g_free(input_file);
    g_main_loop_unref(main_loop);

    return 0;
}
