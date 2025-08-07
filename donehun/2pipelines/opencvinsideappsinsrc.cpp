#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <opencv2/opencv.hpp>

typedef struct {
    GstElement *appsrc;
    GstElement *appsink;
    gboolean video_info_valid;
    GstVideoInfo video_info;
    GTimer *processing_timer;
    double total_processing_time;
    int frame_count;
} CustomData;

// Called when appsink has a new sample
GstFlowReturn new_sample_cb(GstAppSink *appsink, gpointer user_data) {
    CustomData *data = (CustomData *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("Failed to pull sample from appsink\n");
        return GST_FLOW_ERROR;
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
            g_print("Video info: %dx%d\n", data->video_info.width, data->video_info.height);
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

    size_t y_size = width * height;
    size_t uv_size = width * height / 2;
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
                data->frame_count, frame_processing_time, avg, 1000.0 / avg);
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
            gst_buffer_copy_into(processed_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), processed_buffer);
            if (ret != GST_FLOW_OK) {
                g_printerr("Failed to push buffer to appsrc: %d\n", ret);
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

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    CustomData data = {};
    data.video_info_valid = FALSE;
    data.processing_timer = g_timer_new();
    data.total_processing_time = 0.0;
    data.frame_count = 0;

    GError *error = NULL;

    // 1. Camera pipeline (appsink)
    gchar *sink_pipeline_str = g_strdup_printf(
        "v4l2src device=/dev/video0 io-mode=4 ! "
        "video/x-raw, format=NV12, width=1920, height=1080, framerate=60/1 ! "
        "appsink name=my_sink emit-signals=true max-buffers=2 drop=true sync=false"
    );
    GstElement *sink_pipeline = gst_parse_launch(sink_pipeline_str, &error);
    g_free(sink_pipeline_str);
    if (!sink_pipeline) {
        g_printerr("Failed to create sink pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    data.appsink = gst_bin_get_by_name(GST_BIN(sink_pipeline), "my_sink");

    // 2. Streaming pipeline (appsrc → omxh264enc → rtph264pay → udpsink)
    gchar *src_pipeline_str = g_strdup_printf(
        "appsrc name=my_src is-live=true format=GST_FORMAT_TIME do-timestamp=true ! "
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
        "omxh264enc num-slices=8 periodicity-idr=240 cpb-size=500 gdr-mode=horizontal "
        "initial-delay=250 control-rate=low-latency prefetch-buffer=true target-bitrate=20000 "
        "gop-mode=low-delay-p ! video/x-h264, alignment=nal ! "
        "rtph264pay ! "
        "udpsink buffer-size=60000000 host=192.168.25.69 port=5004 async=false max-lateness=-1 qos-dscp=60"
    );
    GstElement *src_pipeline = gst_parse_launch(src_pipeline_str, &error);
    g_free(src_pipeline_str);
    if (!src_pipeline) {
        g_printerr("Failed to create src pipeline: %s\n", error->message);
        g_clear_error(&error);
        gst_object_unref(sink_pipeline);
        return -1;
    }

    data.appsrc = gst_bin_get_by_name(GST_BIN(src_pipeline), "my_src");

    // Register appsink callback
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_cb), &data);

    gst_element_set_state(src_pipeline, GST_STATE_PLAYING);
    gst_element_set_state(sink_pipeline, GST_STATE_PLAYING);

    g_print("Running. Press Ctrl+C to exit.\n");

    // Main loop: Wait for EOS or error
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

    gst_object_unref(bus);
    gst_element_set_state(sink_pipeline, GST_STATE_NULL);
    gst_element_set_state(src_pipeline, GST_STATE_NULL);
    gst_object_unref(sink_pipeline);
    gst_object_unref(src_pipeline);
    g_timer_destroy(data.processing_timer);

    return 0;
}
