#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

using namespace cv;
using namespace std;

// Callback function to process each frame
GstFlowReturn new_sample_callback(GstElement *sink, gpointer data)
{
    // Get FAST detector
    Ptr<FastFeatureDetector> *fast =
        static_cast<Ptr<FastFeatureDetector>*>(data);

    // Pull sample from appsink
    GstSample *sample =
        gst_app_sink_pull_sample(GST_APP_SINK(sink));

    if (!sample)
        return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);

    GstStructure *structure =
        gst_caps_get_structure(caps, 0);

    int width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Create OpenCV Mat (BGR format)
    Mat frame(height, width, CV_8UC3, (void*)map.data);

    // Clone because buffer will be unmapped
    Mat frame_copy = frame.clone();

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    // Convert to grayscale
    Mat gray;
    cvtColor(frame_copy, gray, COLOR_BGR2GRAY);

    // Detect FAST keypoints
    vector<KeyPoint> keypoints;
    (*fast)->detect(gray, keypoints);

    // Draw keypoints
    Mat output;
    drawKeypoints(frame_copy, keypoints, output,
                  Scalar(0, 255, 0),
                  DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    imshow("FAST Features - GStreamer", output);
    waitKey(1);

    cout << "Detected "
         << keypoints.size()
         << " keypoints" << endl;

    return GST_FLOW_OK;
}

int main(int argc, char *argv[])
{
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Create FAST detector
    Ptr<FastFeatureDetector> fast =
        FastFeatureDetector::create(
            20,    
            true, 
            FastFeatureDetector::TYPE_9_16
        );

    cout << "GStreamer initialized successfully!" << endl;

    guint major, minor, micro, nano;
    gst_version(&major, &minor, &micro, &nano);

    cout << "GStreamer version: "
         << major << "."
         << minor << "."
         << micro << endl;

    // Create pipeline
    GError *error = nullptr;

    GstElement *pipeline = gst_parse_launch(
        "filesrc location=/home/tobia/GoogleEarthTest.mp4 ! "
        "decodebin ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink sync=false",
        &error
    );

    if (error)
    {
        cerr << "Error: "
             << error->message << endl;
        g_error_free(error);
        return 1;
    }

    GstElement *appsink =
        gst_bin_get_by_name(GST_BIN(pipeline), "sink");

    if (!appsink)
    {
        cerr << "Failed to get appsink element" << endl;
        gst_object_unref(pipeline);
        return 1;
    }

    // Configure appsink
    g_object_set(G_OBJECT(appsink),
                 "emit-signals", TRUE,
                 "max-buffers", 1,
                 "drop", TRUE,
                 NULL);

    // Connect callback
    g_signal_connect(appsink,
                     "new-sample",
                     G_CALLBACK(new_sample_callback),
                     &fast);

    cout << "Pipeline created, starting playback..." << endl;

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Wait for EOS or error
    GstBus *bus = gst_element_get_bus(pipeline);

    GstMessage *msg =
        gst_bus_timed_pop_filtered(
            bus,
            GST_CLOCK_TIME_NONE,
            (GstMessageType)
            (GST_MESSAGE_ERROR | GST_MESSAGE_EOS)
        );

    if (msg)
    {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
        {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error(msg,
                                    &err,
                                    &debug_info);

            cerr << "Error: "
                 << err->message << endl;

            g_error_free(err);
            g_free(debug_info);
        }

        gst_message_unref(msg);
    }

    gst_object_unref(bus);
    gst_object_unref(appsink);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    destroyAllWindows();

    cout << "Done!" << endl;

    return 0;
}
