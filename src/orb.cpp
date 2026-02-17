#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

using namespace cv;
using namespace std;

// Callback function to process each frame
GstFlowReturn new_sample_callback(GstElement *sink, gpointer data) {
    Ptr<ORB> *orb = static_cast<Ptr<ORB>*>(data);
    
    // Pull the sample from appsink
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {
        return GST_FLOW_ERROR;
    }
    
    // Get the buffer from sample
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    
    // Get video info from caps
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    int width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    
    // Map buffer to access raw data
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    
    // Create OpenCV Mat from buffer data
    // Assuming BGR format (adjust based on your pipeline)
    Mat frame(height, width, CV_8UC3, (void*)map.data);
    Mat frame_copy = frame.clone(); // Clone because we'll unmap the buffer
    
    // Unmap buffer
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    // Process with ORB
    Mat gray;
    cvtColor(frame_copy, gray, COLOR_BGR2GRAY);
    
    vector<KeyPoint> keypoints;
    Mat descriptors;
    (*orb)->detectAndCompute(gray, Mat(), keypoints, descriptors);
    
    // Draw keypoints
    Mat frame_with_keypoints;
    drawKeypoints(frame_copy, keypoints, frame_with_keypoints, 
                  Scalar(0, 255, 0), 
                  DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Display result
    imshow("ORB Features - GStreamer", frame_with_keypoints);
    waitKey(1);
    
    cout << "Detected " << keypoints.size() << " keypoints" << endl;
    
    return GST_FLOW_OK;
}

int main(int argc, char *argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);
    
    // Init ORB class
    Ptr<ORB> orb = ORB::create(10);
    
    cout << "GStreamer initialized successfully!" << endl;
    
    // Get version
    guint major, minor, micro, nano;
    gst_version(&major, &minor, &micro, &nano);
    cout << "GStreamer version: " 
         << major << "." << minor << "." << micro << endl;
    
    // Create pipeline with appsink to extract frames
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(
        "filesrc location=/home/digita/projects/AAUSAT6-C-/Untitled.mp4 ! "
        "decodebin ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink sync=false",
        &error
    );
    
    if (error) {
        cerr << "Error: " << error->message << endl;
        g_error_free(error);
        return 1;
    }
    
    // Get appsink element
    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    if (!appsink) {
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
    
    // Connect callback for new samples
    g_signal_connect(appsink, "new-sample", 
                     G_CALLBACK(new_sample_callback), &orb);
    
    cout << "Pipeline created, starting playback..." << endl;
    
    // Start playing
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    // Wait until error or EOS
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus,
        GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS)
    );
    
    // Handle messages
    if (msg) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error(msg, &err, &debug_info);
            cerr << "Error: " << err->message << endl;
            g_error_free(err);
            g_free(debug_info);
        }
        gst_message_unref(msg);
    }
    
    // Cleanup
    gst_object_unref(bus);
    gst_object_unref(appsink);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    
    destroyAllWindows();
    cout << "Done!" << endl;
    
    return 0;
}