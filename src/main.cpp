#include <gst/gst.h>
#include <iostream>

int main(int argc, char *argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);
    
    std::cout << "GStreamer initialized successfully!" << std::endl;
    
    // Get version
    guint major, minor, micro, nano;
    gst_version(&major, &minor, &micro, &nano);
    
    std::cout << "GStreamer version: " 
              << major << "." << minor << "." << micro << std::endl;
    
    // Create a simple pipeline
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(
        "videotestsrc num-buffers=100 ! autovideosink",
        &error
    );
    
    if (error) {
        std::cerr << "Error: " << error->message << std::endl;
        g_error_free(error);
        return 1;
    }
    
    std::cout << "Pipeline created, starting playback..." << std::endl;
    
    // Start playing
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    // Wait until error or EOS (end of stream)
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus,
        GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS)
    );
    
    // Cleanup
    if (msg) {
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    
    std::cout << "Done!" << std::endl;
    return 0;
}