#include "VideoInputStream/gstreamervideo.h"

#include <iostream>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

bool GstreamerCapture::start(const std::string& config)
{
    if (running_.load()) {
        std::cerr << "[GstreamerCapture] Already running — call stop() first.\n";
        return false;
    }

    // ── Build pipeline ───────────────────────────────────────────────────────
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(config.c_str(), &error);

    if (error) {
        std::cerr << "[GstreamerCapture] Pipeline parse error: "
                  << error->message << "\n";
        g_error_free(error);
        return false;
    }

    // ── Grab appsink (must be named "sink" in the pipeline string) ───────────
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!appsink_) {
        std::cerr << "[GstreamerCapture] Could not find element named 'sink'.\n";
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    // ── Configure appsink ────────────────────────────────────────────────────
    //   max-buffers=1 + drop=TRUE → always give us the latest frame
    //   sync=FALSE                → decode as fast as possible
    //   emit-signals=FALSE        → we use the pull model, not signals
    g_object_set(G_OBJECT(appsink_),
                 "emit-signals", FALSE,
                 "max-buffers",  1,
                 "drop",         TRUE,
                 "sync",         FALSE,
                 nullptr);

    // ── Grab bus for error / EOS polling ────────────────────────────────────
    bus_ = gst_element_get_bus(pipeline_);

    // ── Start playback ───────────────────────────────────────────────────────
    GstStateChangeReturn ret =
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[GstreamerCapture] Failed to set pipeline to PLAYING.\n";
        stop();
        return false;
    }

    running_.store(true);

    std::cout << "[GstreamerCapture] Pipeline started.\n";
    return true;
}

void GstreamerCapture::stop()
{
    running_.store(false);

    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_) {
        gst_object_unref(bus_);
        bus_ = nullptr;
    }
    if (appsink_) {
        gst_object_unref(appsink_);
        appsink_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }

    std::cout << "[GstreamerCapture] Pipeline stopped.\n";
}

std::optional<RawFrame> GstreamerCapture::pull_frame()
{
    if (!running_.load()) {
        return std::nullopt;
    }

    // Check for errors/EOS on the bus (non-blocking)
    check_bus_messages();

    if (!running_.load()) {
        return std::nullopt;
    }

    // Pull a sample from appsink. This blocks until:
    //   1. A frame is available
    //   2. EOS is reached
    //   3. An error occurs
    //   4. The pipeline is stopped
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
    
    if (!sample) {
        // EOS or error — check bus one more time to report the reason
        check_bus_messages();
        running_.store(false);
        return std::nullopt;
    }

    try {
        RawFrame frame = buffer_to_frame(sample);
        gst_sample_unref(sample);
        return frame;
    } catch (const std::exception& e) {
        std::cerr << "[GstreamerCapture] Frame decode error: " << e.what() << "\n";
        gst_sample_unref(sample);
        running_.store(false);
        return std::nullopt;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

RawFrame GstreamerCapture::buffer_to_frame(GstSample* sample) const
{
    GstBuffer*    buffer    = gst_sample_get_buffer(sample);
    GstCaps*      caps      = gst_sample_get_caps(sample);
    GstStructure* structure = gst_caps_get_structure(caps, 0);

    int width  = 0;
    int height = 0;
    gst_structure_get_int(structure, "width",  &width);
    gst_structure_get_int(structure, "height", &height);

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid frame dimensions from caps.");
    }

    GstMapInfo map{};
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        throw std::runtime_error("Failed to map GstBuffer.");
    }

    // We request BGR from the pipeline (video/x-raw,format=BGR),
    // so CV_8UC3 matches directly.
    cv::Mat view(height, width, CV_8UC3, map.data);

    RawFrame frame;
    frame.data   = view.clone();           // deep copy before we unmap
    frame.pts_ns = static_cast<std::int64_t>(
                       GST_BUFFER_PTS(buffer));

    gst_buffer_unmap(buffer, &map);
    return frame;
}

void GstreamerCapture::check_bus_messages()
{
    if (!bus_) return;

    // Non-blocking peek — returns nullptr immediately if no message.
    GstMessage* msg = gst_bus_pop_filtered(
        bus_,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (!msg) return;

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS: {
            std::cout << "[GstreamerCapture] EOS received.\n";
            running_.store(false);
            break;
        }
        case GST_MESSAGE_ERROR: {
            GError* err       = nullptr;
            gchar*  debug_str = nullptr;
            gst_message_parse_error(msg, &err, &debug_str);
            std::cerr << "[GstreamerCapture] Bus error: " << err->message
                      << "\n  Debug: " << (debug_str ? debug_str : "none")
                      << "\n";
            g_error_free(err);
            g_free(debug_str);
            running_.store(false);
            break;
        }
        default:
            break;
    }

    gst_message_unref(msg);
}