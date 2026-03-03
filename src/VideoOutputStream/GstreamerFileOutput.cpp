#include "VideoOutputStream/GstreamerFileOutput.h"

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <sstream>

// ─────────────────────────────────────────────────────────────────────────────
// Helper: Build GStreamer pipeline string
// ─────────────────────────────────────────────────────────────────────────────

std::string GstreamerFileOutput::build_pipeline(const std::string& output_file,
                                                 const std::string& encoder,
                                                 int width, int height, int fps)
{
    std::ostringstream oss;
    
    // appsrc configuration
    oss << "appsrc name=src format=time is-live=false "
        << "caps=video/x-raw,format=BGR,width=" << width 
        << ",height=" << height 
        << ",framerate=" << fps << "/1 ! ";
    
    // Color conversion
    oss << "videoconvert ! ";
    
    // Encoder selection
    std::string enc = encoder.empty() ? "x264" : encoder;
    
    if (enc == "x264" || enc == "h264") {
        oss << "x264enc speed-preset=medium tune=zerolatency ! "
            << "h264parse ! ";
    } else if (enc == "x265" || enc == "h265") {
        oss << "x265enc speed-preset=medium tune=zerolatency ! "
            << "h265parse ! ";
    } else if (enc == "vp8") {
        oss << "vp8enc ! ";
    } else if (enc == "vp9") {
        oss << "vp9enc ! ";
    } else {
        // Default to x264
        oss << "x264enc speed-preset=medium tune=zerolatency ! "
            << "h264parse ! ";
    }
    
    // Container/muxer based on file extension
    std::string ext = output_file.substr(output_file.find_last_of(".") + 1);
    
    if (ext == "mp4") {
        oss << "mp4mux ! ";
    } else if (ext == "mkv") {
        oss << "matroskamux ! ";
    } else if (ext == "webm") {
        oss << "webmmux ! ";
    } else if (ext == "avi") {
        oss << "avimux ! ";
    } else {
        // Default to mp4
        oss << "mp4mux ! ";
    }
    
    // File sink
    oss << "filesink location=" << output_file;
    
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: Check bus for errors
// ─────────────────────────────────────────────────────────────────────────────

void GstreamerFileOutput::check_bus_messages()
{
    if (!bus_) return;

    GstMessage* msg;
    while ((msg = gst_bus_pop(bus_)) != nullptr) {
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError* err;
                gchar*  debug_info;
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "[GstreamerFileOutput] Error: " << err->message << "\n";
                if (debug_info) {
                    std::cerr << "[GstreamerFileOutput] Debug: " << debug_info << "\n";
                }
                g_clear_error(&err);
                g_free(debug_info);
                is_open_.store(false);
                break;
            }
            case GST_MESSAGE_EOS:
                std::cout << "[GstreamerFileOutput] End of stream.\n";
                is_open_.store(false);
                break;
            case GST_MESSAGE_WARNING: {
                GError* err;
                gchar*  debug_info;
                gst_message_parse_warning(msg, &err, &debug_info);
                std::cerr << "[GstreamerFileOutput] Warning: " << err->message << "\n";
                g_clear_error(&err);
                g_free(debug_info);
                break;
            }
            default:
                break;
        }
        gst_message_unref(msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

bool GstreamerFileOutput::init(const std::string& config)
{
    if (is_open_.load()) {
        std::cerr << "[GstreamerFileOutput] Already initialized.\n";
        return false;
    }

    // Parse config: "output.mp4:x264:30:1920x1080"
    std::istringstream iss(config);
    std::string output_file, encoder, resolution;
    
    std::getline(iss, output_file, ':');
    std::getline(iss, encoder, ':');
    
    std::string fps_str;
    std::getline(iss, fps_str, ':');
    if (!fps_str.empty()) {
        fps_ = std::stoi(fps_str);
    }
    
    std::getline(iss, resolution, ':');
    if (!resolution.empty()) {
        size_t x_pos = resolution.find('x');
        if (x_pos != std::string::npos) {
            width_  = std::stoi(resolution.substr(0, x_pos));
            height_ = std::stoi(resolution.substr(x_pos + 1));
        }
    }
    
    if (output_file.empty()) {
        std::cerr << "[GstreamerFileOutput] No output file specified.\n";
        return false;
    }
    
    if (width_ <= 0 || height_ <= 0) {
        std::cerr << "[GstreamerFileOutput] Invalid resolution: " 
                  << width_ << "x" << height_ << "\n";
        return false;
    }

    // Build pipeline
    std::string pipeline_str = build_pipeline(output_file, encoder, width_, height_, fps_);
    std::cout << "[GstreamerFileOutput] Pipeline: " << pipeline_str << "\n";

    // Create pipeline
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
        std::cerr << "[GstreamerFileOutput] Pipeline parse error: "
                  << error->message << "\n";
        g_error_free(error);
        return false;
    }

    // Get appsrc
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (!appsrc_) {
        std::cerr << "[GstreamerFileOutput] Could not find appsrc element.\n";
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    // Get bus for error monitoring
    bus_ = gst_element_get_bus(pipeline_);

    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[GstreamerFileOutput] Failed to set pipeline to PLAYING.\n";
        close();
        return false;
    }

    is_open_.store(true);
    frame_count_ = 0;

    std::cout << "[GstreamerFileOutput] Writing to: " << output_file 
              << " (" << width_ << "x" << height_ << " @ " << fps_ << "fps)\n";
    
    return true;
}

bool GstreamerFileOutput::write_frame(const CroppedFrame& frame)
{
    if (!is_open_.load()) {
        return false;
    }

    if (frame.data.empty()) {
        std::cerr << "[GstreamerFileOutput] Received empty frame.\n";
        return false;
    }

    // Check for errors
    check_bus_messages();
    if (!is_open_.load()) {
        return false;
    }

    // Verify frame dimensions match expected
    if (frame.data.cols != width_ || frame.data.rows != height_) {
        std::cerr << "[GstreamerFileOutput] Frame size mismatch. Expected "
                  << width_ << "x" << height_ << ", got "
                  << frame.data.cols << "x" << frame.data.rows << "\n";
        return false;
    }

    // Create GStreamer buffer from OpenCV Mat
    size_t buffer_size = frame.data.total() * frame.data.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, buffer_size, nullptr);

    // Copy frame data
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, frame.data.data, buffer_size);
        gst_buffer_unmap(buffer, &map);
    } else {
        std::cerr << "[GstreamerFileOutput] Failed to map buffer.\n";
        gst_buffer_unref(buffer);
        return false;
    }

    // Set timestamp
    GST_BUFFER_PTS(buffer) = frame.pts_ns;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, fps_);

    // Push buffer to appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    
    if (ret != GST_FLOW_OK) {
        std::cerr << "[GstreamerFileOutput] Failed to push buffer: " << ret << "\n";
        return false;
    }

    frame_count_++;
    
    if (frame_count_ % 30 == 0) {
        std::cout << "[GstreamerFileOutput] Written " << frame_count_ << " frames\n";
    }

    return true;
}

void GstreamerFileOutput::close()
{
    if (!is_open_.load()) {
        return;
    }

    is_open_.store(false);

    // Send EOS to appsrc to finalize the file
    if (appsrc_) {
        gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
    }

    // Wait for EOS to propagate
    if (bus_) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus_,
            GST_CLOCK_TIME_NONE,
            static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR)
        );
        if (msg) {
            gst_message_unref(msg);
        }
    }

    // Stop pipeline
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }

    // Cleanup
    if (bus_) {
        gst_object_unref(bus_);
        bus_ = nullptr;
    }
    if (appsrc_) {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }

    std::cout << "[GstreamerFileOutput] Closed. Total frames written: " 
              << frame_count_ << "\n";
}

bool GstreamerFileOutput::is_open() const
{
    return is_open_.load();
}
