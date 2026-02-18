#pragma once

#include "interfaces.h"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <atomic>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// GstreamerFileOutput
//
// Implementation of IVideoOutputStream using GStreamer to write video files.
// Uses appsrc to push frames into a GStreamer encoding pipeline.
//
// Config format: "output_file.mp4:[encoder]:[framerate]:[width]x[height]"
//   Example: "output.mp4:x264:30:1920x1080"
//            "output.webm:vp9:30:1920x1080"
//   
// Supported encoders: x264, x265, vp8, vp9, h264, h265
// If encoder is empty, defaults to x264
// ─────────────────────────────────────────────────────────────────────────────

class GstreamerFileOutput : public IVideoOutputStream {
public:
    GstreamerFileOutput() = default;
    ~GstreamerFileOutput() override { close(); }

    // ── IVideoOutputStream ───────────────────────────────────────────────────

    bool init(const std::string& config) override;
    bool write_frame(const CroppedFrame& frame) override;
    void close() override;
    bool is_open() const override;

private:
    // ── GStreamer objects ────────────────────────────────────────────────────
    GstElement* pipeline_  = nullptr;
    GstElement* appsrc_    = nullptr;
    GstBus*     bus_       = nullptr;

    std::atomic<bool> is_open_{ false };
    
    // Video parameters
    int width_  = 0;
    int height_ = 0;
    int fps_    = 30;
    
    std::uint64_t frame_count_ = 0;

    // ── Helpers ──────────────────────────────────────────────────────────────
    std::string build_pipeline(const std::string& output_file,
                               const std::string& encoder,
                               int width, int height, int fps);
    void check_bus_messages();
};
