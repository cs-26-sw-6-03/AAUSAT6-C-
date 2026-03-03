#pragma once

#include "interfaces.h"

#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// OpenCVWindowOutput
//
// Implementation of IVideoOutputStream using OpenCV's cv::imshow().
// Displays frames in a named window.
//
// Config format: window name (e.g., "Output 1920x1080")
// ─────────────────────────────────────────────────────────────────────────────

class OpenCVWindowOutput : public IVideoOutputStream {
public:
    OpenCVWindowOutput() = default;
    ~OpenCVWindowOutput() override { close(); }

    // ── IVideoOutputStream ───────────────────────────────────────────────────

    bool init(const std::string& config) override;
    bool write_frame(const CroppedFrame& frame) override;
    void close() override;
    bool is_open() const override;

private:
    std::string window_name_;
    bool        is_open_ = false;
};
