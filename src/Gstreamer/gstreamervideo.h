#pragma once

#include "interfaces.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// GstreamerCapture
//
// Simplified implementation using the pull model — no queue, no mutex.
//
// Design:
//   - The pipeline includes videorate to decimate 60fps → 30fps upstream
//   - pull_frame() directly calls gst_app_sink_pull_sample(), which blocks
//     until a frame is ready or the pipeline hits EOS/error
//   - GStreamer's internal appsink queue handles buffering (max-buffers=1)
// ─────────────────────────────────────────────────────────────────────────────

class GstreamerCapture : public IGstreamerCapture {
public:
    GstreamerCapture() = default;
    ~GstreamerCapture() override { stop(); }

    // ── IGstreamerCapture ────────────────────────────────────────────────────

    bool start(const std::string& pipeline_description) override;
    void stop() override;

    std::optional<RawFrame> pull_frame() override;

private:
    // ── GStreamer objects ────────────────────────────────────────────────────
    GstElement* pipeline_  = nullptr;
    GstElement* appsink_   = nullptr;
    GstBus*     bus_       = nullptr;

    std::atomic<bool> running_{ false };

    // ── Helpers ──────────────────────────────────────────────────────────────
    RawFrame buffer_to_frame(GstSample* sample) const;
    void     check_bus_messages();
};