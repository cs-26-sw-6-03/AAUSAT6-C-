#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/video.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

// ─────────────────────────────────────────────
// Types & Aliases
// ─────────────────────────────────────────────

// Raw frame coming off the GStreamer appsink — owns its data
struct RawFrame {
    cv::Mat               data;        // full 4K (e.g. 4056x3040) BGR image
    std::int64_t          pts_ns = 0;  // PTS in nanoseconds
};

// The normalized center point returned by the detector
struct DetectionResult {
    cv::Point2f           center;          // pixel coords in source space
    float                 confidence = 0.f;
    bool                  valid      = false;
};

// A frame after stabilization, ready to crop
struct StabilizedFrame {
    cv::Mat               data;
    cv::Point2f           suggested_center; // propagated from detection
    std::int64_t          pts_ns = 0;
};

// Final deliverable — 1920×1080 crop
struct CroppedFrame {
    cv::Mat               data;        // exactly OUTPUT_W × OUTPUT_H
    cv::Rect              src_roi;     // the ROI used in the stabilized source
    std::int64_t          pts_ns = 0;
};

// ─────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────

constexpr int SRC_W       = 4056;
constexpr int SRC_H       = 3040;
constexpr int OUTPUT_W    = 1920;
constexpr int OUTPUT_H    = 1080;

// ─────────────────────────────────────────────
// I. GStreamer Capture Interface
// ─────────────────────────────────────────────

class IGstreamerCapture {
public:
    virtual ~IGstreamerCapture() = default;

    // Build & start the pipeline. Returns false on failure.
    virtual bool            start(const std::string& pipeline_description) = 0;

    // Stop and tear down the pipeline.
    virtual void            stop() = 0;

    // Pull the next frame from the appsink. Blocks until one is available
    // or the pipeline is stopped. Returns nullopt when the stream ends (EOS)
    // or on error.
    virtual std::optional<RawFrame> pull_frame() = 0;
};

// ─────────────────────────────────────────────
// II. Object Detection Interface
// ─────────────────────────────────────────────

// The detector is given a reference image (the "template" file) at init time.
// For each call to detect(), it returns the pixel center of the best match
// in the provided source-resolution frame.

class IObjectDetector {
public:
    virtual ~IObjectDetector() = default;

    // Load the DNN model and the reference object image.
    //   model_config   – e.g. path to .cfg / .pbtxt
    //   model_weights  – e.g. path to .weights / .pb / .onnx
    //   reference_image – image file of the object to track
    virtual bool            init(const std::string& model_config,
                                 const std::string& model_weights,
                                 const std::string& reference_image) = 0;

    // Run detection on a full source-resolution frame.
    // Returns a DetectionResult; result.valid == false if nothing found.
    virtual DetectionResult detect(const RawFrame& frame) = 0;

    // Optional: warm up the model with a dummy forward pass.
    virtual void            warmup() {}
};

// ─────────────────────────────────────────────
// III. Video Stabilization Interface
// ─────────────────────────────────────────────

// Accepts the raw 4K frame plus the detected center (so the stabilizer can
// preserve the subject position). Outputs a stabilized frame of the same
// source resolution.

class IVideoStabilizer {
public:
    virtual ~IVideoStabilizer() = default;

    // Initialize the stabilizer (e.g. load a pre-trained flow model,
    // set smoothing window size, etc.)
    virtual bool            init(const std::string& model_config  = "",
                                 const std::string& model_weights = "") = 0;

    // Feed a new raw frame. Returns the stabilized version at the same
    // resolution, with the detected center adjusted for any applied transform.
    virtual StabilizedFrame stabilize(const RawFrame&      frame,
                                      const DetectionResult& detection) = 0;

    // Flush any internal buffer (call at EOS).
    virtual void            flush() = 0;
};

// ─────────────────────────────────────────────
// IV. Cropping Interface
// ─────────────────────────────────────────────

// Takes a stabilized source-resolution frame and the desired crop center,
// outputs a clamped OUTPUT_W×OUTPUT_H region.
//
// Clamping rule: if the ideal rect would exceed the source boundary, the rect
// is shifted (not scaled) so it fits entirely within [0, SRC_W) × [0, SRC_H).

class IFrameCropper {
public:
    virtual ~IFrameCropper() = default;

    // Compute the output ROI from the given center, clamped to source bounds.
    // Pure utility — does not own state.
    virtual cv::Rect        compute_roi(cv::Point2f center,
                                        int src_w = SRC_W,
                                        int src_h = SRC_H,
                                        int out_w = OUTPUT_W,
                                        int out_h = OUTPUT_H) const = 0;

    // Perform the actual crop and return the final frame.
    virtual CroppedFrame    crop(const StabilizedFrame& frame) = 0;
};

// ─────────────────────────────────────────────
// V. Pipeline Orchestrator Interface
// ─────────────────────────────────────────────

// Wires the four stages together and drives the frame loop.
// Accepts a callback that is invoked for every completed output frame.

using FrameCallback = std::function<void(const CroppedFrame&)>;

class IPipelineOrchestrator {
public:
    virtual ~IPipelineOrchestrator() = default;

    struct Config {
        std::string gst_pipeline_desc;    // full GStreamer launch string
        std::string detector_config;
        std::string detector_weights;
        std::string detector_reference;   // path to the object image file
        std::string stabilizer_config;    // leave empty for classical methods
        std::string stabilizer_weights;
    };

    // Wire up all components; call before run().
    virtual bool            init(const Config&     cfg,
                                 FrameCallback      on_frame) = 0;

    // Blocking: run the frame loop until EOS or stop() is called.
    virtual void            run() = 0;

    // Signal the frame loop to exit cleanly from another thread.
    virtual void            stop() = 0;

    virtual bool            is_running() const = 0;
};