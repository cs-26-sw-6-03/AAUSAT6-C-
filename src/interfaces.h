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
    cv::Mat                   data;
    std::int64_t              pts_ns = 0;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;
    bool                      features_computed = false;
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

// Final deliverable — cropped region at requested output resolution
struct CroppedFrame {
    cv::Mat               data;        // cropped frame at output resolution
    cv::Rect              src_roi;     // the ROI used in the stabilized source
    std::int64_t          pts_ns = 0;
};

// ─────────────────────────────────────────────
// I. Video Input Stream Interface
// ─────────────────────────────────────────────

// Generic interface for reading frames from any video source:
// - Video files (via GStreamer, FFmpeg, OpenCV, etc.)
// - Live cameras (v4l2, RTSP streams, etc.)
// - Test patterns or synthetic data

class IVideoInputStream {
public:
    virtual ~IVideoInputStream() = default;

    // Initialize and start the stream with the given configuration.
    // The config string format depends on the implementation.
    // Returns false on failure.
    virtual bool            start(const std::string& config) = 0;

    // Stop and tear down the stream.
    virtual void            stop() = 0;

    // Pull the next frame from the stream. Blocks until one is available
    // or the stream is stopped. Returns nullopt when the stream ends (EOS)
    // or on error.
    virtual std::optional<RawFrame> pull_frame() = 0;
};

// ─────────────────────────────────────────────
// II. Video Output Stream Interface
// ─────────────────────────────────────────────

// Generic interface for displaying or saving output frames:
// - OpenCV windows (cv::imshow)
// - Video files (via GStreamer, FFmpeg, OpenCV VideoWriter, etc.)
// - Network streams (RTSP, WebRTC, etc.)
// - Headless/null sink for testing

class IVideoOutputStream {
public:
    virtual ~IVideoOutputStream() = default;

    // Initialize the output stream with the given configuration.
    // The config string format depends on the implementation.
    // Returns false on failure.
    virtual bool            init(const std::string& config) = 0;

    // Write a frame to the output stream.
    // Returns false on error.
    virtual bool            write_frame(const CroppedFrame& frame) = 0;

    // Flush any buffers and close the stream.
    virtual void            close() = 0;

    // Check if the output stream is still active.
    // For windows: returns false if user closed the window.
    // For files: returns false if write error occurred.
    virtual bool            is_open() const = 0;
};

// ─────────────────────────────────────────────
// III. Feature Detection Interface
// ─────────────────────────────────────────────

// Generic interface for detecting and describing features in frames.
// Can be used for:
// - Object detection (finding a reference object)
// - Feature matching (BRISK, ORB, SIFT, etc.)
// - Deep learning based detection
// - Template matching

class IFeatureDetector {
public:
    virtual ~IFeatureDetector() = default;

    // Initialize the detector with configuration.
    //   model_config   – e.g. path to .cfg / .pbtxt, or detector params
    //   model_weights  – e.g. path to .weights / .pb / .onnx (if using DNN)
    //   reference_image – image file of the object/pattern to track
    virtual bool            init(const std::string& model_config,
                                 const std::string& model_weights,
                                 const std::string& reference_image) = 0;

    // Run detection on a frame.
    // Returns a DetectionResult; result.valid == false if nothing found.
    virtual DetectionResult detect(RawFrame& frame) = 0;

    // Optional: warm up the model with a dummy forward pass.
    virtual void            warmup() {}
};

// ─────────────────────────────────────────────
// IV. Video Stabilization Interface
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
// V. Cropping Interface
// ─────────────────────────────────────────────

// Takes a stabilized source-resolution frame and the desired crop center,
// outputs a clamped crop region at the requested output resolution.
//
// Clamping rule: if the ideal rect would exceed the source boundary, the rect
// is shifted (not scaled) so it fits entirely within the source bounds.

class IFrameCropper {
public:
    virtual ~IFrameCropper() = default;

    // Compute the output ROI from the given center, clamped to source bounds.
    // Pure utility — does not own state.
    // All dimensions must be provided explicitly.
    virtual cv::Rect        compute_roi(cv::Point2f center,
                                        int src_w,
                                        int src_h,
                                        int out_w,
                                        int out_h) const = 0;

    // Perform the actual crop and return the final frame.
    // The output dimensions are specified by out_w and out_h.
    virtual CroppedFrame    crop(const StabilizedFrame& frame,
                                 int out_w,
                                 int out_h) = 0;
};

// ─────────────────────────────────────────────
// VI. Pipeline Orchestrator Interface
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