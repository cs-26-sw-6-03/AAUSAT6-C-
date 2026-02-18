#include "interfaces.h"
#include "VideoInputStream/gstreamervideo.h"
#include "FeatureDetection/StubDetector.h"
#include "FeatureDetection/BriskDetector.h"
#include "Stabilization/StubStabilizer.h"
#include "Cropping/StubCropper.h"
#include "VideoOutputStream/OpenCVWindowOutput.h"
#include "VideoOutputStream/GstreamerFileOutput.h"

#include <gst/gst.h>
#include <opencv2/highgui.hpp>

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Resolution configuration
// ─────────────────────────────────────────────────────────────────────────────

struct ResolutionConfig {
    int src_width;
    int src_height;
    int output_width;
    int output_height;
};

// ─────────────────────────────────────────────────────────────────────────────
// Graceful shutdown on Ctrl-C
// ─────────────────────────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{ false };

static void signal_handler(int /*sig*/)
{
    g_shutdown.store(true);
}

// ─────────────────────────────────────────────────────────────────────────────
// Build the GStreamer launch string
//
// Source: file  →  decode  →  framerate  →  scale to target resolution  →  BGR  →  appsink
//
// If you want to later swap in a live camera, replace the first two elements:
//   v4l2src device=/dev/video0 ! video/x-raw,width=<src_w>,height=<src_h>
// ─────────────────────────────────────────────────────────────────────────────

static std::string build_pipeline(const std::string& video_path,
                                   int src_width, int src_height)
{
    return
        "filesrc location=" + video_path + " ! "
        "decodebin ! "
        "videorate ! "                                   // ← Frame rate conversion
        "video/x-raw,framerate=30/1 ! "                  // ← Force 30fps output
        "videoconvert ! "
        "videoscale ! "
        "video/x-raw,format=BGR,"
            "width=" + std::to_string(src_width) + ","
            "height=" + std::to_string(src_height) + " ! "
        "appsink name=sink sync=false";
}

// ─────────────────────────────────────────────────────────────────────────────
// main
//
// Usage:
//   ./video_pipeline <input_video> <reference_image> [output_file]
//
// Arguments:
//   input_video      - Path to input video file (required)
//   reference_image  - Path to reference object image (required)
//   output_file      - Optional: Path to output video file (e.g., output.mp4)
//                      If not specified, displays output in a window
//
// Examples:
//   ./video_pipeline input.mp4 reference.jpg
//   ./video_pipeline input.mp4 reference.jpg output.mp4
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    // ── GStreamer global init ────────────────────────────────────────────────
    gst_init(&argc, &argv);

    // ── Signal handling ──────────────────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    // ── Resolution Configuration ─────────────────────────────────────────────────────
    ResolutionConfig res_config{
        4056,  // src_width
        3040,  // src_height
        1920,  // output_width
        1080   // output_height
    };
    // ── Configuration ────────────────────────────────────────────────────────
    const std::string video_path      = (argc > 1)
                                          ? argv[1]
                                          : "/home/slessing/Projects/AAUSAT6-C-/Untitled.mp4";
    const std::string reference_image = (argc > 2)
                                          ? argv[2]
                                          : "/home/slessing/Projects/AAUSAT6-C-/reference_object.jpg";
    const std::string output_file     = (argc > 3) ? argv[3] : "";  // Optional output file

    std::cout << "Video source  : " << video_path      << "\n"
              << "Reference img : " << reference_image  << "\n";
    if (!output_file.empty()) {
        std::cout << "Output file   : " << output_file << "\n";
    }

    // ── Instantiate pipeline stages ──────────────────────────────────────────
    auto input      = std::make_unique<GstreamerCapture>();
    auto detector   = std::make_unique<StubDetector>();
    auto stabilizer = std::make_unique<BriskStabilizer>();
    auto cropper    = std::make_unique<StubCropper>();
    
    // Create appropriate output stream based on whether output file is specified
    std::unique_ptr<IVideoOutputStream> output;
    if (!output_file.empty()) {
        output = std::make_unique<GstreamerFileOutput>();
    } else {
        output = std::make_unique<OpenCVWindowOutput>();
    }

    // ── Init detection & stabilization ──────────────────────────────────────
    if (!detector->init("", "", reference_image)) {
        std::cerr << "Detector init failed.\n";
        return 1;
    }
    if (!stabilizer->init("", "")) {
        std::cerr << "Stabilizer init failed.\n";
        return 1;
    }

    // ── Init output stream ───────────────────────────────────────────────────
    std::string output_config;
    if (!output_file.empty()) {
        // GStreamer file output: "output.mp4:x264:30:1920x1080"
        output_config = output_file + ":x264:" + 
                       std::to_string(30) + ":" +
                       std::to_string(res_config.output_width) + "x" + 
                       std::to_string(res_config.output_height);
    } else {
        // OpenCV window output: just the window title
        output_config = "Output (" + 
                       std::to_string(res_config.output_width) + "x" + 
                       std::to_string(res_config.output_height) + ")";
    }
    
    if (!output->init(output_config)) {
        std::cerr << "Output stream init failed.\n";
        return 1;
    }

    // ── Start input stream ───────────────────────────────────────────────────
    const std::string pipeline_str = build_pipeline(video_path,
                                                     res_config.src_width,
                                                     res_config.src_height);
    std::cout << "Pipeline: " << pipeline_str << "\n\n";
    if (!input->start(pipeline_str)) {
        std::cerr << "Failed to start input stream.\n";
        return 1;
    }

    // ── Frame loop ───────────────────────────────────────────────────────────
    std::size_t frame_count = 0;

    while (!g_shutdown.load() && output->is_open()) {

        // 1. Pull raw frame from the input stream (blocks until available or EOS).
        auto maybe_frame = input->pull_frame();
        if (!maybe_frame.has_value()) {
            std::cout << "Stream ended (EOS or error).\n";
            break;
        }
        RawFrame& raw = *maybe_frame;

        // 2. Object detection → get center point only.
        DetectionResult detection = detector->detect(raw);
        if (detection.valid) {
            // Overlay detected centre
            cv::circle(raw.data,
                       static_cast<cv::Point>(detection.center),
                       12, { 0, 255, 0 }, 2);
        }

        // 3. Video stabilization (operates at source resolution).
        StabilizedFrame stabilized = stabilizer->stabilize(raw, detection);

        // 4. Crop to output resolution centred on the detected object.
        CroppedFrame cropped = cropper->crop(stabilized,
                                             res_config.output_width,
                                             res_config.output_height);

        // 5. Write to output stream (display window).
        if (!output->write_frame(cropped)) {
            std::cout << "Output stream closed.\n";
            g_shutdown.store(true);
        }

        ++frame_count;
        if (frame_count % 30 == 0) {
            std::cout << "Processed " << frame_count << " frames  |  "
                      << "ROI: " << cropped.src_roi << "\n";
        }
    }

    // ── Cleanup ──────────────────────────────────────────────────────────────
    stabilizer->flush();
    input->stop();
    output->close();
    cv::destroyAllWindows();

    std::cout << "Done. Total frames processed: " << frame_count << "\n";
    return 0;
}