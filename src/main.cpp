#include "interfaces.h"
#include "Gstreamer/gstreamervideo.h"
#include "ObjectDetection/StubDetector.h"
#include "Stabilization/StubStabilizer.h"
#include "Cropping/StubCropper.h"
#include "ObjectDetection/ORBDetector.h"

#include <gst/gst.h>
#include <opencv2/highgui.hpp>

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

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
// Source: file  →  decode  →  framerate  →  scale to 4K  →  BGR  →  appsink
//
// If you want to later swap in a live camera, replace the first two elements:
//   v4l2src device=/dev/video0 ! video/x-raw,width=4056,height=3040
// ─────────────────────────────────────────────────────────────────────────────

static std::string build_pipeline(const std::string& video_path)
{
    return
        "filesrc location=" + video_path + " ! "
        "decodebin ! "
        "videorate ! "                                   // ← Frame rate conversion
        "video/x-raw,framerate=30/1 ! "                  // ← Force 30fps output
        "videoconvert ! "
        "videoscale ! "
        "video/x-raw,format=BGR,"
            "width=" + std::to_string(SRC_W) + ","
            "height=" + std::to_string(SRC_H) + " ! "
        "appsink name=sink sync=false";
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    // ── GStreamer global init ────────────────────────────────────────────────
    gst_init(&argc, &argv);

    // ── Signal handling ──────────────────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── Configuration ────────────────────────────────────────────────────────
    const std::string video_path      = (argc > 1)
                                          ? argv[1]
                                          : "/home/slessing/Projects/AAUSAT6-C-/Untitled.mp4";
    const std::string reference_image = (argc > 2)
                                          ? argv[2]
                                          : "/home/slessing/Projects/AAUSAT6-C-/reference_object.jpg";

    std::cout << "Video source  : " << video_path      << "\n"
              << "Reference img : " << reference_image  << "\n";

    // ── Instantiate pipeline stages ──────────────────────────────────────────
    auto capture    = std::make_unique<GstreamerCapture>();
    auto detector   = std::make_unique<ORBDetector>();
    auto stabilizer = std::make_unique<StubStabilizer>();
    auto cropper    = std::make_unique<StubCropper>();

    // ── Init detection & stabilization ──────────────────────────────────────
    if (!detector->init("", "", reference_image)) {
        std::cerr << "Detector init failed.\n";
        return 1;
    }
    if (!stabilizer->init("", "")) {
        std::cerr << "Stabilizer init failed.\n";
        return 1;
    }

    // ── Start GStreamer capture ──────────────────────────────────────────────
    const std::string pipeline_str = build_pipeline(video_path);
    std::cout << "Pipeline: " << pipeline_str << "\n\n";

    if (!capture->start(pipeline_str)) {
        std::cerr << "Failed to start GStreamer pipeline.\n";
        return 1;
    }

    // ── Frame loop ───────────────────────────────────────────────────────────
    std::size_t frame_count = 0;

    while (!g_shutdown.load()) {

        // 1. Pull raw 4K frame from the appsink (blocks until available or EOS).
        auto maybe_frame = capture->pull_frame();
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

        // 4. Crop to 1920×1080 centred on the detected object.
        CroppedFrame output = cropper->crop(stabilized);

        // 5. Display / hand off downstream.
        cv::imshow("Output 1920x1080", output.data);

        ++frame_count;
        if (frame_count % 30 == 0) {
            std::cout << "Processed " << frame_count << " frames  |  "
                      << "ROI: " << output.src_roi << "\n";
        }

        // Quit on 'q'.
        if (cv::waitKey(1) == 'q') {
            g_shutdown.store(true);
        }
    }

    // ── Cleanup ──────────────────────────────────────────────────────────────
    stabilizer->flush();
    capture->stop();
    cv::destroyAllWindows();

    std::cout << "Done. Total frames processed: " << frame_count << "\n";
    return 0;
}