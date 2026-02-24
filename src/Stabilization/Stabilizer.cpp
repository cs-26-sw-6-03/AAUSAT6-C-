#include "Stabilization/Stabilizer.h"
#include <iostream>

bool Stabilizer::init(const std::string&, const std::string&)
{
    std::cout << "[Stabilizer] init() — pass-through mode.\n";
    return true;
}

StabilizedFrame Stabilizer::stabilize(const RawFrame&       frame,
                                      const DetectionResult& detection,
                                      const std::vector<cv::KeyPoint>& keypoints)
{
    // No-op: forward the frame and centre unchanged.
    StabilizedFrame sf;
    
    sf.data             = frame.data;        // zero-copy (same Mat header)
    if(detection.valid){
        sf.suggested_center = detection.center;
    } else {
        sf.suggested_center = { static_cast<float>(frame.data.cols) / 2.f,
                     static_cast<float>(frame.data.rows) / 2.f };
    }
    sf.pts_ns           = frame.pts_ns;
    return sf;
}

void Stabilizer::flush() {}