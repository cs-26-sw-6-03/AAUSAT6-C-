#include "Stabilization/StubStabilizer.h"
#include <iostream>

bool StubStabilizer::init(const std::string&, const std::string&)
{
    std::cout << "[StubStabilizer] init() — pass-through mode.\n";
    return true;
}

StabilizedFrame StubStabilizer::stabilize(const RawFrame&       frame,
                                          const DetectionResult& detection)
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

void StubStabilizer::flush() {}