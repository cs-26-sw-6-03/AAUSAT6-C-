#include "ObjectDetection/StubDetector.h"
#include <iostream>

bool StubDetector::init(const std::string&, const std::string&,
                        const std::string&)
{
    std::cout << "[StubDetector] init() — returning frame centre.\n";
    return true;
}

DetectionResult StubDetector::detect(const RawFrame& frame)
{
    // Always report the geometric centre of the source frame.
    DetectionResult r;
    r.center     = { static_cast<float>(frame.data.cols) / 2.f,
                     static_cast<float>(frame.data.rows) / 2.f };
    r.confidence = 1.f;
    r.valid      = true;
    return r;
}