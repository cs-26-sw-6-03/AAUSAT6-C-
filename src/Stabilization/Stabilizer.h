#pragma once

#include "interfaces.h"

class Stabilizer : public IVideoStabilizer {
public:
    bool init(const std::string&, const std::string&) override;

    StabilizedFrame stabilize(const RawFrame&       frame,
                              const DetectionResult& detection,
                              const std::vector<cv::KeyPoint>& keypoints) override;


    void flush() override;
};