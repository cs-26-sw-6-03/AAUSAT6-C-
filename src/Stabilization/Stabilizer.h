#pragma once

#include "interfaces.h"

class Stabilizer : public IVideoStabilizer {
public:
    bool init(const std::string&, const std::string&) override;

    StabilizedFrame stabilize(const RawFrame&       frame,
                              const DetectionResult& detection) override;

    void flush() override;

private:
        cv::Mat prevGray;
        cv::Mat smoothedTransform = cv::Mat::eye(2, 3, CV_64F);
        double alpha = 0.9; 
};