#pragma once

#include "interfaces.h"

class Stabilizer : public IVideoStabilizer {
public:
    bool init(const std::string&, const std::string&) override;

    StabilizedFrame stabilize(const RawFrame&       frame,
                              const DetectionResult& detection) override;

    void flush() override;

    cv::Mat fixBorder(const cv::Mat& frame);

private:
        cv::Mat prevGray;
        cv::Mat smoothedTransform = cv::Mat::eye(2, 3, CV_64F);
        double alpha = 0.9; // If we need better stabilization then lower this number. (when lowering the number this latentcy is getting worse)

        double smoothed_dx = 0.0;
        double smoothed_dy = 0.0;
        double smoothed_da = 0.0;
};