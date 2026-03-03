#pragma once

#include "interfaces.h"
#include <opencv2/features2d.hpp>
#include <deque>


class Stabilizer : public IVideoStabilizer
{
public:
    bool init(const std::string &, const std::string &) override;

    StabilizedFrame stabilize(const RawFrame &frame,
                              const DetectionResult &detection) override;

    void flush() override;

    cv::Mat fixBorder(const cv::Mat &frame);

private:
    cv::Mat prevGray;
    cv::Mat smoothedTransform = cv::Mat::eye(2, 3, CV_64F);
    double alpha = 0.9; // If we need better stabilization then lower this number. (when lowering the number this latentcy is getting worse)

    cv::Mat prev_gray_;
    std::vector<cv::KeyPoint> prev_kps_;
    cv::Mat prev_desc_;

    cv::Ptr<cv::ORB> orb_detector_;  // ORB detector for keypoint detection

    double smoothed_dx = 0.0;
    double smoothed_dy = 0.0;
    double smoothed_da = 0.0;

    void get_features(RawFrame&                  frame,
                      const cv::Mat&             gray,
                      std::vector<cv::KeyPoint>& kps,
                      cv::Mat&                   desc) const;
};