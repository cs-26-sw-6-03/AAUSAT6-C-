#include "FeatureDetection/FastDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <vector>

bool FastDetector::init(const std::string& /*model_config*/, 
                        const std::string& /*model_weights*/, 
                        const std::string& /*reference_image*/) 
{
    // No model to load for this simple demo; always succeed
    std::cout << "[FastDetector] init()\n";
    return true;
}

DetectionResult FastDetector::detect(RawFrame& frame)
{
    // convert to grayscale
    cv::Mat gray;
    cv::cvtColor(frame.data, gray, cv::COLOR_BGR2GRAY);

    // run FAST
    auto fast = cv::FastFeatureDetector::create(
        20,    // threshold
        true,  // nonmaxSuppression
        cv::FastFeatureDetector::TYPE_9_16
    );

    std::vector<cv::KeyPoint> keypoints;
    fast->detect(gray, keypoints);

    DetectionResult result;
    if (keypoints.empty()) {
        result.valid = false;
        return result;
    }

    // Draw keypoints
    cv::Mat output;
    cv::drawKeypoints(gray, keypoints, output,
                  cv::Scalar(0, 255, 0),
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::imshow("FAST Features - GStreamer", output);
    cv::waitKey(1);

    std::cout << "Detected "
         << keypoints.size()
         << " keypoints" << std::endl;

    // compute average location as our "detection centre"
    cv::Point2f avg(0.f, 0.f);
    for (auto &kp : keypoints)
        avg += kp.pt;
    avg *= (1.f / keypoints.size());

    result.center     = avg;
    result.valid      = true;
    result.confidence = static_cast<float>(keypoints.size());
    return result;
}