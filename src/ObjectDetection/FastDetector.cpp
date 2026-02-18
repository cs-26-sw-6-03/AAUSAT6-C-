#include "ObjectDetection/FastDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <vector>

bool FastDetector::init(const std::string& /*model_config*/, 
                        const std::string& /*model_weights*/, 
                        const std::string& reference_image) 
{
    // No model to load for this simple demo; always succeed
    std::cout << "[FastDetector] init()\n";
    reference_image_path = reference_image;
    return true;
}

DetectionResult FastDetector::detect(const RawFrame& frame)
{

    cv::Mat imageMat = cv::imread(reference_image_path);
    cv::Mat referenceGray;
    cv::cvtColor(image, referenceGray, cv::COLOR_BGR2GRAY);
    
    cv::Mat gray;
    cv::cvtColor(frame.data, gray, cv::COLOR_BGR2GRAY);
    

    
    auto fast = cv::FastFeatureDetector::create(
        20,    
        true,  
        cv::FastFeatureDetector::TYPE_9_16
    );

    orb = cv.ORB_create()

    std::vector<cv::KeyPoint> keypoints;
    fast->detect(gray, keypoints);

    DetectionResult result;
    if (keypoints.empty()) {
        result.valid = false;
        return result;
    }

    // Draw keypoints
    cv::Mat output;
    drawKeypoints(frame.data, keypoints, output,
                  cv::Scalar(0, 255, 0),
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::imshow("FAST Features - GStreamer", output);
    cv::waitKey(1);

    std::cout << "Detected "
         << keypoints.size()
         << " keypoints" << std::endl;

    
    cv::Point2f avg(0.f, 0.f);
    for (auto &kp : keypoints)
        avg += kp.pt;
    avg *= (1.f / keypoints.size());

    result.center     = avg;
    result.valid      = true;
    result.confidence = static_cast<float>(keypoints.size());
    return result;
}
