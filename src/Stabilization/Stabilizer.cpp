#include "Stabilization/Stabilizer.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

bool Stabilizer::init(const std::string &, const std::string &)
{
    std::cout << "[Stabilizer] init() — initializing ORB feature detector.\n";
    // Create ORB detector with 500 keypoints
    orb_detector_ = cv::ORB::create(500);
    return true;
}

void Stabilizer::get_features(RawFrame&                  frame,
                                       const cv::Mat&             gray,
                                       std::vector<cv::KeyPoint>& kps,
                                       cv::Mat&                   desc) const
{
    if (frame.features_computed) {
        // ORBDetector already ran on this frame — reuse its results
        kps  = frame.keypoints;
        desc = frame.descriptors;
        return;
    }

    // Compute features using ORB detector
    orb_detector_->detectAndCompute(gray, cv::noArray(), kps, desc);

    // Cache so that any later pipeline stage can also reuse them
    frame.keypoints          = kps;
    frame.descriptors        = desc;
    frame.features_computed  = true;
}

StabilizedFrame Stabilizer::stabilize(const RawFrame &frame,
                                      const DetectionResult &)
{

    cv::Mat frameMat = frame.data;

    cv::Mat gray;
    cv::cvtColor(frameMat, gray, cv::COLOR_BGR2GRAY);

    if (prevGray.empty())
    {
        prevGray = gray.clone();
        StabilizedFrame result;
        result.data = frameMat;
        return result;
    }

    // Detect keypoints and descriptors in both frames using ORB
    std::vector<cv::KeyPoint> prevKps, currKps;
    cv::Mat prevDesc, currDesc;
    
    orb_detector_->detectAndCompute(prevGray, cv::noArray(), prevKps, prevDesc);
    orb_detector_->detectAndCompute(gray, cv::noArray(), currKps, currDesc);

    if (prevDesc.empty() || currDesc.empty())
    {
        prevGray = gray.clone();
        StabilizedFrame result;
        result.data = frameMat;
        return result;
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(prevDesc, currDesc, matches);

    std::vector<cv::DMatch> goodMatches;
    const float DISTANCE_THRESHOLD = 60.f;
    for (const auto& match : matches)
    {
        if (match.distance < DISTANCE_THRESHOLD)
            goodMatches.push_back(match);
    }

    if (goodMatches.size() < 4)
    {
        prevGray = gray.clone();
        StabilizedFrame result;
        result.data = frameMat;
        return result;
    }

    // Extract matched points
    std::vector<cv::Point2f> prevPts, currPts;
    for (const auto& match : goodMatches)
    {
        prevPts.push_back(prevKps[match.queryIdx].pt);
        currPts.push_back(currKps[match.trainIdx].pt);
    }

    cv::Mat T = cv::estimateAffinePartial2D(prevPts, currPts); // denne extractor et 2 x 3 matrix.

    if (T.empty())
    {
        prevGray = gray.clone();
        StabilizedFrame result;
        result.data = frameMat;
        return result;
    }
    // her kigger vi på Horizontal og Vertical i vores metrix
    double dx = T.at<double>(0,2); 
    double dy = T.at<double>(1,2);
    double da = std::atan2(T.at<double>(1,0),
                            T.at<double>(0,0));

    smoothed_dx = alpha * smoothed_dx + (1.0 - alpha) * dx;
    smoothed_dy = alpha * smoothed_dy + (1.0 - alpha) * dy;
    smoothed_da = alpha * smoothed_da + (1.0 - alpha) * da;

    cv::Mat smoothedT = cv::Mat::eye(2, 3, CV_64F);

    smoothedT.at<double>(0,0) = std::cos(smoothed_da);
    smoothedT.at<double>(0,1) = -std::sin(smoothed_da);
    smoothedT.at<double>(1,0) = std::sin(smoothed_da);
    smoothedT.at<double>(1,1) = std::cos(smoothed_da);

    smoothedT.at<double>(0,2) = smoothed_dx;
    smoothedT.at<double>(1,2) = smoothed_dy;



    cv::Mat stabilized;
    cv::warpAffine(frameMat, stabilized, smoothedT, frameMat.size());

    prevGray = gray.clone();

    StabilizedFrame result;
    result.data = stabilized;
    return result;
}

cv::Mat Stabilizer::fixBorder(const cv::Mat& frame){
    double scale = 1.04; // Ved at ændre dette kan du ændre zoom - 1.04 = 4% zoom

    cv::Point2f center(frame.cols / 2.0f, frame.rows / 2.0f);
    cv::Mat T = cv::getRotationMatrix2D(center, 0, scale);

    cv::Mat scaled;
    cv::warpAffine(frame, scaled, T, frame.size());

    return scaled;
}

void Stabilizer::flush() {}