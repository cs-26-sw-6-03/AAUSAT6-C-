#include "Stabilization/Stabilizer.h"
#include <opencv2/calib3d.hpp>
#include <iostream>

bool Stabilizer::init(const std::string &, const std::string &)
{
    std::cout << "[Stabilizer] init() — pass-through mode.\n";
    return true;
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

    std::vector<cv::Point2f> prevPts, currPts;
    cv::goodFeaturesToTrack(prevGray, prevPts, 200, 0.01, 30);

    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prevGray, gray, prevPts, currPts, status, err);

    std::vector<cv::Point2f> prevFiltered, currFiltered;

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            prevFiltered.push_back(prevPts[i]);
            currFiltered.push_back(currPts[i]);
        }
    }

    cv::Mat T = cv::estimateAffinePartial2D(prevFiltered, currFiltered); // denne extractor et 2 x 3 matrix.

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