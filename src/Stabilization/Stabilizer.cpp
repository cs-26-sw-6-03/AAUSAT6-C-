#include "Stabilization/Stabilizer.h"
#include <opencv2/calib3d.hpp>
#include <iostream>

bool Stabilizer::init(const std::string&, const std::string&)
{
    std::cout << "[Stabilizer] init() — pass-through mode.\n";
    return true;
}

StabilizedFrame Stabilizer::stabilize(const RawFrame& frame,
                                      const DetectionResult&)
{
    cv::Mat frameMat = frame.data; // adapt to your struct

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

    cv::Mat T = cv::estimateAffinePartial2D(prevFiltered, currFiltered);

    if (!T.empty())
    {
        smoothedTransform = alpha * smoothedTransform + (1 - alpha) * T;
    }

    cv::Mat stabilized;
    cv::warpAffine(frameMat, stabilized, smoothedTransform, frameMat.size());

    prevGray = gray.clone();

    StabilizedFrame result;
    result.data = stabilized;
    return result;
}

// std::vector<Trajectoy> cumsum (std::vector<T>)

void Stabilizer::flush() {}