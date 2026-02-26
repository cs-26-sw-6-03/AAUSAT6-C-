#pragma once

#include "interfaces.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

using namespace cv;
using namespace std;

class ORBDetector : public IFeatureDetector {
public:
    bool init(const std::string&, const std::string&,
              const std::string&) override;

    DetectionResult detect(const RawFrame& frame) override;
    Ptr<ORB> ModelORB;
    std::string reference_image_path;
    Mat objectMatGray;
    vector<KeyPoint> keypointsObject;
    Mat descriptorsObject;
    cv::Size referenceSize;
    Point2f smoothedCenter;
    bool lastValid = false;
};