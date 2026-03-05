#include "Stabilization/Stabilizer.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

bool Stabilizer::init(const std::string &, const std::string &)
{
    if (!sharedorb_)
    {
        ownedorb_ = cv::ORB::create(orb_n_features);
        std::cout << "[Stabilizer] No shared ORB model — created own ("
                  << orb_n_features << " features).\n";
    }
    else
    {
        std::cout << "[Stabilizer] Using shared ORB model from ORBDetector.\n";
    }

    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);

    frame_idx_ = 0;
    prev_gray_.release();
    prev_kps_.clear();
    prev_desc_.release();

    std::cout << "[Stabilizer] Initialized with alpha=" << alpha << ".\n";
    return true;
}

void Stabilizer::get_features(RawFrame &frame,
                              const cv::Mat &gray,
                              std::vector<cv::KeyPoint> &kps,
                              cv::Mat &desc) const
{
    if (frame.features_computed)
    {
        // ORBDetector already ran on this frame — reuse its results
        kps = frame.keypoints;
        desc = frame.descriptors;
        return;
    }

    // Compute features ourselves using whichever ORB model is active
    active_orb()->detectAndCompute(gray, cv::noArray(), kps, desc);

    // Cache so that any later pipeline stage can also reuse them
    frame.keypoints = kps;
    frame.descriptors = desc;
    frame.features_computed = true;
}

StabilizedFrame Stabilizer::stabilize(const RawFrame &frame,
                                      const DetectionResult &detection)
{
    // Cast away const to cache features on this frame
    RawFrame &mutable_frame = const_cast<RawFrame &>(frame);

    StabilizedFrame out;
    out.pts_ns = frame.pts_ns;
    out.suggested_center = detection.valid
                               ? detection.center
                               : cv::Point2f(frame.data.cols / 2.f, frame.data.rows / 2.f);

    cv::Mat frameMat = frame.data;

    cv::Mat gray;
    cv::cvtColor(frameMat, gray, cv::COLOR_BGR2GRAY);

    if (prevGray.empty())
    {
        std::vector<cv::KeyPoint> curr_kps;
        cv::Mat curr_desc;
        get_features(mutable_frame, gray, curr_kps, curr_desc);

        prev_gray_ = gray.clone();
        prev_kps_ = curr_kps;
        prev_desc_ = curr_desc;
        prevGray = gray.clone();

        StabilizedFrame result;
        result.data = frameMat;
        ++frame_idx_;
        return result;
    }

    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(
        prev_gray_,
        gray,
        prev_pts_,
        curr_pts,
        status,
        err);

    std::vector<cv::Point2f> prevFiltered;
    std::vector<cv::Point2f> currFiltered;

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            prevFiltered.push_back(prev_pts_[i]);
            currFiltered.push_back(curr_pts[i]);
        }
    }

    if (prevFiltered.size() < 10)
    {
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;

        get_features(mutable_frame, gray, kps, desc);

        prev_pts_.clear();
        for (const auto &kp : kps)
            prev_pts_.push_back(kp.pt);

        prev_gray_ = gray.clone();

        StabilizedFrame result;
        result.data = frameMat;

        ++frame_idx_;
        return result;
    }

    cv::Mat T = cv::estimateAffinePartial2D(prevFiltered, currFiltered); // denne extractor et 2 x 3 matrix.

    if (T.empty())
    {
        prev_pts_ = currFiltered;
        prev_gray_ = gray.clone();

        StabilizedFrame result;
        result.data = frameMat;

        ++frame_idx_;
        return result;
    }
    // her kigger vi på Horizontal og Vertical i vores metrix
    double dx = T.at<double>(0, 2);
    double dy = T.at<double>(1, 2);
    double da = std::atan2(T.at<double>(1, 0),
                           T.at<double>(0, 0));

    smoothed_dx = alpha * smoothed_dx + (1.0 - alpha) * dx;
    smoothed_dy = alpha * smoothed_dy + (1.0 - alpha) * dy;
    smoothed_da = alpha * smoothed_da + (1.0 - alpha) * da;

    cv::Mat smoothedT = cv::Mat::eye(2, 3, CV_64F);

    smoothedT.at<double>(0, 0) = std::cos(smoothed_da);
    smoothedT.at<double>(0, 1) = -std::sin(smoothed_da);
    smoothedT.at<double>(1, 0) = std::sin(smoothed_da);
    smoothedT.at<double>(1, 1) = std::cos(smoothed_da);

    smoothedT.at<double>(0, 2) = smoothed_dx;
    smoothedT.at<double>(1, 2) = smoothed_dy;

    cv::Mat stabilized;
    cv::warpAffine(frameMat, stabilized, smoothedT, frameMat.size());

    // Convert 2x3 affine matrix to 3x3 for perspectiveTransform
    cv::Mat warp_3x3 = cv::Mat::eye(3, 3, CV_64F);
    smoothedT.copyTo(warp_3x3.rowRange(0, 2));

    if (detection.valid)
    {
        std::vector<cv::Point2f> center_in = {detection.center};
        std::vector<cv::Point2f> center_out;

        cv::perspectiveTransform(
            center_in,
            center_out,
            warp_3x3);

        float cx = std::max(
            0.f,
            std::min(center_out[0].x,
                     (float)(frame.data.cols - 1)));

        float cy = std::max(
            0.f,
            std::min(center_out[0].y,
                     (float)(frame.data.rows - 1)));

        out.suggested_center = {cx, cy};
    }

    prev_pts_ = currFiltered;
    prev_gray_ = gray.clone();

    if (frame_idx_ % 30 == 0)
    {
        std::cout << "[Stabilizer] Frame "
                  << frame_idx_
                  << " | tracked points: "
                  << prev_pts_.size()
                  << "\n";
    }

    ++frame_idx_;

    out.data = stabilized;
    return out;
}

void Stabilizer::flush() {}