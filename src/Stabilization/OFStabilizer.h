#pragma once

#include "interfaces.h"
#include <opencv2/features2d.hpp>
#include <deque>


class OFStabilizer : public IVideoStabilizer
{
public:
    bool init(const std::string &, const std::string &) override;

    StabilizedFrame stabilize(const RawFrame &frame,
                              const DetectionResult &detection) override;

    void flush() override;
    
    // Allow setting a shared ORB model (e.g., from ORBDetector)
    void set_orb_model(cv::Ptr<cv::ORB> orb) { sharedorb_ = orb; }

private:
    static constexpr int orb_n_features = 300;  // Reduced from 1000 for faster matching
    
    cv::Ptr<cv::ORB>        sharedorb_;      // May be set by ORBDetector
    cv::Ptr<cv::ORB>        ownedorb_;       // Create our own if no shared model
    cv::Ptr<cv::BFMatcher>  matcher_;

    cv::Mat prevGray;
    cv::Mat smoothedTransform = cv::Mat::eye(2, 3, CV_64F);
    double alpha = 0.9; // If we need better stabilization then lower this number. (when lowering the number this latentcy is getting worse)

    cv::Mat prev_gray_;
    std::vector<cv::KeyPoint> prev_kps_;
    cv::Mat prev_desc_;
    std::vector<cv::Point2f> prev_pts_;

    double smoothed_dx = 0.0;
    double smoothed_dy = 0.0;
    double smoothed_da = 0.0;

    double traj_dx;
    double traj_dy;
    double traj_da; 

    double diff_dx; 
    double diff_dy; 
    double diff_da;
    
    size_t frame_idx_ = 0;

    // Get the active ORB detector (shared or owned)
    cv::Ptr<cv::ORB> active_orb() const {
        return sharedorb_ ? sharedorb_ : ownedorb_;
    }

    void get_features(RawFrame&                  frame,
                      const cv::Mat&             gray,
                      std::vector<cv::KeyPoint>& kps,
                      cv::Mat&                   desc) const;
};