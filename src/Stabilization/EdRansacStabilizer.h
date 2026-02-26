#pragma once

#include "interfaces.h"
#include <opencv2/features2d.hpp>
#include <deque>

class EDRansacStabilizer : public IVideoStabilizer {
public:


    int    orb_n_features        = 2000;
    float  lowe_ratio            = 0.75f;
    double ransac_reproj_thresh  = 3.0;   // pixels
    float  ed_threshold          = 0.5f;  // pixels
    int    min_inliers           = 10;
    int    smooth_radius         = 15;    // trailing frames

    EDRansacStabilizer()  = default;
    ~EDRansacStabilizer() override = default;

    void set_orb_model(cv::Ptr<cv::ORB> sharedorb) { sharedorb_ = sharedorb; }

    bool init(const std::string& model_config  = "",
              const std::string& model_weights = "") override;

    StabilizedFrame stabilize(const RawFrame&        frame,
                              const DetectionResult& detection) override;

    void flush() override {}

private:
    //Could also use SIFT, BRISK or Fast, for fast we would also need a descriptor, but we do that in the detector, so that approach can be reused
    cv::Ptr<cv::ORB>     sharedorb_;
    cv::Ptr<cv::ORB>     ownedorb_;
    cv::Ptr<cv::BFMatcher> matcher_;

    // Previous-frame data (for adjacent-frame registration)
    cv::Mat prev_gray_;
    std::vector<cv::KeyPoint> prev_kps_;
    cv::Mat                   prev_desc_;

    std::vector<cv::Mat> trajectory_;

    
    //Helpers

    cv::Ptr<cv::ORB> active_orb() const {
        return sharedorb_ ? sharedorb_ : ownedorb_;
    }

    void get_features(RawFrame&                  frame,
                      const cv::Mat&             gray,
                      std::vector<cv::KeyPoint>& kps,
                      cv::Mat&                   desc) const;

    cv::Mat ed_ransac(const std::vector<cv::Point2f>& pts_prev,
                      const std::vector<cv::Point2f>& pts_curr) const;

    cv::Mat smooth_transform(std::size_t idx) const;

    bool initialized_ = false;
    std::size_t frame_idx_ = 0;
};