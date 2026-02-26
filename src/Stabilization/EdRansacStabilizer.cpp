#include "EdRansacStabilizer.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────

bool EDRansacStabilizer::init(const std::string& /*model_config*/,
                               const std::string& /*model_weights*/)
{
    // Only create our own ORB model if no shared one has been provided.
    // If set_orb_model() is called after init(), own_orb_ is simply unused.
    if (!sharedorb_) {
        ownedorb_ = cv::ORB::create(params_.orb_n_features);
        std::cout << "[EDRansacStabilizer] No shared ORB model — created own ("
                  << params_.orb_n_features << " features).\n";
    } else {
        std::cout << "[EDRansacStabilizer] Using shared ORB model from ORBDetector.\n";
    }

    // ORB uses binary descriptors → Hamming distance
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);

    frame_idx_ = 0;
    trajectory_.clear();
    prev_gray_.release();
    prev_kps_.clear();
    prev_desc_.release();

    std::cout << "[EDRansacStabilizer] Initialized.\n"
              << "  Lowe ratio      : " << params_.lowe_ratio            << "\n"
              << "  RANSAC thresh   : " << params_.ransac_reproj_thresh  << " px\n"
              << "  ED threshold    : " << params_.ed_threshold          << " px\n"
              << "  Smooth radius   : " << params_.smooth_radius         << " frames\n";

    return true;
}

void EDRansacStabilizer::get_features(RawFrame&                  frame,
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

    // Compute features ourselves using whichever ORB model is active
    active_orb()->detectAndCompute(gray, cv::noArray(), kps, desc);

    // Cache so that any later pipeline stage can also reuse them
    frame.keypoints          = kps;
    frame.descriptors        = desc;
    frame.features_computed  = true;
}

StabilizedFrame EDRansacStabilizer::stabilize(const RawFrame&        frame,
                                               const DetectionResult& detection)
{
    // stabilize() receives a const ref from the interface but we need to write
    // back cached features. We cast away const here intentionally — the frame
    // data itself (pixels, pts) is never modified, only the feature cache.
    RawFrame& mutable_frame = const_cast<RawFrame&>(frame);

    StabilizedFrame out;
    out.pts_ns           = frame.pts_ns;
    out.suggested_center = detection.valid
        ? detection.center
        : cv::Point2f(frame.data.cols / 2.f, frame.data.rows / 2.f);

    // ── Grayscale conversion ─────────────────────────────────────────────────
    cv::Mat gray;
    cv::cvtColor(frame.data, gray, cv::COLOR_BGR2GRAY);

    // ── Extract / reuse features for current frame ───────────────────────────
    std::vector<cv::KeyPoint> curr_kps;
    cv::Mat curr_desc;
    get_features(mutable_frame, gray, curr_kps, curr_desc);

    // ── First frame: store state and pass through unchanged ──────────────────
    if (frame_idx_ == 0 || prev_gray_.empty()) {
        trajectory_.push_back(cv::Mat::eye(3, 3, CV_64F));
        prev_gray_ = gray;
        prev_kps_  = curr_kps;
        prev_desc_ = curr_desc;
        out.data   = frame.data.clone();
        ++frame_idx_;
        return out;
    }

    // ── Match previous → current ─────────────────────────────────────────────
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(prev_desc_, curr_desc, knn_matches, 2);

    // Lowe's ratio test
    std::vector<cv::Point2f> pts_prev, pts_curr;
    for (const auto& m : knn_matches) {
        if (m.size() < 2) continue;
        if (m[0].distance < params_.lowe_ratio * m[1].distance) {
            pts_prev.push_back(prev_kps_[m[0].queryIdx].pt);
            pts_curr.push_back(curr_kps[m[0].trainIdx].pt);
        }
    }

    // ── ED-RANSAC homography ─────────────────────────────────────────────────
    cv::Mat H_inter = cv::Mat::eye(3, 3, CV_64F);  // fallback: identity (no warp)

    if ((int)pts_prev.size() >= params_.min_inliers) {
        cv::Mat H = ed_ransac(pts_prev, pts_curr);
        if (!H.empty()) {
            H_inter = H;
        } else {
            std::cerr << "[EDRansacStabilizer] ED-RANSAC failed at frame "
                      << frame_idx_ << " — using identity.\n";
        }
    } else {
        std::cerr << "[EDRansacStabilizer] Too few matches ("
                  << pts_prev.size() << ") at frame "
                  << frame_idx_ << " — using identity.\n";
    }

    // ── Accumulate trajectory ────────────────────────────────────────────────
    // T[i] = H_inter(i-1→i) * T[i-1]   gives us frame 0 → frame i
    cv::Mat T_curr = H_inter * trajectory_.back();
    trajectory_.push_back(T_curr.clone());

    // ── Smoothed trajectory ──────────────────────────────────────────────────
    cv::Mat T_smooth = smooth_transform(frame_idx_);

    // ── Correction warp: what we need to apply to the raw frame ─────────────
    // warp = T_smooth * T_curr⁻¹
    cv::Mat warp = T_smooth * T_curr.inv();

    // ── Apply warp ───────────────────────────────────────────────────────────
    cv::Mat stabilized;
    cv::warpPerspective(frame.data, stabilized, warp,
                        frame.data.size(),
                        cv::INTER_LINEAR,
                        cv::BORDER_REPLICATE);

    // ── Transform suggested center through warp ──────────────────────────────
    if (detection.valid) {
        std::vector<cv::Point2f> center_in  = { detection.center };
        std::vector<cv::Point2f> center_out;
        cv::perspectiveTransform(center_in, center_out, warp);

        float cx = std::max(0.f, std::min(center_out[0].x, (float)(frame.data.cols - 1)));
        float cy = std::max(0.f, std::min(center_out[0].y, (float)(frame.data.rows - 1)));
        out.suggested_center = { cx, cy };
    }

    out.data = stabilized;

    // ── Update previous-frame state ──────────────────────────────────────────
    // Note: we store the raw (un-warped) keypoints because the next frame's
    // inter-frame registration is against the raw previous frame, not the
    // stabilized version. The warp is applied only to pixels for output.
    prev_gray_ = gray;
    prev_kps_  = curr_kps;
    prev_desc_ = curr_desc;

    if (frame_idx_ % 30 == 0) {
        std::cout << "[EDRansacStabilizer] Frame " << frame_idx_
                  << " | raw matches: " << pts_prev.size()
                  << " | cache hit: " << (frame.features_computed ? "yes" : "no")
                  << "\n";
    }

    ++frame_idx_;
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// ed_ransac
//
// Pass 1: standard RANSAC homography → initial inlier set
// Pass 2: project inliers through H, discard any with ED > ed_threshold
// Final:  least-squares re-estimation on the clean inlier set
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat EDRansacStabilizer::ed_ransac(const std::vector<cv::Point2f>& pts_prev,
                                       const std::vector<cv::Point2f>& pts_curr) const
{
    if ((int)pts_prev.size() < params_.min_inliers) return {};

    // ── Pass 1: RANSAC ────────────────────────────────────────────────────────
    cv::Mat inlier_mask;
    cv::Mat H = cv::findHomography(pts_prev, pts_curr,
                                   cv::RANSAC,
                                   params_.ransac_reproj_thresh,
                                   inlier_mask);
    if (H.empty()) return {};

    // Collect RANSAC inliers
    std::vector<cv::Point2f> inl_prev, inl_curr;
    for (int i = 0; i < (int)pts_prev.size(); ++i) {
        if (inlier_mask.at<uchar>(i)) {
            inl_prev.push_back(pts_prev[i]);
            inl_curr.push_back(pts_curr[i]);
        }
    }
    if ((int)inl_prev.size() < params_.min_inliers) return {};

    // ── Pass 2: Euclidean distance filter ────────────────────────────────────
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(inl_prev, projected, H);

    std::vector<cv::Point2f> ed_prev, ed_curr;
    for (int i = 0; i < (int)inl_prev.size(); ++i) {
        float dx = projected[i].x - inl_curr[i].x;
        float dy = projected[i].y - inl_curr[i].y;
        if (std::sqrt(dx * dx + dy * dy) < params_.ed_threshold) {
            ed_prev.push_back(inl_prev[i]);
            ed_curr.push_back(inl_curr[i]);
        }
    }
    if ((int)ed_prev.size() < params_.min_inliers) return {};

    // ── Final: least-squares re-estimation on clean set ──────────────────────
    return cv::findHomography(ed_prev, ed_curr, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// smooth_transform
//
// Causal trailing-window average over the last smooth_radius trajectory entries.
// Averaging 3×3 matrix entries directly is an approximation, but accurate
// enough for the small inter-frame motions typical in satellite video.
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat EDRansacStabilizer::smooth_transform(std::size_t idx) const
{
    int from  = std::max(0, (int)idx - params_.smooth_radius);
    int to    = (int)idx;

    cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
    int count = 0;
    for (int i = from; i <= to && i < (int)trajectory_.size(); ++i) {
        sum += trajectory_[i];
        ++count;
    }

    return count > 0 ? sum / (double)count : cv::Mat::eye(3, 3, CV_64F);
}