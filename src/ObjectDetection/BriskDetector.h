#pragma once

#include "interfaces.h"
#include <opencv2/features2d.hpp>

/**
 * @brief BRISK-based object detector using feature matching
 * 
 * This detector:
 * 1. Loads a reference image at initialization
 * 2. Detects BRISK keypoints and computes descriptors for both reference and incoming frames
 * 3. Matches features using BFMatcher with Hamming distance
 * 4. Returns the center of the matched object region
 */
class BriskDetector : public IObjectDetector {
public:
    BriskDetector();
    ~BriskDetector() override = default;

    /**
     * @brief Initialize the detector with a reference image
     * 
     * @param model_config   (unused - kept for interface compatibility)
     * @param model_weights  (unused - kept for interface compatibility)
     * @param reference_image Path to the reference object image
     * @return true if initialization succeeded
     */
    bool init(const std::string& model_config,
              const std::string& model_weights,
              const std::string& reference_image) override;

    /**
     * @brief Detect the object in the given frame
     * 
     * Uses BRISK feature matching to locate the reference object.
     * Returns the center position of the matched region.
     * 
     * @param frame The raw input frame
     * @return DetectionResult containing center point, confidence, and validity
     */
    DetectionResult detect(const RawFrame& frame) override;

    /**
     * @brief Warm up the detector (optional)
     */
    void warmup() override;

private:
    // BRISK feature detector and descriptor
    cv::Ptr<cv::BRISK> brisk_;
    
    // Reference image data
    cv::Mat reference_image_;
    cv::Mat reference_gray_;
    std::vector<cv::KeyPoint> reference_keypoints_;
    cv::Mat reference_descriptors_;
    
    // Feature matcher (using Hamming distance for binary descriptors)
    cv::Ptr<cv::BFMatcher> matcher_;
    
    // Detection parameters
    int min_good_matches_ = 10;      // Minimum matches required for valid detection
    float ratio_threshold_ = 0.75f;   // Lowe's ratio test threshold
    
    /**
     * @brief Apply Lowe's ratio test to filter good matches
     */
    std::vector<cv::DMatch> filter_matches(
        const std::vector<std::vector<cv::DMatch>>& knn_matches) const;
    
    /**
     * @brief Compute the center of matched keypoints
     */
    cv::Point2f compute_center(
        const std::vector<cv::KeyPoint>& keypoints,
        const std::vector<cv::DMatch>& good_matches) const;
    
    /**
     * @brief Compute confidence based on number and quality of matches
     */
    float compute_confidence(
        const std::vector<cv::DMatch>& good_matches,
        size_t total_reference_keypoints) const;
};
