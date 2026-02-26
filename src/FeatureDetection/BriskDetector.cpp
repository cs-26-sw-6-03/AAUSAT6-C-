#include "FeatureDetection/BriskDetector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

BriskDetector::BriskDetector()
    // thresh = threshold (default 30, higher = fewer keypoints = faster)
    // octaves = scale levels (default 3, lower = fewer scales = faster)
    : brisk_(cv::BRISK::create(60, 3))  
    , matcher_(cv::BFMatcher::create(cv::NORM_HAMMING, false))  // crossCheck=false for kNN
    //, matcher_(cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2)))
{
}

bool BriskDetector::init(const std::string& /*model_config*/,
                         const std::string& /*model_weights*/,
                         const std::string& reference_image)
{
    std::cout << "[BriskDetector] Initializing with reference image: " 
              << reference_image << std::endl;
    
    // Load the reference image
    reference_image_ = cv::imread(reference_image, cv::IMREAD_COLOR);
    if (reference_image_.empty()) {
        std::cerr << "[BriskDetector] ERROR: Could not load reference image: " 
                  << reference_image << std::endl;
        return false;
    }
    
    // Convert to grayscale
    cv::cvtColor(reference_image_, reference_gray_, cv::COLOR_BGR2GRAY);
    
    // Detect keypoints and compute descriptors for reference image
    brisk_->detectAndCompute(reference_gray_, cv::noArray(),
                            reference_keypoints_, reference_descriptors_);
    
    if (reference_keypoints_.empty()) {
        std::cerr << "[BriskDetector] ERROR: No keypoints found in reference image" 
                  << std::endl;
        return false;
    }
    
    std::cout << "[BriskDetector] Found " << reference_keypoints_.size() 
              << " keypoints in reference image" << std::endl;
    std::cout << "[BriskDetector] Descriptor size: " 
              << reference_descriptors_.size() << std::endl;
    std::cout << "[BriskDetector] Initialization complete" << std::endl;
    
    return true;
}

DetectionResult BriskDetector::detect(const RawFrame& frame)
{
    DetectionResult result;
    result.valid = false;
    result.confidence = 0.0f;
    
    if (frame.data.empty()) {
        std::cerr << "[BriskDetector] ERROR: Frame data is EMPTY" << std::endl;
        return result;
    }
    
    if (reference_descriptors_.empty()) {
        std::cerr << "[BriskDetector] ERROR: Reference descriptors are EMPTY" << std::endl;
        return result;
    }
    
    // Convert frame to grayscale
    cv::Mat frame_gray;
    if (frame.data.channels() == 3) {
        cv::cvtColor(frame.data, frame_gray, cv::COLOR_BGR2GRAY);
    } else {
        frame_gray = frame.data;
    }
    
    // Resize for speed: toggle to enable/disable
    constexpr bool USE_RESIZE = false;     // Set to false to disable resize
    constexpr float SCALE = 0.5f;         // Resize scale (0.5 = 4x faster)
    
    cv::Mat detection_frame;
    std::vector<cv::KeyPoint> frame_keypoints;
    cv::Mat frame_descriptors;
    
    if (USE_RESIZE) {
        // Detect on resized image
        cv::resize(frame_gray, detection_frame, cv::Size(), SCALE, SCALE, cv::INTER_LINEAR);
        brisk_->detectAndCompute(detection_frame, cv::noArray(),
                                frame_keypoints, frame_descriptors);
        
        // Scale keypoint coordinates back to original resolution
        for (auto& kp : frame_keypoints) {
            kp.pt.x /= SCALE;
            kp.pt.y /= SCALE;
            kp.size /= SCALE;
        }
    } else {
        // Detect on full resolution
        brisk_->detectAndCompute(frame_gray, cv::noArray(),
                                frame_keypoints, frame_descriptors);
    }
    
    if (frame_keypoints.empty() || frame_descriptors.empty()) {
        // No keypoints found in frame
        return result;
    }
    
    // Match descriptors using KNN (k=2 for Lowe's ratio test)
    std::vector<std::vector<cv::DMatch>> knn_matches;
    try {
        matcher_->knnMatch(reference_descriptors_, frame_descriptors, 
                          knn_matches, 2);
    } catch (const cv::Exception& e) {
        std::cerr << "[BriskDetector] ERROR: Matching failed: " << e.what() << std::endl;
        return result;
    }
    
    // Apply Lowe's ratio test to filter good matches
    std::vector<cv::DMatch> good_matches = filter_matches(knn_matches);
    
    // Check if we have enough good matches
    if (good_matches.size() < static_cast<size_t>(min_good_matches_)) {
        // Not enough matches for reliable detection
        return result;
    }
    
    // Compute center of matched region in the frame
    result.center = compute_center(frame_keypoints, good_matches);
    result.confidence = compute_confidence(good_matches, reference_keypoints_.size());
    result.valid = true;
    
    // Debug output
    static size_t frame_count = 0;
    if (frame_count++ % 30 == 0) {  // Print every 30 frames
        std::cout << "[BriskDetector] Frame keypoints: " << frame_keypoints.size()
                  << ", Good matches: " << good_matches.size()
                  << ", Center: (" << result.center.x << ", " << result.center.y << ")"
                  << ", Confidence: " << result.confidence << std::endl;
    }
    
    return result;
}

void BriskDetector::warmup()
{
    std::cout << "[BriskDetector] Warmup - Creating dummy frame for processing" 
              << std::endl;
    
    // Create a dummy frame for warm-up using reference image dimensions
    RawFrame dummy_frame;
    if (!reference_image_.empty()) {
        dummy_frame.data = cv::Mat(reference_image_.rows, reference_image_.cols, 
                                   CV_8UC3, cv::Scalar(128, 128, 128));
    } else {
        // Fallback to a generic size if no reference loaded yet
        dummy_frame.data = cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    }
    dummy_frame.pts_ns = 0;
    
    // Run a detection to warm up
    detect(dummy_frame);
    
    std::cout << "[BriskDetector] Warmup complete" << std::endl;
}

std::vector<cv::DMatch> BriskDetector::filter_matches(
    const std::vector<std::vector<cv::DMatch>>& knn_matches) const
{
    std::vector<cv::DMatch> good_matches;
    
    for (const auto& match_pair : knn_matches) {
        // Lowe's ratio test requires at least 2 matches
        if (match_pair.size() < 2) {
            continue;
        }
        
        // If the best match is significantly better than the second best,
        // it's likely a good match
        if (match_pair[0].distance < ratio_threshold_ * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }
    
    return good_matches;
}

cv::Point2f BriskDetector::compute_center(
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<cv::DMatch>& good_matches) const
{
    // Extract locations of matched keypoints in the frame
    std::vector<cv::Point2f> matched_points;
    matched_points.reserve(good_matches.size());
    
    for (const auto& match : good_matches) {
        // trainIdx refers to the frame keypoints
        matched_points.push_back(keypoints[match.trainIdx].pt);
    }
    
    // Compute the mean position (centroid) of all matched points
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& pt : matched_points) {
        center.x += pt.x;
        center.y += pt.y;
    }
    
    if (!matched_points.empty()) {
        center.x /= matched_points.size();
        center.y /= matched_points.size();
    }
    
    return center;
}

float BriskDetector::compute_confidence(
    const std::vector<cv::DMatch>& good_matches,
    size_t total_reference_keypoints) const
{
    if (total_reference_keypoints == 0) {
        return 0.0f;
    }
    
    // Base confidence on the ratio of good matches to reference keypoints
    float match_ratio = static_cast<float>(good_matches.size()) / 
                        static_cast<float>(total_reference_keypoints);
    
    // Also consider the quality (distance) of matches
    float avg_distance = 0.0f;
    for (const auto& match : good_matches) {
        avg_distance += match.distance;
    }
    avg_distance /= good_matches.size();
    
    // Normalize distance (typical BRISK descriptor distances are in range 0-512)
    // Lower distance is better, so invert it
    float distance_score = 1.0f - std::min(avg_distance / 512.0f, 1.0f);
    
    // Combine both metrics (weighted average)
    float confidence = 0.6f * std::min(match_ratio, 1.0f) + 0.4f * distance_score;
    
    return std::min(confidence, 1.0f);
}
