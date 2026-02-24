#include "ObjectDetection/ORBDetector.h"
#include <iostream>
#include <vector>
using namespace std;

bool ORBDetector::init(const std::string&, const std::string&,
                       const std::string& reference_image)
{
    std::cout << "[ORBDetector] init()\n";
    ModelORB = ORB::create(500);
    reference_image_path = reference_image;

    // Load and compute reference descriptors ONCE here, not every frame
    Mat objectMat = cv::imread(reference_image_path);
    if (objectMat.empty()) {
        std::cerr << "[ORBDetector] Failed to load reference image: " << reference_image_path << "\n";
        return false;
    }
    cvtColor(objectMat, objectMatGray, COLOR_BGR2GRAY);
    ModelORB->detectAndCompute(objectMatGray, Mat(), keypointsObject, descriptorsObject);

    if (keypointsObject.empty()) {
        std::cerr << "[ORBDetector] No keypoints found in reference image.\n";
        return false;
    }

    referenceSize = objectMat.size();
    return true;
}

DetectionResult ORBDetector::detect(const RawFrame& frame)
{
    DetectionResult r;
    r.valid = false;

    Mat gray_frame;
    cvtColor(frame.data, gray_frame, COLOR_BGR2GRAY);

    vector<KeyPoint> keypointsFrame;
    Mat descriptorsFrame;
    ModelORB->detectAndCompute(gray_frame, Mat(), keypointsFrame, descriptorsFrame);

    if (descriptorsFrame.empty() || keypointsObject.empty()) return r;

    // Match frame descriptors against the pre-computed reference descriptors
    BFMatcher bruteforceMatcher(cv::NORM_HAMMING, true);
    vector<DMatch> matches;
    bruteforceMatcher.match(descriptorsFrame, descriptorsObject, matches);

    if (matches.empty()) return r;

    // Filter by absolute Hamming distance threshold (more reliable than relative formula)
    // ORB distances range 0-256; good matches are typically < 60-80
    const float DISTANCE_THRESHOLD = 60.f;
    const int MIN_GOOD_MATCHES = 8; // Require a meaningful number of inliers

    vector<DMatch> goodMatches;
    for (const auto& m : matches)
        if (m.distance < DISTANCE_THRESHOLD)
            goodMatches.push_back(m);

    if ((int)goodMatches.size() < MIN_GOOD_MATCHES) return r;

    // --- Geometric verification with homography ---
    // This is the key step that eliminates false positives.
    // A valid detection should have keypoints consistent with a planar transform.
    vector<Point2f> ptsFrame, ptsObject;
    for (const auto& m : goodMatches) {
        ptsFrame.push_back(keypointsFrame[m.queryIdx].pt);
        ptsObject.push_back(keypointsObject[m.trainIdx].pt);
    }

    Mat inlierMask;
    Mat H = findHomography(ptsObject, ptsFrame, RANSAC, 3.0, inlierMask);

    if (H.empty()) return r;

    // Count inliers and reject if too few survive RANSAC
    int inlierCount = countNonZero(inlierMask);
    if (inlierCount < MIN_GOOD_MATCHES) return r;

    // Project the center of the reference image through the homography
    // This gives a stable, geometry-consistent center rather than a keypoint average
    Point2f refCenter(referenceSize.width / 2.f, referenceSize.height / 2.f);
    vector<Point2f> refPts = { refCenter };
    vector<Point2f> projectedPts;
    perspectiveTransform(refPts, projectedPts, H);

    // Sanity check: projected center should be within the frame bounds
    Point2f detectedCenter = projectedPts[0];
    if (detectedCenter.x < 0 || detectedCenter.y < 0 ||
        detectedCenter.x >= gray_frame.cols ||
        detectedCenter.y >= gray_frame.rows)
        return r;

    // Smooth the center over time to reduce frame-to-frame jitter
    const float ALPHA = 0.4f; // lower = smoother but more lag
    if (!lastValid) {
        smoothedCenter = detectedCenter;
        lastValid = true;
    } else {
        smoothedCenter = ALPHA * detectedCenter + (1.f - ALPHA) * smoothedCenter;
    }

    // Confidence based on inlier ratio
    r.center     = smoothedCenter;
    r.confidence = (float)inlierCount / (float)goodMatches.size();
    r.valid      = true;
    return r;
}

std::vector<cv::KeyPoint> ORBDetector::detectKeypoints(const RawFrame& frame)
{
    std::vector<cv::KeyPoint> keypoints;

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(frame.data, gray, cv::COLOR_BGR2GRAY);

    // Detect keypoints
    ModelORB->detect(gray, keypoints);

    return keypoints;
}