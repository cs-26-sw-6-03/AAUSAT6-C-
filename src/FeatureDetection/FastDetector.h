#pragma once

#include "interfaces.h"
#include <opencv2/features2d.hpp>

// Simple FAST-based detector that satisfies the IObjectDetector
// interface.  It doesn't use any DNN model; the "reference image"
// argument is ignored.  The implementation lives in FastDetector.cpp.

class FastDetector : public IFeatureDetector {
public:

    bool init(const std::string& model_config,
              const std::string& model_weights,
              const std::string& reference_image) override;

    DetectionResult detect(const RawFrame& frame)override;
};
