#pragma once

#include "interfaces.h"

class StubDetector : public IFeatureDetector {
public:
    bool init(const std::string&, const std::string&,
              const std::string&) override;

    DetectionResult detect(RawFrame& frame) override;
};