#include "interfaces.h"
#include <csignal>
#include <iostream>
#include <memory>
#include <string>

class StubDetector : public IObjectDetector {
public:
    bool init(const std::string&, const std::string&,
              const std::string&) override
    {
        std::cout << "[StubDetector] init() — returning frame centre.\n";
        return true;
    }

    DetectionResult detect(const RawFrame& frame) override
    {
        // Always report the geometric centre of the source frame.
        DetectionResult r;
        r.center     = { static_cast<float>(frame.data.cols) / 2.f,
                         static_cast<float>(frame.data.rows) / 2.f };
        r.confidence = 1.f;
        r.valid      = true;
        return r;
    }
};