#include "interfaces.h"
#include <iostream>

class StubStabilizer : public IVideoStabilizer {
public:
    bool init(const std::string&, const std::string&) override
    {
        std::cout << "[StubStabilizer] init() — pass-through mode.\n";
        return true;
    }

    StabilizedFrame stabilize(const RawFrame&       frame,
                              const DetectionResult& detection) override
    {
        // No-op: forward the frame and centre unchanged.
        StabilizedFrame sf;
        sf.data             = frame.data;        // zero-copy (same Mat header)
        sf.suggested_center = detection.center;
        sf.pts_ns           = frame.pts_ns;
        return sf;
    }

    void flush() override {}
};