#pragma once

#include "interfaces.h"

class StubStabilizer : public IVideoStabilizer {
public:
    bool init(const std::string&, const std::string&) override;

    StabilizedFrame stabilize(const RawFrame&       frame,
                              const DetectionResult& detection) override;

    void flush() override;
};