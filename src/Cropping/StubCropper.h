#pragma once

#include "interfaces.h"

class StubCropper : public IFrameCropper {
public:
    cv::Rect compute_roi(cv::Point2f center,
                         int src_w, int src_h,
                         int out_w, int out_h) const override;

    CroppedFrame crop(const StabilizedFrame& frame,
                      int out_w, int out_h) override;
};