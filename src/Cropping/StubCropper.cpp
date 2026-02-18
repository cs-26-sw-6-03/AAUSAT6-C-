#include "Cropping/StubCropper.h"

cv::Rect StubCropper::compute_roi(cv::Point2f center,
                                   int src_w, int src_h,
                                   int out_w, int out_h) const
{
    // Half-extents of the output window.
    int half_w = out_w / 2;
    int half_h = out_h / 2;

    // Ideal top-left corner.
    int x = static_cast<int>(center.x) - half_w;
    int y = static_cast<int>(center.y) - half_h;

    // Clamp so the rect stays inside [0, src_w) × [0, src_h).
    x = std::max(0, std::min(x, src_w - out_w));
    y = std::max(0, std::min(y, src_h - out_h));

    return { x, y, out_w, out_h };
}

CroppedFrame StubCropper::crop(const StabilizedFrame& frame)
{
    cv::Rect roi = compute_roi(
        frame.suggested_center,
        frame.data.cols, frame.data.rows,
        OUTPUT_W, OUTPUT_H);

    CroppedFrame cf;
    cf.data    = frame.data(roi).clone();
    cf.src_roi = roi;
    cf.pts_ns  = frame.pts_ns;
    return cf;
}