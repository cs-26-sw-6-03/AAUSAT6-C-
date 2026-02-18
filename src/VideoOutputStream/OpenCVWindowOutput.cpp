#include "VideoOutputStream/OpenCVWindowOutput.h"

#include <opencv2/highgui.hpp>
#include <iostream>

bool OpenCVWindowOutput::init(const std::string& config)
{
    if (is_open_) {
        std::cerr << "[OpenCVWindowOutput] Already initialized.\n";
        return false;
    }

    // Config is the window name
    window_name_ = config.empty() ? "Output" : config;
    
    // Create the window
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    
    is_open_ = true;
    std::cout << "[OpenCVWindowOutput] Window created: " << window_name_ << "\n";
    
    return true;
}

bool OpenCVWindowOutput::write_frame(const CroppedFrame& frame)
{
    if (!is_open_) {
        return false;
    }

    if (frame.data.empty()) {
        std::cerr << "[OpenCVWindowOutput] Received empty frame.\n";
        return false;
    }

    cv::imshow(window_name_, frame.data);
    
    // Check if window was closed by user (waitKey returns -1 if window closed)
    // We use a short wait to keep the display responsive
    int key = cv::waitKey(1);
    
    // Check if user pressed 'q' to quit
    if (key == 'q' || key == 'Q') {
        is_open_ = false;
        return false;
    }
    
    // Check if the window still exists
    try {
        if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1) {
            is_open_ = false;
            return false;
        }
    } catch (...) {
        is_open_ = false;
        return false;
    }
    
    return true;
}

void OpenCVWindowOutput::close()
{
    if (is_open_) {
        cv::destroyWindow(window_name_);
        is_open_ = false;
        std::cout << "[OpenCVWindowOutput] Window closed: " << window_name_ << "\n";
    }
}

bool OpenCVWindowOutput::is_open() const
{
    return is_open_;
}
