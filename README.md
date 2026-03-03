# AAUSAT6 - C++ Video Pipeline

Video processing pipeline with feature detection, stabilization, and cropping.

## Requirements

- C++17 compiler (GCC/Clang)
- CMake 3.16+
- GStreamer 1.0
- OpenCV 4.x
- pkg-config



## Dependencies

Install these (arch btw):
``` 
gstreamer
glib2
gst-plugins-base
gst-plugins-good
gst-plugins-bad
gst-plugins-ugly
cmake
vtk
hdf5
opencv
qt6-base
``` 

**Arch Linux**:

```bash
sudo pacman -S gstreamer glib2 gst-plugins-base gst-plugins-good \
               gst-plugins-bad gst-plugins-ugly cmake opencv
```

**Ubuntu/Debian**:

```bash
sudo apt install build-essential cmake pkg-config \
                 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                 gstreamer1.0-plugins-{good,bad,ugly} libopencv-dev
```

## Building

### Using Make (Recommended)

```bash
make        # Build the project
make clean  # Clean build artifacts (do before building anew)
```

### Manual Build

```bash
cmake --preset default
cmake --build build
```

## Running

```bash
# Display in window
./build/video_pipeline <input_video> <reference_image>

# Write to file
./build/video_pipeline <input_video> <reference_image> <output.mp4>
```

### Development

```bash
make run ARGS='samples/test_clip.mp4 samples/area.png'
make run ARGS='samples/test_clip.mp4 samples/area.png output.mp4'
``` 

## VS Code IntelliSense

For intellisense via VS Code, these paths should be included:
``` 
${workspaceFolder}/**
/usr/include/gstreamer-1.0
/usr/include/glib-2.0
/usr/lib/glib-2.0/include
/usr/include/sysprof-6
/usr/include/opencv4
``` 