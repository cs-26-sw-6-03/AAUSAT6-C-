# AAUSAT6 - C++

## Building

```bash
./build.sh
```

Or use CMake directly:
```bash
cmake --preset default
cmake --build build
```

For now, there is no cross compilation to ARM, aka, the board that will be used. This will be added later.

## Requirements

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