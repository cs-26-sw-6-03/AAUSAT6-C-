# AAUSAT6 - C++

## Building and running

Using MakeFile:

```bash
#Available targets:
make / make build       #- Build the project (default config)
make build CONFIG=debug #- Build with debug configuration
make build CONFIG=release #- Build with release configuration
make run                #- Build and run the project
make run ARGS='...'     #- Build and run with arguments
make clean              #- Remove build directory
make help               #- Show this help message
```

Or use CMake directly:

```bash
cmake --preset default
cmake --build build
# Go to build folder
cd build
# Run the code
./video_pipeline [Args*]
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