# AAUSAT6 - C++

First, navigate to the build folder (if it is not there, create it), and run ```cmake .. ```

Then, in the root folder, running ```make``` will build the project. For now, there is no cross compilation to ARM, aka, the board that will be used. This will be added later.

For intellisense via VS Code, these paths should be included
``` 
${workspaceFolder}/**
/usr/include/gstreamer-1.0
/usr/include/glib-2.0
/usr/lib/glib-2.0/include
/usr/include/sysprof-6
/usr/include/opencv4
``` 

Install these (arch btw)
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
``` 