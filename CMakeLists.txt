cmake_minimum_required(VERSION 3.10)
project(VideoStabilization)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# Find OpenCV (if you have it installed)
find_package(OpenCV REQUIRED)

# Create executable
add_executable(video_stabilization
    src/main.cpp
)

# Link libraries
target_include_directories(video_stabilization PRIVATE 
    ${GSTREAMER_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(video_stabilization 
    ${GSTREAMER_LIBRARIES}
    ${OpenCV_LIBS}
)