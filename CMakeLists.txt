cmake_minimum_required(VERSION 3.10)
project(VideoStabilization)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Qt6
find_package(Qt6 COMPONENTS Core Gui Widgets QUIET)
if(Qt6_FOUND)
    message(STATUS "Found Qt6: ${Qt6_VERSION}")
endif()

# Create executable
add_executable(video_stabilization
    src/orb.cpp
)

# Include directories
target_include_directories(video_stabilization PRIVATE 
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(video_stabilization 
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${OpenCV_LIBS}
)

# Link Qt6 if found
if(Qt6_FOUND)
    target_link_libraries(video_stabilization 
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
    )
endif()