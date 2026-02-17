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

# Find Qt6 (required by OpenCV modules compiled with Qt support)
find_package(Qt6 COMPONENTS Core Gui Widgets QUIET)
if(Qt6_FOUND)
    message(STATUS "Found Qt6: ${Qt6_VERSION}")
endif()

# Create executable
add_executable(video_stabilization
    src/Fast.cpp
)

# Link libraries
target_include_directories(video_stabilization PRIVATE 
    ${GSTREAMER_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
)

target_link_libraries(video_stabilization 
    ${GSTREAMER_LIBRARIES}
    ${OpenCV_LIBS}
    ${GSTREAMER_APP_LIBRARIES}
)

# Link Qt6 if found (required for OpenCV modules with Qt support)
if(Qt6_FOUND)
    target_link_libraries(video_stabilization 
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
    )
endif()