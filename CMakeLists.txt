cmake_minimum_required(VERSION 3.16)
project(video_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST     REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

find_package(OpenCV REQUIRED)

# ── Include directories ──────────────────────────────────────────────────────

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# ── Source files ─────────────────────────────────────────────────────────────

set(SOURCES
    src/main.cpp
    src/VideoInputStream/gstreamervideo.cpp
    src/FeatureDetection/StubDetector.cpp
    src/Stabilization/StubStabilizer.cpp
    src/Stabilization/Stabilizer.cpp
    src/Cropping/StubCropper.cpp
    src/FeatureDetection/ORBDetector.cpp
    src/VideoOutputStream/OpenCVWindowOutput.cpp
    src/VideoOutputStream/GstreamerFileOutput.cpp
)

# ── Main executable ──────────────────────────────────────────────────────────

add_executable(video_pipeline ${SOURCES})

target_link_libraries(video_pipeline PRIVATE
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${OpenCV_LIBS}
)

target_compile_options(video_pipeline PRIVATE
    ${GST_CFLAGS_OTHER}
    -Wall -Wextra -Wpedantic
)

# ── Optional: Install target ─────────────────────────────────────────────────

install(TARGETS video_pipeline
    RUNTIME DESTINATION bin
)