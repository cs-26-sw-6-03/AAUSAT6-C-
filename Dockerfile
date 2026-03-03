# Use Ubuntu as base image for better package availability
FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    pkg-config \
    git \
    # GStreamer
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    # GLib
    libglib2.0-dev \
    # OpenCV dependencies
    libopencv-dev \
    # Cleanup
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy project files
COPY CMakeLists.txt CMakePresets.json Makefile ./
COPY src/ ./src/

# Build the project using Makefile
RUN make build CONFIG=default


# Copy entrypoint script
COPY entrypoint.sh ./
RUN chmod +x entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]

# Default command runs the application with help/placeholder paths
CMD ["/app/build/video_pipeline"]

