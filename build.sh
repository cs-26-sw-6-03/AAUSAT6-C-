#!/bin/bash
# Build script that automatically creates build folder and configures/builds the project

CONFIG="default"
CLEAN=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN=true
            shift
            ;;
        --config)
            CONFIG="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--clean] [--config <preset>]"
            exit 1
            ;;
    esac
done

# Clean build folder if requested
if [ "$CLEAN" = true ] && [ -d "build" ]; then
    echo "Cleaning build folder..."
    rm -rf build
fi

# Create build folder if it doesn't exist
if [ ! -d "build" ]; then
    echo "Creating build folder..."
    mkdir build
fi

# Configure with CMake
echo "Configuring with CMake preset: $CONFIG"
cmake --preset $CONFIG

if [ $? -ne 0 ]; then
    echo "Configuration failed!"
    exit 1
fi

# Build
echo "Building..."
cmake --build build

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Build completed successfully!"
