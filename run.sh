# Create build folder if it doesn't exist
if [ -d "build" ]; then
    cd build/
    make
    ./video_stabilization
    cd ..
fi