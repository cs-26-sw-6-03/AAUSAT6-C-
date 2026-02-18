.PHONY: build run clean help

all: build # Default target 

build: # Build the project
	@echo "Configuring with CMake..."
	@cmake --preset default
	@echo "Building..."
	@cmake --build build
	@echo "Build complete!"

run: build # Run the executable with optional arguments
	@echo "Running video_pipeline..."
	@./build/video_pipeline $(ARGS)

clean: # Clean build directory
	@echo "Cleaning build directory..."
	@rm -rf build
	@echo "Clean complete!"

help: # Help message
	@echo "Available targets:"
	@echo "  make / make build  - Build the project"
	@echo "  make run          - Build and run the project"
	@echo "  make run ARGS='...' - Build and run with arguments"
	@echo "  make clean        - Remove build directory"
	@echo "  make help         - Show this help message"
