.PHONY: build run clean help docker-build docker-run

# Configuration preset (default, debug, release)
CONFIG ?= default

# Docker configuration
DOCKER_IMAGE ?= aausat6-video-pipeline
VIDEO ?=
REF ?=
OUTPUT ?= output.mp4

all: build # Default target

build: # Build the project
	@echo "Configuring with CMake preset: $(CONFIG)..."
	@cmake --preset $(CONFIG)
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
	@echo "  make / make build       - Build the project (default config)"
	@echo "  make build CONFIG=debug - Build with debug configuration"
	@echo "  make build CONFIG=release - Build with release configuration"
	@echo "  make run                - Build and run the project"
	@echo "  make run ARGS='...'     - Build and run with arguments"
	@echo "  make clean              - Remove build directory"
	@echo "  make docker-build       - Build Docker image"
	@echo "  make docker-run VIDEO=... REF=... [OUTPUT=...] - Run in Docker"
	@echo "  make help               - Show this help message"

docker-build: # Build Docker image
	@echo "Building Docker image..."
	@docker build -t $(DOCKER_IMAGE) .

docker-run: # Run pipeline in Docker
	@if [ -z "$(VIDEO)" ] || [ -z "$(REF)" ]; then \
		echo "Error: VIDEO and REF are required"; \
		echo "Usage: make docker-run VIDEO=path/to/video.mp4 REF=path/to/ref.jpg [OUTPUT=output.mp4]"; \
		exit 1; \
	fi
	@echo "Running pipeline in Docker..."
	@docker run --rm \
		-v $$(pwd):/app/output:rw \
		-v $$(dirname $$(realpath $(VIDEO))):/app/data:ro \
		-v $$(dirname $$(realpath $(REF))):/app/ref:ro \
		-w /app/output \
		$(DOCKER_IMAGE) \
		/app/build/video_pipeline \
		/app/data/$$(basename $(VIDEO)) \
		/app/ref/$$(basename $(REF)) \
		$(OUTPUT)
	@echo "✅ Done! Output: $(OUTPUT)"

