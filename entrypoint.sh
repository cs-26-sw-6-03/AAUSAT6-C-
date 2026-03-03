#!/bin/bash

set -e

export QT_QPA_PLATFORM=offscreen

echo "Starting video_pipeline..."
echo "Arguments: $@"
echo ""

exec "$@"

