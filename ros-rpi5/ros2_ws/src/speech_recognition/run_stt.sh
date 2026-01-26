#!/bin/bash
# Quick start script for STT node - 2026 OPTIMIZED

# Source ROS2 workspace
source /home/pi5/ros2_ws/install/setup.bash

# Default to large-v3-turbo (best model in 2026)
MODEL_SIZE=${1:-large-v3-turbo}

echo "=========================================="
echo "  STT Node - 2026 Best Speech Recognition"
echo "=========================================="
echo ""
echo "Starting with: $MODEL_SIZE"
echo ""

case $MODEL_SIZE in
  large-v3-turbo)
    echo "✓ Using large-v3-turbo (~1.6GB)"
    echo "  - Best model in 2026"
    echo "  - 7.75% WER (99%+ accuracy)"
    echo "  - 6x faster than large-v3"
    echo "  - 216x real-time speed"
    ;;
  distil-large-v3)
    echo "✓ Using distil-large-v3 (~756MB)"
    echo "  - 6.3x faster than large-v3"
    echo "  - Within 1% WER of large-v3"
    ;;
  large-v3)
    echo "✓ Using large-v3 (~3GB)"
    echo "  - Maximum accuracy"
    echo "  - Slower processing"
    ;;
  *)
    echo "Using: $MODEL_SIZE"
    ;;
esac

echo ""
echo "On first run, model will be downloaded."
echo ""
echo "Usage: ./run_stt.sh [MODEL]"
echo "  large-v3-turbo - DEFAULT - Best in 2026 (~1.6GB)"
echo "  distil-large-v3 - Faster alternative (~756MB)"
echo "  large-v3       - Maximum accuracy (~3GB)"
echo "  medium         - Good balance (~1.5GB)"
echo "  small          - Lightweight (~466MB)"
echo "  base           - Basic (~145MB)"
echo "  tiny           - Testing only (~75MB)"
echo ""
echo "Starting in 3 seconds..."
sleep 3

ros2 run speech_recognition stt_node --ros-args -p model_size:=$MODEL_SIZE
