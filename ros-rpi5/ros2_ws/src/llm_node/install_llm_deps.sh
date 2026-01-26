#!/bin/bash
# Installation script for LLM Node dependencies and model

set -e

echo "=========================================="
echo "LLM Node Setup for Raspberry Pi 5"
echo "=========================================="

# Install Python dependencies
echo ""
echo "[1/4] Installing Python dependencies..."
echo "Note: Using --break-system-packages for ROS2 development environment"
echo ""

# Check if running in a virtual environment
if [ -n "$VIRTUAL_ENV" ]; then
    echo "Virtual environment detected: $VIRTUAL_ENV"
    PIP_FLAGS=""
else
    echo "Installing to system Python (required for ROS2)"
    PIP_FLAGS="--break-system-packages"
fi

pip3 install --upgrade pip $PIP_FLAGS
pip3 install llama-cpp-python --extra-index-url https://abetlen.github.io/llama-cpp-python/whl/cpu $PIP_FLAGS
pip3 install huggingface-hub $PIP_FLAGS

# Create model directory
echo ""
echo "[2/4] Creating model directory..."
mkdir -p ~/llm_models

# Download model (note: correct filename uses uppercase)
MODEL_FILE="Llama-3.2-1B-Instruct-Q4_K_M.gguf"
echo ""
echo "[3/4] Downloading Llama 3.2 1B model..."
echo "This may take a few minutes depending on your internet speed..."
echo "Model: $MODEL_FILE (~750MB)"

if [ -f ~/llm_models/$MODEL_FILE ]; then
    echo "Model already exists at ~/llm_models/$MODEL_FILE"
    echo "Skipping download."
else
    # Use Python to download (most reliable method)
    python3 << PYEOF
from huggingface_hub import hf_hub_download
import os

model_dir = os.path.expanduser("~/llm_models")
os.makedirs(model_dir, exist_ok=True)

print("Downloading model from HuggingFace...")
hf_hub_download(
    repo_id="bartowski/Llama-3.2-1B-Instruct-GGUF",
    filename="$MODEL_FILE",
    local_dir=model_dir,
    local_dir_use_symlinks=False
)
print("Model downloaded successfully!")
PYEOF
fi

# Build ROS2 package
echo ""
echo "[4/4] Building ROS2 packages..."
cd ~/ros2_ws
colcon build --packages-select llm_node speech_recognition

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "To use the nodes, source the workspace:"
echo "  source ~/ros2_ws/install/setup.bash"
echo ""
echo "Then run the pipeline in separate terminals:"
echo "  Terminal 1: ros2 run speech_recognition stt_node"
echo "  Terminal 2: ros2 run llm_node llm_node"
echo "  Terminal 3: ros2 run tts_speaker tts_speaker_node"
echo ""
echo "Model location: ~/llm_models/$MODEL_FILE"
echo "Model size: ~750MB (Q4_K_M quantization)"
echo ""
