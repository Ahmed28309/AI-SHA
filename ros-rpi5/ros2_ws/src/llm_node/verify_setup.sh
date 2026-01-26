#!/bin/bash
# Verification script for LLM Node setup

echo "=========================================="
echo "LLM Node Setup Verification"
echo "=========================================="
echo ""

# Check llama-cpp-python
echo "[1/3] Checking llama-cpp-python..."
if python3 -c "import llama_cpp" 2>/dev/null; then
    echo "✓ llama-cpp-python is installed"
    python3 -c "from llama_cpp import Llama; print(f'  Version: {Llama.__module__}')"
else
    echo "✗ llama-cpp-python is NOT installed"
    echo "  Run: pip3 install llama-cpp-python --break-system-packages"
fi

echo ""

# Check huggingface-hub
echo "[2/3] Checking huggingface-hub..."
if python3 -c "import huggingface_hub" 2>/dev/null; then
    echo "✓ huggingface-hub is installed"
else
    echo "✗ huggingface-hub is NOT installed"
    echo "  Run: pip3 install huggingface-hub --break-system-packages"
fi

echo ""

# Check model file
echo "[3/3] Checking Llama model..."
MODEL_PATH=~/llm_models/Llama-3.2-1B-Instruct-Q4_K_M.gguf
if [ -f "$MODEL_PATH" ]; then
    echo "✓ Model found at $MODEL_PATH"
    SIZE=$(du -h "$MODEL_PATH" | cut -f1)
    echo "  Size: $SIZE"
else
    echo "✗ Model NOT found at $MODEL_PATH"
    echo "  Run installation script:"
    echo "  cd ~/ros2_ws/src/llm_node && ./install_llm_deps.sh"
fi

echo ""

# Check ROS2 packages
echo "[4/4] Checking ROS2 packages..."
source ~/ros2_ws/install/setup.bash 2>/dev/null

if ros2 pkg list | grep -q "llm_node"; then
    echo "✓ llm_node package is built"
else
    echo "✗ llm_node package is NOT built"
    echo "  Run: cd ~/ros2_ws && colcon build --packages-select llm_node"
fi

if ros2 pkg list | grep -q "speech_recognition"; then
    echo "✓ speech_recognition package is built"
else
    echo "✗ speech_recognition package is NOT built"
    echo "  Run: cd ~/ros2_ws && colcon build --packages-select speech_recognition"
fi

echo ""
echo "=========================================="
echo "Setup verification complete"
echo "=========================================="
