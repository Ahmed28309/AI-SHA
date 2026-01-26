# LLM Node for ROS2

Lightweight conversational AI node using Llama 3.2 1B for Raspberry Pi 5.

## Pipeline Architecture

```
[Microphone] → [Speech Recognition] → [LLM Node] → [TTS Speaker] → [Audio Out]
                (/speech_rec)          (/speech/text)
```

## Setup Instructions

### 1. Install Dependencies

```bash
# Install llama-cpp-python (with CPU optimizations)
pip3 install llama-cpp-python --extra-index-url https://abetlen.github.io/llama-cpp-python/whl/cpu
```

### 2. Download Llama 3.2 1B Model

```bash
# Install huggingface-cli if not already installed
pip3 install huggingface-hub

# Create model directory
mkdir -p ~/llm_models

# Download quantized model (Q4_K_M - good balance of speed and quality)
huggingface-cli download bartowski/Llama-3.2-1B-Instruct-GGUF \
    llama-3.2-1b-instruct-q4_k_m.gguf \
    --local-dir ~/llm_models
```

Alternative smaller model for even faster inference:
```bash
# Download Q3_K_M (smaller, faster, slightly lower quality)
huggingface-cli download bartowski/Llama-3.2-1B-Instruct-GGUF \
    llama-3.2-1b-instruct-q3_k_m.gguf \
    --local-dir ~/llm_models
```

### 3. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select llm_node
source install/setup.bash
```

### 4. Run the Complete Pipeline

Terminal 1 - Speech Recognition:
```bash
ros2 run speech_recognition stt_node
```

Terminal 2 - LLM Node:
```bash
ros2 run llm_node llm_node
```

Terminal 3 - TTS Speaker:
```bash
ros2 run tts_speaker tts_speaker_node
```

## Configuration

### LLM Node Parameters

You can customize the node with ROS2 parameters:

```bash
ros2 run llm_node llm_node --ros-args \
    -p model_path:=~/llm_models/llama-3.2-1b-instruct-q4_k_m.gguf \
    -p temperature:=0.7 \
    -p max_tokens:=150 \
    -p n_threads:=4 \
    -p system_prompt:="You are a helpful AI assistant. Keep responses brief."
```

### Performance Tips for Raspberry Pi 5

1. **Model size**: Q4_K_M provides good balance. Use Q3_K_M for faster inference.
2. **Threads**: 4 threads works well on Pi 5. Adjust based on system load.
3. **Max tokens**: Keep responses short (100-150 tokens) for faster generation.
4. **Context window**: 2048 tokens is sufficient for most conversations.

## Topics

- **Input**: `/speech_rec` (std_msgs/String) - Recognized speech from STT node
- **Output**: `/speech/text` (std_msgs/String) - LLM response for TTS node

## Conversation Features

- Maintains conversation history (last 5 exchanges)
- Context-aware responses
- Optimized for conversational speed on Pi 5
