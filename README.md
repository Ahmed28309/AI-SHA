# AI-SHARJAH Robot System

ISC Sharjah Educational Robot - Complete Voice Assistant System

## Quick Start

Launch everything with one command:

```bash
./start_robot.sh
```

## System Components

- **Speech Recognition (STT)**: Listens to voice commands
- **LLM Processing**: AI brain for intelligent responses
- **Text-to-Speech (TTS)**: Speaks responses
- **BNO055 IMU**: Motion/orientation sensor

## Stop the Robot

Press `Ctrl+C` to stop all nodes.

## Requirements

- ROS2 Jazzy
- Whisper model (base.en)
- Llama 3.2 LLM model
- Audio device (ReSpeaker/microphone)
- BNO055 IMU sensor (optional)
