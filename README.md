# AI-SHA — School Support Robot

This project is developed by a team of high-school students to create a functional assistant robot for schools. The robot aims to support teachers, administrators, and parents by performing simple tasks, providing information, and improving campus efficiency.

## Quick Start (Pi5 ROS2 System)

Launch the full voice assistant system:

```bash
./start_robot.sh
```

Or individual components:

```bash
./launch_stt.sh   # Speech recognition
./launch_tts.sh   # Text-to-speech
```

## Pi5 ROS2 Packages (`src/`)

| Package | Description |
|---|---|
| `bmp180_pressure` | BMP180 temperature & pressure sensor |
| `bno055_imu` | BNO055 9-DOF IMU driver |
| `gps_gt_u7` | GPS module driver |
| `llm_display` | LLM response display node |
| `rain_sensor` | Rain sensor (Arduino serial bridge) |
| `robot_display` | Robot face/display node |
| `soil_moisture` | Soil moisture sensor (Arduino serial bridge) |
| `speaker_monitor` | Audio output monitor |
| `tts_elevenlabs` | ElevenLabs text-to-speech node |

**Requirements:** ROS2 Jazzy · Whisper (base.en) · Llama 3.2 · Audio device (ReSpeaker/microphone)

## Project Overview

Our robot integrates multiple computing platforms and custom CAD-designed hardware to perform navigation, sensing, and interaction tasks within a school environment.

### Core Components
- **NVIDIA Jetson Nano** — AI processing and sensor integration
- **Raspberry Pi 5** — ROS2 sensor nodes, voice assistant, peripherals
- **Intel RealSense D435** — Depth camera
- **RPLIDAR A2** — 360° mapping
- **BNO055 9-DOF IMU**, rotary encoders, audio modules

### CAD & Mechanical Design
- Robot frame and mounting points designed in CAD
- 3D-printed and machined parts
- Modular design for easy upgrades and repairs

### Planned Capabilities
- Delivering documents or small items
- Assisting teachers and admin with quick tasks
- Guiding parents or visitors in the school
- Providing information via voice or screen interface
- Navigating hallways using LIDAR + depth sensing

### Project Goals
- Build a helpful and safe school-assistant robot
- Learn real engineering: robotics, CAD, electronics, AI, and software
- Create a platform future students can build on
- Show what high-school teams can achieve with teamwork and creativity

## Created by Students
This entire robot — from CAD design to wiring to programming — is built by high-school students passionate about robotics and engineering.
