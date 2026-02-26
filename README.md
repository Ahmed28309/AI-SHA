# AI-SHA &mdash; School Assistant Robot

> A multi-platform educational robot built by high-school students at the International School of Choueifat, Sharjah.
> AI-SHA navigates hallways, answers questions, and helps teachers, students, and
> visitors using voice interaction, computer vision, and autonomous driving.

---

## System Architecture

```
                         ┌──────────────────────────────────────────┐
                         │         NVIDIA Jetson Orin Nano 8GB       │
                         │              (ROS 2 Humble)               │
                         │                                           │
                         │  ┌───────────┐  ┌──────────────────────┐ │
                         │  │  LLM Node │  │   YOLOv8 Detection   │ │
                         │  │ (Gemini / │  │  + Face + Gesture    │ │
                         │  │  Llama3.2)│  │  + OCR (TensorRT)   │ │
                         │  └─────┬─────┘  └──────────┬───────────┘ │
                         │        │   ▲               │             │
                         │  ┌─────┴───┴─────┐  ┌─────┴───────────┐ │
                         │  │  STT Node     │  │  SLAM Toolbox   │ │
                         │  │ (Whisper GPU) │  │  (LD19 LiDAR)   │ │
                         │  └───────────────┘  └─────────────────┘ │
                         └────────┬───────────────────┬─────────────┘
                    /tts_text     │                    │ /scan
                    (responses)   │ /speech/text       │
            ┌─────────────────────┘                    │
            │                                          │
  ┌─────────▼──────────────────────────────────────────┼────────────┐
  │              Raspberry Pi 5  (ROS 2 Jazzy)         │            │
  │                                                    │            │
  │  ┌────────────┐  ┌────────────────────┐  ┌────────┘          │ │
  │  │  TTS Node  │  │  Display Nodes     │  │                   │ │
  │  │ (ElevenLabs│  │  Face / Dashboard  │  │  BNO055 IMU       │ │
  │  │    API)    │  │  (PyQt5)           │  │  (I2C, 50Hz)      │ │
  │  └────────────┘  └────────────────────┘  └───────────────────┘ │
  └─────────────────────────────────────────────────────────────────┘
                                │
                         USB Serial / GPIO
            ┌───────────────────▼───────────────────┐
            │         Raspberry Pi 4                 │
            │                                        │
            │  Motor Control (GPIO + PIDF)           │
            │  4x Encoders (E38S6G5-600B-G24N)      │
            │  Odometry Publisher                     │
            │  /cmd_vel subscriber                   │
            └────────────────────────────────────────┘
```

### Voice Pipeline

```
Microphone ──▶ [STT Node] ──/speech/text──▶ [LLM Node] ──/tts_text──▶ [TTS Node] ──▶ Speaker
  (ReSpeaker      (Faster-Whisper            (Gemini 2.5 Flash        (ElevenLabs
   6-channel)      GPU-accelerated)           or Llama 3.2 3B)         API)
                                                  ▲
                                                  │
                              /detection/objects_simple (from YOLOv8)
```

---

## Directory Structure

```
AI-SHA/
│
├── llm_node/                Jetson LLM node (Gemini / Llama 3.2)
│   ├── llm_node/                Core node implementations
│   │   ├── llm_node.py              Local Llama 3.2 3B inference
│   │   ├── llm_node_gemini.py       Google Gemini 2.5 Flash
│   │   ├── llm_node_api.py          API-based LLM interface
│   │   └── chess_handler.py         Chess game handler
│   ├── prompts/                 System prompts
│   │   ├── sabis_robot_system.txt   Full SABIS school context
│   │   └── sabis_robot_concise.txt  Hardware-aware concise prompt
│   └── launch/
│       └── llm_sabis.launch.py      Launch with SABIS prompts
│
├── stt_node/                Jetson STT node (Faster-Whisper)
│   └── stt_node/
│       ├── stt_node.py              Local Whisper (GPU-accelerated)
│       ├── stt_assemblyai.py        AssemblyAI cloud STT
│       └── stt_node_api.py          API-based STT interface
│
├── yolov8_ros/              Jetson YOLOv8 vision node (TensorRT)
│   └── yolov8_ros/
│       └── yolov8_node.py           Object detection + face + gesture + OCR
│
├── robot_brain/             Jetson local LLM with vision context
│   └── robot_brain/
│       └── robot_brain.py           Llama 3.2 3B text generation
│
├── robot_bringup/           Launch files and configs
│   ├── launch/
│   │   ├── bringup.launch.py       Camera + STT + YOLO
│   │   ├── cerebro.launch.py       Full AI pipeline
│   │   ├── slam.launch.py          SLAM mapping
│   │   └── slam_simple.launch.py   Lightweight SLAM
│   └── config/
│       ├── ld19.yaml                LiDAR configuration
│       ├── slam_toolbox.yaml        SLAM parameters
│       └── slam.rviz                RViz visualization
│
├── jetson/                  Additional Jetson packages
│   └── robot_description/       Robot URDF model (sensor frames)
│
├── ldlidar_stl_ros2/        LD-19 LiDAR driver (third-party)
│
├── src/                     RPi4/RPi5 sensor & actuator packages
│   ├── bmp180_pressure/         Barometric pressure & temperature
│   ├── bno055_imu/              9-DOF IMU driver (I2C)
│   ├── gps_gt_u7/               GPS module driver
│   ├── llm_display/             Robot face + dashboard display (PyQt5)
│   ├── mecanum_driver/          Mecanum wheel control + Arduino firmware
│   ├── motor_control/           GPIO motor + encoder nodes
│   ├── rain_sensor/             Rain sensor + Arduino firmware
│   ├── robot_display/           Robot face display (Pi4)
│   ├── soil_moisture/           Soil moisture + Arduino firmware
│   ├── speaker_monitor/         Audio playback monitor
│   └── tts_elevenlabs/          Text-to-Speech (ElevenLabs API)
│
├── scripts/                 Shell launchers and test scripts
├── tools/                   Standalone CLI utilities
├── docs/                    Documentation and guides
└── legacy/                  Old prototypes (kept for reference)
```

---

## ROS 2 Packages

### AI & Vision (Jetson Orin Nano)

| Package | Description | Key Topics |
|---------|-------------|------------|
| `llm_node` | LLM integration &mdash; Gemini 2.5 Flash (cloud) or Llama 3.2 3B (local, Q4_K_M). Loads SABIS school prompts, conversation history, vision context. | sub: `/speech/text` &rarr; pub: `/tts_text` |
| `stt_node` | Speech-to-text &mdash; Faster-Whisper (GPU). VAD, mic muting during TTS, 10s cooldown. Supports AssemblyAI cloud fallback. | pub: `/speech/text` |
| `yolov8_ros` | Real-time detection &mdash; YOLOv8m (TensorRT, 30+ FPS). Face detection (MediaPipe), hand gestures (17 types), OCR (EasyOCR), depth estimation. | pub: `/detection/objects_simple`, `/detection/faces_simple`, `/detection/gestures_simple`, `/detection/ocr_simple` |
| `robot_brain` | Local LLM text generation with vision context integration (Project Cerebro). | sub: `/speech_rec`, `/detection/objects_simple` &rarr; pub: `/speech/text` |
| `robot_bringup` | Launch files: `cerebro.launch.py` (full AI pipeline), `bringup.launch.py` (camera + detection), `slam.launch.py` (mapping). | &mdash; |

### Sensors & Actuators (RPi4/RPi5 &mdash; `src/`)

| Package | Description | Publishes | Hardware |
|---------|-------------|-----------|----------|
| `bmp180_pressure` | Temperature & pressure | `/bmp180/pressure`, `/temperature`, `/altitude` | BMP180 (I2C) |
| `bno055_imu` | 9-DOF IMU driver | `/imu/data` | BNO055 (I2C) |
| `gps_gt_u7` | GPS positioning | `/gps/fix`, `/gps/vel`, `/gps/time_ref` | GT-U7 (Serial) |
| `rain_sensor` | Rain detection | `/rain_sensor/raw`, `/raining` | Arduino (Serial) |
| `soil_moisture` | Soil moisture | `/soil_moisture/raw`, `/dry` | Arduino (Serial) |
| `mecanum_driver` | Mecanum wheel control | `/wheel_speeds` | Arduino (Serial) |
| `motor_control` | GPIO motor + encoders | `/motor_status`, `/encoders/rpm` | GPIO (pigpio) |
| `tts_elevenlabs` | Text-to-Speech | `/tts/started`, `/tts/finished` | Speaker (audio) |
| `speaker_monitor` | Audio state monitor | `/audio/playing` | MAX98357A DAC |
| `llm_display` | Robot face + chat display | &mdash; | Elecrow Display |
| `robot_display` | Robot face display | &mdash; | Pi4 Display |

---

## Hardware

| Component | Model | Purpose | Connection |
|-----------|-------|---------|------------|
| AI Computer | NVIDIA Jetson Orin Nano 8GB | LLM, vision, STT, SLAM | Ethernet |
| Speech/Display | Raspberry Pi 5 | TTS output, animated face | Ethernet |
| Motor Control | Raspberry Pi 4 | Motors, encoders, odometry | GPIO/Serial |
| Depth Camera | Intel RealSense D435 | RGB + depth perception | USB (Jetson) |
| LiDAR | LD-19 2D | 360-degree laser scanning | Serial |
| IMU | BNO055 9-DOF | Orientation & motion | I2C (50Hz) |
| Microphone | ReSpeaker Mic Array v3.0 | 6-channel voice capture | USB Audio |
| Speaker | MAX98357A DAC + Amplifier | Voice output | I2S |
| Encoders | E38S6G5-600B-G24N (x4) | Wheel odometry (600 PPR) | GPIO |
| Motors | 3x DC + H-bridge | Skid-steer drive | GPIO PWM |
| GPS | GT-U7 | Outdoor positioning | Serial |
| Pressure | BMP180 | Barometric pressure | I2C |
| Display | Elecrow Touchscreen | Robot face display | HDMI/DSI |

---

## AI Models

| Model | Task | Platform | Performance |
|-------|------|----------|-------------|
| YOLOv8m | Object detection | TensorRT (Jetson GPU) | 30+ FPS |
| Faster-Whisper (small) | Speech-to-text | CUDA (Jetson GPU) | 0.5&ndash;1.5s latency |
| Llama 3.2 3B (Q4_K_M) | Local LLM | llama.cpp (Jetson GPU) | ~1&ndash;2s response |
| Google Gemini 2.5 Flash | Cloud LLM | API | ~1s response |
| ElevenLabs | Text-to-speech | API (RPi5) | Real-time streaming |
| MediaPipe | Face + hand gesture detection | CPU/GPU | Real-time |
| EasyOCR | Text recognition | CUDA | Real-time |

---

## System Prompts

AI-SHA uses custom system prompts for the SABIS school environment, located in `llm_node/prompts/`:

- **`sabis_robot_system.txt`** &mdash; Full SABIS school context: 140-year history, school policies, CA assessment system, campus navigation, response protocols for students/parents/teachers/visitors.
- **`sabis_robot_concise.txt`** &mdash; Hardware-aware prompt with complete robot specification, general knowledge assistant mode, response rules for TTS clarity.

---

## Getting Started

### Prerequisites

**Jetson Orin Nano:**
```bash
sudo apt install ros-humble-desktop
pip3 install llama-cpp-python ultralytics faster-whisper mediapipe easyocr
# Place Llama-3.2-3B-Instruct-Q4_K_M.gguf in ~/models/
```

**Raspberry Pi 5:**
```bash
sudo apt install ros-jazzy-desktop
pip3 install elevenlabs PyQt5
```

**Raspberry Pi 4:**
```bash
sudo apt install ros-humble-desktop
pip3 install adafruit-circuitpython-bno055 smbus2 pyserial pynmea2 pigpio
sudo apt install pigpiod ffmpeg
```

### Build

```bash
git clone git@github.com:Ahmed28309/AI-SHA.git
cd AI-SHA

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Environment Variables

```bash
# Required for TTS
export ELEVENLABS_API_KEY="your-key-here"

# Required for Gemini LLM
export GOOGLE_API_KEY="your-key-here"

# Cross-machine communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

### Launch

```bash
# Full AI pipeline (Jetson &mdash; camera + STT + YOLO + LLM)
ros2 launch robot_bringup cerebro.launch.py

# Camera + detection only
ros2 launch robot_bringup bringup.launch.py

# SLAM mapping
ros2 launch robot_bringup slam.launch.py

# LLM with SABIS prompts
ros2 launch llm_node llm_sabis.launch.py
```

### Quick Start &mdash; Individual Nodes

```bash
# STT (speech recognition)
ros2 run stt_node stt_node

# Object detection
ros2 run yolov8_ros yolov8_node

# TTS (text-to-speech)
ros2 run tts_elevenlabs tts_elevenlabs_node

# Robot face display
ros2 run llm_display robot_face_display

# IMU
ros2 run bno055_imu bno055_node

# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Useful Topics

```bash
ros2 topic list                      # See all active topics

ros2 topic echo /speech/text         # STT output
ros2 topic echo /tts_text            # LLM responses
ros2 topic echo /detection/objects_simple  # Detected objects
ros2 topic echo /imu/data            # IMU readings
ros2 topic echo /scan                # LiDAR data
ros2 topic echo /cmd_vel             # Movement commands
```

---

## Documentation

| Guide | Description |
|-------|-------------|
| [Jetson Integration](docs/JETSON_INTEGRATION.md) | Setting up Pi-to-Jetson ROS 2 communication |
| [TTS Troubleshooting](docs/TTS_TROUBLESHOOTING.md) | Debugging audio output issues |
| [Hardware Setup](docs/HARDWARE_SETUP.pdf) | Wiring diagrams and sensor connections |

---

## Project Team

**AI-SHA** is built entirely by high-school students at the **International School of Choueifat, Sharjah**.
From CAD design to wiring to programming &mdash; every part of this robot was designed and built by our team.

| Member |
|--------|
| Srinjay Shankar |
| Yousef Saleh |
| Hamza Husseini |
| Ahmed Rizwan |
| Abdulhayee Yamin |
| Jameel Ahmed |
| Yasir AlGuburi |
| Nouredin |

**Our Goal:** Build a helpful, safe school-assistant robot and learn real engineering along the way.
We hope future students will continue to build on this platform.

---

## License

MIT License &mdash; see [LICENSE](LICENSE) for details.
