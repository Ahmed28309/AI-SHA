# AI-SHA &mdash; School Assistant Robot

> A multi-platform educational robot built by high-school students at the International School of Choueifat, Sharjah.
> AI-SHA navigates hallways, answers questions, and helps teachers, students, and
> visitors using voice interaction, computer vision, and autonomous driving.

---

## System Architecture

```
                         ┌──────────────────────────────────┐
                         │       NVIDIA Jetson Orin Nano     │
                         │           (ROS 2 Humble)          │
                         │                                   │
                         │  ┌───────────┐  ┌──────────────┐ │
                         │  │  LLM Node │  │  YOLOv8 Obj  │ │
                         │  │ (Llama3.2 │  │  Detection   │ │
                         │  │    3B)    │  │  (TensorRT)  │ │
                         │  └─────┬─────┘  └──────┬───────┘ │
                         │        │   ▲           │         │
                         └────────┼───┼───────────┼─────────┘
                    /tts_text     │   │ /speech/  │ /detected_objects
                    (responses)   │   │   text    │
            ┌─────────────────────┼───┼───────────┘
            │                     │   │
  ┌─────────▼─────────────────────┼───┼──────────────────────────────┐
  │              Raspberry Pi 4  (ROS 2 Humble)                       │
  │                               │   │                               │
  │  ┌────────────┐  ┌───────────┴───┴──────────┐  ┌──────────────┐ │
  │  │  TTS Node  │  │  Display Nodes  (PyQt5)  │  │ LiDAR LD-19  │ │
  │  │ (ElevenLabs│  │  Face / Dashboard / Chat  │  │    Driver    │ │
  │  │    API)    │  │                           │  │              │ │
  │  └────────────┘  └──────────────────────────┘  └──────────────┘ │
  │                                                                   │
  │  ┌─────────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐ │
  │  │ BNO055 IMU  │  │  BMP180  │  │ GPS GT-U7│  │   Speaker    │ │
  │  │   (I2C)     │  │ Pressure │  │ (Serial) │  │   Monitor    │ │
  │  └─────────────┘  └──────────┘  └──────────┘  └──────────────┘ │
  │                                                                   │
  │  ┌────────────────────────────────────────────────────────────┐  │
  │  │  Mecanum Driver (Serial)  /  Motor Control (GPIO + PID)   │  │
  │  └──────────────────────────┬─────────────────────────────────┘  │
  └─────────────────────────────┼─────────────────────────────────────┘
                                │  USB Serial / GPIO
                   ┌────────────▼──────────────────┐
                   │       Arduino Uno  (x2)        │
                   │                                 │
                   │  4x Mecanum motors  (L293D)     │
                   │  Rain sensor  (analog)          │
                   │  Soil moisture  (analog)         │
                   └─────────────────────────────────┘
```

### Voice Pipeline

```
Microphone ──▶ [STT Node] ──/speech/text──▶ [Jetson LLM] ──/tts_text──▶ [TTS Node] ──▶ Speaker
                                                  │
                                           /detected_objects
                                                  │
                                           [Display Node]
```

---

## Directory Structure

```
AI-SHA/
│
├── src/                     ROS 2 packages — sensors, display, motors
│   ├── bmp180_pressure/         Barometric pressure & temperature
│   ├── bno055_imu/              9-DOF IMU driver
│   ├── gps_gt_u7/               GPS module driver
│   ├── llm_display/             LLM response display (Pi5)
│   ├── mecanum_driver/          Mecanum wheel control + Arduino firmware
│   ├── motor_control/           GPIO motor + encoder nodes
│   ├── rain_sensor/             Rain sensor + Arduino firmware
│   ├── robot_display/           Robot face display (Pi4)
│   ├── soil_moisture/           Soil moisture + Arduino firmware
│   ├── speaker_monitor/         Audio playback monitor
│   └── tts_elevenlabs/          Text-to-Speech (ElevenLabs API)
│
├── robot_bringup/           Launch files for the full robot
├── llm_node/                Jetson LLM node (Llama 3.2 3B)
├── yolov8_ros/              Jetson YOLOv8 vision node
├── ldlidar_stl_ros2/        LD-19 LiDAR driver (third-party)
│
├── scripts/                 Shell launchers and test scripts
│   ├── start_robot.sh           Launch the full robot system
│   ├── launch_stt.sh            Start speech recognition
│   ├── launch_tts.sh            Start text-to-speech
│   └── test_*.py / test_*.sh    Diagnostics and integration tests
│
├── tools/                   Standalone CLI utilities
│   ├── motor_cli.py             Interactive mecanum motor control
│   └── lambdacli.py             Lambda Cloud GPU management
│
├── docs/                    Documentation and guides
│   ├── JETSON_INTEGRATION.md    Pi ↔ Jetson ROS 2 communication
│   ├── TTS_TROUBLESHOOTING.md   Audio output debugging
│   └── HARDWARE_SETUP.pdf       Wiring diagrams and sensor connections
│
└── legacy/                  Old prototypes (kept for reference)
```

---

## ROS 2 Packages

### Sensors & Actuators (`src/`)

| Package | Description | Publishes | Subscribes | Hardware |
|---------|-------------|-----------|------------|----------|
| `bmp180_pressure` | Temperature & pressure | `/bmp180/pressure`, `/temperature`, `/altitude` | &mdash; | BMP180 (I2C) |
| `bno055_imu` | 9-DOF IMU driver | `/imu/data` | &mdash; | BNO055 (I2C) |
| `gps_gt_u7` | GPS positioning | `/gps/fix`, `/gps/vel`, `/gps/time_ref` | &mdash; | GT-U7 (Serial) |
| `rain_sensor` | Rain detection | `/rain_sensor/raw`, `/raining` | &mdash; | Arduino (Serial) |
| `soil_moisture` | Soil moisture | `/soil_moisture/raw`, `/dry` | &mdash; | Arduino (Serial) |
| `mecanum_driver` | Mecanum wheel control | `/wheel_speeds` | `/cmd_vel` | Arduino (Serial) |
| `motor_control` | GPIO motor + encoders | `/motor_status`, `/encoders/rpm` | `/cmd_rpm` | GPIO (pigpio) |
| `tts_elevenlabs` | Text-to-Speech | `/robot/speaking`, `/tts/started`, `/tts/finished` | `/tts_text`, `/pause` | Speaker (audio) |
| `speaker_monitor` | Audio state monitor | `/audio/playing` | &mdash; | MAX98357A DAC |
| `llm_display` | LLM chat display | &mdash; | `/speech/text`, `/speech_rec` | Elecrow Display |
| `robot_display` | Robot face display | &mdash; | `/speech/text`, `/tts_text` | Pi4 Display |

### AI & Vision (Jetson)

| Package | Description | Publishes | Subscribes |
|---------|-------------|-----------|------------|
| `llm_node` | Llama 3.2 3B inference | `/tts_text` | `/speech/text` |
| `yolov8_ros` | YOLOv8 object detection | `/detected_objects` | `/camera/color/image_raw` |

### Infrastructure

| Package | Description |
|---------|-------------|
| `robot_bringup` | Launch files: `bringup.launch.py` (Pi4 sensors + LiDAR), `cerebro.launch.py` (full system) |
| `ldlidar_stl_ros2` | LD-19 LiDAR driver (third-party package) |

---

## Hardware

| Component | Purpose | Connection |
|-----------|---------|------------|
| Raspberry Pi 4 | Main controller &mdash; sensors, display, TTS | &mdash; |
| NVIDIA Jetson Orin Nano | LLM inference, vision, STT | Ethernet to Pi4 |
| Arduino Uno (x2) | Motor control, analog sensors | USB Serial to Pi4 |
| BNO055 IMU | Orientation & motion | I2C |
| BMP180 | Barometric pressure & temperature | I2C |
| GT-U7 GPS | Outdoor positioning | Serial |
| LD-19 LiDAR | 360-degree distance scanning | Serial |
| Intel RealSense D435 | RGB + Depth camera | USB (Jetson) |
| 4x Mecanum Wheels + Motors | Omnidirectional movement | L293D via Arduino |
| Rain Sensor | Weather detection | Analog via Arduino |
| Soil Moisture Sensor | Environmental monitoring | Analog via Arduino |
| ReSpeaker Microphone | Voice input | USB Audio |
| MAX98357A DAC + Speaker | Voice output | I2S |
| Elecrow Touchscreen | Robot face display | HDMI/DSI |

---

## Getting Started

### Prerequisites

**Raspberry Pi 4:**
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Python packages
pip3 install adafruit-circuitpython-bno055 smbus2 pyserial pynmea2 pigpio PyQt5 elevenlabs

# System
sudo apt install pigpiod ffmpeg
```

**Jetson Orin Nano:**
```bash
sudo apt install ros-humble-desktop
pip3 install llama-cpp-python ultralytics
# Place Llama-3.2-3B-Instruct-Q4_K_M.gguf in ~/models/
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

# For cross-machine Jetson <-> Pi communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

### Launch

```bash
# Full robot (Pi4 — sensors, LiDAR, camera)
ros2 launch robot_bringup bringup.launch.py

# Full system (Jetson — AI pipeline)
ros2 launch robot_bringup cerebro.launch.py

# Or use the quick-start script
./scripts/start_robot.sh
```

---

## Quick Start &mdash; Individual Components

```bash
# IMU
ros2 run bno055_imu bno055_node

# GPS
ros2 launch gps_gt_u7 gps_gt_u7.launch.py

# Mecanum drive (requires Arduino)
ros2 launch mecanum_driver mecanum_driver.launch.py

# Motor CLI (standalone, no ROS needed)
python3 tools/motor_cli.py

# Display
ros2 launch llm_display display.launch.py

# TTS
ros2 run tts_elevenlabs tts_elevenlabs_node

# Keyboard teleop (drive the robot)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Useful Topics

```bash
ros2 topic list                      # See all active topics

ros2 topic echo /speech/text         # STT output
ros2 topic echo /tts_text            # LLM responses
ros2 topic echo /imu/data            # IMU readings
ros2 topic echo /gps/fix             # GPS position
ros2 topic echo /cmd_vel             # Movement commands
ros2 topic echo /wheel_speeds        # Motor feedback
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
