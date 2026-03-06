# AI-SHA &mdash; Smart Agricultural & School Assistant Robot

> A multi-platform robot built by high-school students at the International School of Choueifat, Sharjah.
> AI-SHA navigates autonomously, answers questions, monitors crops, detects plant diseases,
> and performs smart farming tasks using voice interaction, computer vision, and autonomous navigation.

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
├── plant_disease_training/  PC workstation training pipeline (RTX GPU)
│   ├── model.py                     MobileNetV3-Small classifier (15 classes)
│   ├── prepare_dataset.py           Dataset download, parsing, 80/10/10 split
│   ├── train.py                     AMP training loop, AUC, early stopping
│   ├── evaluate.py                  Test-set evaluation & confusion matrix
│   ├── export_onnx.py               PyTorch → ONNX export + onnxsim
│   ├── build_tensorrt.py            ONNX → TRT FP16 engine (workstation)
│   ├── test_inference.py            4-mode accuracy / benchmark / pipeline / gallery
│   ├── patch_yolov8_node.py         Idempotent patcher for yolov8_node.py
│   └── requirements.txt             Training environment dependencies
│
├── yolov8_ros/              Jetson YOLOv8 vision node (TensorRT)
│   └── yolov8_ros/
│       ├── yolov8_node.py           Object detection + face + gesture + OCR + disease
│       └── plant_disease_engine.py  TensorRT FP16 plant disease inference wrapper
│
├── robot_brain/             Jetson local LLM + Farm Brain orchestrator
│   ├── robot_brain/
│   │   ├── robot_brain.py           Llama 3.2 3B text generation
│   │   └── farm_brain.py            Farm Brain: autonomous ag orchestrator
│   ├── launch/
│   │   └── farm_brain.launch.py     Full farm pipeline launch
│   └── config/
│       ├── farm_brain_config.yaml   Brain parameters
│       └── farm_locations.json      Named farm waypoints (editable)
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
| `yolov8_ros` | Real-time detection &mdash; YOLOv8m (TensorRT, 30+ FPS). Face detection (MediaPipe), hand gestures (17 types), OCR (EasyOCR), depth estimation. **Plant disease classification** (MobileNetV3 TRT, &lt;2ms) on any detected plant/food COCO class. | pub: `/detection/objects_simple`, `/detection/faces_simple`, `/detection/gestures_simple`, `/detection/ocr_simple`, `/detection/disease_simple` |
| `robot_brain` | Local LLM text generation with vision context integration (Project Cerebro). | sub: `/speech_rec`, `/detection/objects_simple` &rarr; pub: `/speech/text` |
| `robot_brain` (farm_brain) | **Farm Brain orchestrator** &mdash; autonomous agricultural monitoring, LLM intent parsing, Nav2 navigation, sensor fusion, plant disease response, watering/sowing control. | sub: all sensors &rarr; pub: `/cmd_vel`, `/tts_text`, `/farm_brain/*` |
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
| MobileNetV3-Small (PlantVillage) | Plant disease classification | TensorRT FP16 (Jetson GPU) | 1.9ms / image, 99.7% accuracy |

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

# Farm brain &mdash; autonomous agricultural monitoring
ros2 launch robot_brain farm_brain.launch.py

# Farm brain with auto-patrol and auto-water
ros2 launch robot_brain farm_brain.launch.py auto_patrol:=true auto_water:=true

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

# Farm brain (autonomous ag monitor)
ros2 run robot_brain farm_brain

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
ros2 topic echo /detection/disease_simple  # Plant disease classification results
ros2 topic echo /imu/data            # IMU readings
ros2 topic echo /scan                # LiDAR data
ros2 topic echo /cmd_vel             # Movement commands
ros2 topic echo /farm_brain/status   # Farm brain state (JSON)
ros2 topic echo /farm_brain/sensor_summary  # Aggregated sensor data (JSON)
```

---

## Farm Brain &mdash; Autonomous Agricultural Orchestrator

AI-SHA includes a **Farm Brain** node that transforms the robot into a smart agricultural monitor.
It fuses all sensors, uses the LLM for natural-language command interpretation, and drives
autonomous navigation via Nav2.

### Pipeline

```
Voice Command ──► [STT] ──► [LLM] ──► Intent (JSON)
                                            │
                                    ┌───────▼────────┐
                                    │   Farm Brain    │
                                    │  State Machine  │
                                    └──┬──┬──┬──┬──┬─┘
                                       │  │  │  │  │
                        ┌──────────────┘  │  │  │  └──────────────┐
                        ▼                 ▼  ▼  ▼                 ▼
                   [Nav2 Goal]      [Vision] [Sensors]      [Actuators]
                        │               │      │               │
                   Navigate to      Confirm  Soil/Rain/     Water pump
                   farm location    disease  Temp/GPS       Seed dispenser
```

### State Machine

```
IDLE ──► LISTENING ──► PLANNING ──► NAVIGATING ──► INSPECTING ──► ACTION ──► IDLE
                                         │                            │
                                    (Nav2 + Isaac)              WATERING / SOWING
                                                                      │
PATROLLING ──► (visit each section, inspect, repeat) ──► RETURNING_HOME
```

### Voice Commands

| Command | Action |
|---------|--------|
| "Go to row 1" | Navigate to named location |
| "Inspect tomato section" | Navigate + run plant disease detection |
| "Water row 2" | Navigate + activate water pump |
| "Sow pepper section" | Navigate + dispense seeds |
| "Patrol" | Autonomous sweep of all farm sections |
| "Status" / "Report" | Full sensor summary via TTS |
| "Stop" | Emergency stop, cancel all movement |

### Sensor Fusion

The farm brain subscribes to **all** sensor topics and aggregates them into a single snapshot
published at `/farm_brain/sensor_summary` (JSON, 0.1 Hz):

| Sensor | Topic | Data |
|--------|-------|------|
| Soil moisture | `/soil_moisture/moisture` | Moisture % |
| Rain sensor | `/rain_sensor/raining` | Boolean + intensity |
| BMP180 | `/bmp180/temperature`, `/bmp180/pressure` | Temp, pressure, altitude |
| BNO055 IMU | `/imu/data` | Orientation |
| GPS | `/gps/fix` | Lat/lon/alt |
| Odometry | `/odom` | Robot position |
| YOLOv8 | `/detection/objects_simple` | Detected objects |
| Plant Disease | `/detection/disease_simple` | Disease classification |

### Configuration

Edit `robot_brain/config/farm_locations.json` to define your farm layout:

```json
{
  "row_1":          {"x": 2.0, "y": 0.0, "yaw": 0.0},
  "tomato_section": {"x": 4.0, "y": 0.0, "yaw": 0.0}
}
```

### Launch

```bash
# Full farm brain pipeline (all Jetson nodes)
ros2 launch robot_brain farm_brain.launch.py

# With autonomous patrol (visits all sections every 5 min)
ros2 launch robot_brain farm_brain.launch.py auto_patrol:=true

# With auto-watering when soil is dry
ros2 launch robot_brain farm_brain.launch.py auto_water:=true

# Farm brain only (sensors + Nav2 already running)
ros2 run robot_brain farm_brain
```

### Isaac Sim Integration

The farm brain includes a placeholder for an Isaac Sim trained RL navigation policy.
Export your trained model as ONNX/TensorRT and pass it at launch:

```bash
ros2 launch robot_brain farm_brain.launch.py \
    isaac_model_path:=/path/to/isaac_policy.engine
```

### Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/farm_brain/status` | `String` | JSON: state, current action, target |
| `/farm_brain/sensor_summary` | `String` | JSON: all aggregated sensor readings |
| `/farm_brain/water_cmd` | `Bool` | Water pump on/off command |
| `/farm_brain/seed_cmd` | `Bool` | Seed dispenser on/off command |
| `/cmd_vel` | `Twist` | Velocity commands (fallback when Nav2 unavailable) |
| `/tts_text` | `String` | Voice responses sent to TTS |

---

## Plant Disease Classifier

AI-SHA includes a secondary vision head that classifies plant diseases in real time whenever the
YOLOv8 detector finds a plant-related object (COCO classes: `potted plant`, `banana`, `apple`,
`orange`, `broccoli`, `carrot`).

### How it works

```
RealSense D435
    │  (BGR frame)
    ▼
YOLOv8m (TensorRT) ──► detected plant ROI
                                │
                        ┌───────▼────────┐
                        │ MobileNetV3-S  │  ◄── plant_disease_engine.py
                        │  TRT FP16      │       (PyTorch CUDA buffers)
                        │  224×224 input │
                        └───────┬────────┘
                                │
                        ▼ /detection/disease_simple (JSON)
                        {"class":"potted plant",
                         "disease":"Tomato: Late_blight",
                         "conf":0.91, "depth":1.23}
```

### Model

| Property | Value |
|----------|-------|
| Architecture | MobileNetV3-Small |
| Parameters | ~1.1 M |
| Dataset | PlantVillage (15 disease classes — Tomato, Potato, Pepper) |
| Val accuracy | **99.61%** |
| Test accuracy | **99.70%** |
| Jetson latency | **1.9 ms** (batch=1, TRT FP16) |
| Engine size | 3.1 MB |

### Training (on workstation with RTX GPU)

```bash
cd plant_disease_training
pip install -r requirements.txt

# 1 — Download dataset & create 80/10/10 split
python prepare_dataset.py

# 2 — Train (AMP, AdamW, CosineAnnealingWarmRestarts, early stopping)
python train.py

# 3 — Evaluate on held-out test split
python evaluate.py

# 4 — Export to ONNX + simplify
python export_onnx.py

# 5 — Build TensorRT engine (run on the Jetson with trtexec)
trtexec --onnx=checkpoints/plant_disease_classifier_sim.onnx \
        --saveEngine=plant_disease_classifier.engine \
        --fp16 --minShapes=input:1x3x224x224 \
        --optShapes=input:4x3x224x224 --maxShapes=input:8x3x224x224

# 6 — Test accuracy & benchmark (workstation)
python test_inference.py --mode accuracy
python test_inference.py --mode benchmark
python test_inference.py --mode pipeline --save_dir /tmp/results
```

### Deploying on the Jetson

Copy engine and mapping file to the Jetson, then launch the YOLO node with disease params:

```bash
ros2 run yolov8_ros yolov8_node \
  --ros-args \
  -p disease_engine_path:=~/plant_disease_models/plant_disease_classifier.engine \
  -p disease_class_mapping_path:=~/plant_disease_models/class_mapping.json \
  -p enable_disease_classifier:=true \
  -p disease_confidence_threshold:=0.60
```

The annotated video stream (`/detection/image_annotated`) shows a **blue pill label** above each
plant bounding box: `[Tomato: Late_blight  91%]`.

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
