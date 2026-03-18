# AI-SHA Repository Changelog — Code Review Remediation

**Period:** March 2026 (multiple sessions)
**Total PRs Merged:** 43 (#2–#3 on initial branch, #6–#37 on `master`, #1–#6 on `main`)

This document summarizes all changes made to the AI-SHA repository during the iterative external code review remediation process. Each PR addressed specific reviewer comments, with every suggestion validated against the actual codebase before implementation.

---

## Phase 1 — Initial Scaffold & Early Fixes (PRs #2–#3, #6–#12)

### PR #2 — Fix 3 Code-Review Risks
- Fixed hardcoded IPs, VRAM contention, and ROS distro lock.

### PR #3 — Embed Source Packages
- Made the repository fully self-contained by embedding all source packages.

### PR #6 — Fix Concurrency Bugs in TTS, Brain, and WhatsApp Nodes
**Files:** `brain_node.py`, `tts_node.py`, `whatsapp_listener.py`
- Fixed thread-safety issues in the TTS audio pipeline, brain intent routing, and WhatsApp message handling.

### PR #7 — Fix Executor Blocking, Topic Hygiene, and Temp File Collision
**Files:** `stt_node.py`, `tts_node.py`, `plant_disease_node.py`
- Resolved executor blocking in audio nodes and temp file naming collisions in the plant disease classifier.

### PR #8 — Fix admin_node RAG Bugs: Crash, RAM, Offline, Prompt Structure
**Files:** `admin_node.py`, `build_knowledge.py`
- Fixed crash on empty knowledge base, excessive RAM usage during embedding, offline model fallback, and prompt structure issues.

### PR #9 — Fix Concurrency, Hardware Recovery, and Safety Across 5 Nodes
**Files:** `action_node.py`, `admin_node.py`, `aisha_launch.py`, `bno055_node.py`, `mecanum_driver_node.py`
- Addressed race conditions and hardware recovery across all major nodes. Added safety guards for IMU I2C errors and motor driver serial disconnects.

### PR #10 — Upgrade RAG: BGE Embeddings, Semantic Chunking, Metadata
**Files:** `admin_node.py`, `build_knowledge.py`
- Switched from generic embeddings to `BAAI/bge-small-en-v1.5` for superior retrieval accuracy. Implemented section-aware semantic chunking with `file_name` and `section` metadata for grade-filtered queries.

### PR #11 — Fix Audio Race, History Tracking, IndexError, Wake Word Ack
**Files:** `admin_node.py`, `brain_node.py`, `stt_node.py`, `tts_node.py`
- Fixed race condition between STT and TTS audio streams. Resolved history tracking IndexError and wake word acknowledgment timing.

### PR #12 — Add Pi 4b Support and 4-Device Architecture
**Files:** `.env.example`, `README.md`, `generate_fastdds_configs.sh`, `aisha_launch.py`, `rpi_launch.py`, `pi4_motor_launch.py`
- Extended architecture from 3 devices (Jetson, RPi 5, Workstation) to 4 by adding Raspberry Pi 4b as a dedicated motor controller. Created FastDDS config generator and Pi 4b motor launch file.

---

## Phase 2 — Hardware Integration & Stability (PRs #13–#20)

### PR #13 — Fix Hardware Mapping, Kinematics, Serial Blocking, Thread Safety
**Files:** `jetson_launch.py`, `rpi_launch.py`, `pi4_motor_launch.py`, `mecanum_driver_node.py`, `plant_disease_engine.py`
- Corrected mecanum wheel kinematics formulas, fixed serial port blocking during Arduino reconnection, and resolved thread-safety issues in the motor driver.

### PR #14 — Harden Cognitive Layer: LLM-First Routing, SABIS Levels, Refusal Reminder
**Files:** `admin_node.py`, `brain_node.py`
- Switched intent classification from keyword-first to LLM-first routing via Gemma 3 (270M). Added SABIS level support (A–L) alongside numeric grades. Strengthened academic refusal rules in the system prompt.

### PR #15 — Fix Middleware Conflict, Serial Race, FastDDS 4-Device Mesh, L293D Warning
**Files:** `fastdds_*.xml`, `rpi_launch.py`, `mecanum_motor_control.ino`, `mecanum_driver_node.py`, `cerebro.launch.py`
- Resolved RMW middleware conflict between CycloneDDS and FastDDS. Fixed serial write race condition. Established initial 4-device FastDDS unicast mesh (single-participant per device; multi-participant discovery hardened in Phase 5 PR #1). Added L293D thermal shutdown warning to Arduino firmware.

### PR #16 — Fix RAG Data Bugs: Grade Boundary, Intro Loss, school_facts Format, VRAM
**Files:** `admin_node.py`, `brain_node.py`, `build_knowledge.py`, `school_facts.md`, `school_facts.txt`
- Fixed grade boundary regex matching ("Grade 1" matching "Grade 10"). Prevented `build_knowledge.py` from losing intro paragraphs. Restructured `school_facts` data format for better chunking. Added VRAM management guards.

### PR #17 — Fix Launch Conflicts, IMU Pipeline, Serial Blocking, VRAM Thrashing
**Files:** `brain_node.py`, `rpi_launch.py`, `mecanum_driver_node.py`, `bringup.launch.py`
- Resolved conflicting launch files between `robot_bringup` and `aisha_integration`. Fixed IMU data pipeline on RPi 5. Eliminated VRAM thrashing from concurrent model loads.

### PR #18 — Fix History Corruption, Pin 13 Twitch, FastDDS Safety, Motor Mirroring
**Files:** `brain_node.py`, `tts_node.py`, `jetson_launch.py`, `rpi_launch.py`, `mecanum_motor_control.ino`, `mecanum_driver_node.py`
- Fixed conversation history corruption under concurrent access. Resolved Arduino pin 13 LED twitch on boot (shared with RL_IN2). Added FastDDS discovery safety checks. Implemented software motor mirroring for right-side motors.

### PR #19 — Fix Stale Queue, Ollama URL, FastDDS Comments, Static IP Docs
**Files:** `.env.example`, `fastdds_rpi.xml`, `generate_fastdds_configs.sh`, `brain_node.py`
- Fixed stale question queue accumulation. Corrected Ollama API URL format. Added static IP documentation for all devices.

### PR #20 — Fix Deque Bug, keep_alive Docs, Disable Disease Classifier
**Files:** `brain_node.py`, `jetson_launch.py`
- Fixed deque rotation bug in pending question tracking. Documented `keep_alive` model management strategy. Disabled plant disease classifier by default to save ~0.3 GB VRAM.

---

## Phase 3 — Motor Safety & VRAM Optimization (PRs #21–#27)

### PR #21 — Fix Executor-Blocking Reconnect, Label FastDDS Peer IPs
**Files:** `fastdds_*.xml`, `generate_fastdds_configs.sh`, `mecanum_driver_node.py`
- Replaced blocking `connect_serial()` retry loop with non-blocking `_reconnect_timer` (1 Hz). Added human-readable IP labels to FastDDS peer entries.

### PR #22 — Fix Runaway-Robot Watchdog, Rename speed_to_pwm Arg, Fix Hardcoded Paths
**Files:** `jetson_launch.py`, `rpi_launch.py`, `mecanum_driver_node.py`
- Implemented redundant stop counter in watchdog (10 sends × 0.1s) to survive dropped USB serial packets. Fixed hardcoded file paths in launch files.

### PR #23 — Fix Kinematics Vector Distortion, History Race, Watchdog Counter
**Files:** `fastdds_jetson.xml`, `generate_fastdds_configs.sh`, `brain_node.py`, `mecanum_driver_node.py`
- Fixed proportional wheel speed normalization (previously independent clamping distorted motion vectors). Introduced `_history_lock` threading lock for brain_node conversation history. Reset watchdog stop counter on new `cmd_vel`.

### PR #24 — Fix History Desync Race, Add Motor Deadband, Admin keep_alive, Type Safety
**Files:** `admin_node.py`, `brain_node.py`, `mecanum_driver_node.py`
- Fixed history desync race between intent routing and response recording. Added `min_motor_pwm` deadband compensation for static friction. Set admin_node Ollama `keep_alive=30s` for VRAM recycling. Added explicit type casting for all ROS parameters. (Serial/history fixes in this phase are incremental tightening of races first surfaced in PRs #11–#13.)

### PR #25 — Add Dummy Odometry for SLAM, Fix Serial Race, Expire Stale History
**Files:** `brain_node.py`, `jetson_launch.py`, `mecanum_driver_node.py`
- Created `dummy_odom.py` publishing identity `odom→base_link` TF at 50 Hz for slam_toolbox. Fixed serial write race with `serial_lock`. Added initial stale history expiry (response-triggered; decoupled from response loop in PR #33).

### PR #26 — Fix VRAM Overlap, brain_responding Race, top_k Overflow, TF/ALSA Docs
**Files:** `brain_node.py`, `jetson_launch.py`, `rpi_launch.py`
- Fixed VRAM overlap between Gemma 3 router and Llama 3.2 inference by skipping LLM routing while ADMIN questions are pending. Fixed `brain_responding` flag race. Capped `similarity_top_k` to prevent context window overflow.

### PR #27 — Tag Pending Questions with Intent, Add LiDAR TF, Delete Legacy Launch Files
**Files:** `brain_node.py`, `jetson_launch.py`, `rpi_launch.py`, `bringup.launch.py`, `cerebro.launch.py`
- Tagged pending questions with intent type for better debugging. Added `base_link→laser` static TF to launch. Deleted legacy standalone launch files superseded by integrated launches.

---

## Phase 4 — Production Hardening (PRs #28–#37)

### PR #28 — Guard VRAM Contention, Document chrony/SLAM Limits, Fix Regex Log
**Files:** `brain_node.py`, `jetson_launch.py`, `slam_toolbox.yaml`
- Added VRAM contention guards between concurrent model requests. Documented chrony NTP sync requirement for multi-SBC TF. Fixed regex log gap in intent classification.

### PR #29 — Fix Parameter Type Mismatch in pi4_motor_launch.py and mecanum_driver
**Files:** `pi4_motor_launch.py`, `mecanum_driver_node.py`
- Fixed ROS 2 `LaunchConfiguration` string-to-numeric type mismatch. All numeric parameters now explicitly cast with `int()`/`float()`.

### PR #30 — Pass Ollama VRAM Limits in API, Add History Thread Lock, Stagger Launch
**Files:** `admin_node.py`, `brain_node.py`, `jetson_launch.py`
- Passed `OLLAMA_NUM_GPU` and `OLLAMA_NUM_CTX` as per-request API overrides (not env vars, which only affect `ollama serve`). Extended `_history_lock` scope to cover new brain_node code paths added since PR #23. Staggered cognitive node startup (5s/8s/11s) to prevent thundering herd.

### PR #31 — Add Emergency Stop Bypass Before Ollama, Document Serial Permissions
**Files:** `brain_node.py`, `pi4_motor_launch.py`
- Emergency keywords ("stop", "halt", "freeze") now bypass Ollama entirely for instant response. Documented `/dev/ttyACM0` serial permission requirements.

### PR #32 — Reset Watchdog Stop Counter on New cmd_vel, Document QoS and Token Budget
**Files:** `jetson_launch.py`, `rpi_launch.py`, `mecanum_driver_node.py`
- Hardened watchdog stop counter reset (PR #23 introduced the counter; this PR fixed an edge case where a new `cmd_vel` arriving mid-stop-sequence didn't clear `_stop_sends_remaining`, causing brief stutter). Documented QoS settings and LLM token budget.

### PR #33 — Fix I2C Bus Param, Proactive Stale Expiry, SLAM QoS Compatibility
**Files:** `brain_node.py`, `bno055_node.py`, `slam_toolbox.yaml`, `slam_toolbox_simple.yaml`
- Fixed I2C bus parameter (bus 1 vs bus 0) for BNO055 on RPi 5. Decoupled stale question expiry from the response callback (PR #25's expiry only ran when responses arrived; this PR added a standalone timer so orphaned questions are cleaned up even if the LLM never responds). Tuned SLAM QoS for reliability.

### PR #34 — Fix Intent Race Condition with Separate Response Topics, Delete Legacy Launch
**Files:** `action_node.py`, `admin_node.py`, `brain_node.py`, `aisha_launch.py`
- Split `/robot_speech` into separate `/admin_response` and `/action_response` topics to prevent intent response cross-contamination. Deleted legacy monolithic launch file.

### PR #35 — Fix Leftover _pending_questions Crash, Improve WhatsApp Auth Error Message
**Files:** `action_node.py`, `brain_node.py`
- Fixed crash from leftover `_pending_questions` reference after UUID migration. Improved WhatsApp authentication error messages.

### PR #36 — Add ROS_DOMAIN_ID Checks Across All SBCs, Decouple cmd_vel from Serial Writes
**Files:** `jetson_launch.py`, `rpi_launch.py`, `pi4_motor_launch.py`, `mecanum_driver_node.py`
- Added `ROS_DOMAIN_ID` consistency check at launch (warns if unset or mismatched). Decoupled cmd_vel callback from serial writes via 20 Hz `_flush_serial` timer to prevent Arduino UART overflow.

### PR #37 — Replace dummy_odom with rf2o Laser Odometry, Switch Whisper to int8, Add L293D Warning
**Files:** `jetson_launch.py`, `pi4_motor_launch.py`
- Added `odom_source` launch arg (default `'laser'`) switching the default odometry source from dummy_odom to rf2o_laser_odometry for scan-based odom. `dummy_odom.py` retained as a fallback (`odom_source:='dummy'`). Switched Whisper from float16 to int8 (saves ~500 MB VRAM).

---

## Phase 5 — Final Remediation on `main` (PRs #1–#6, March 18, 2026)

### PR #1 — Fix FastDDS Discovery, UUID Intent Tracking, Watchdog Zombie Race
**Files:** `brain_node.py`, `admin_node.py`, `action_node.py`, `mecanum_driver_node.py`, `generate_fastdds_configs.sh`, `config/fastdds_*.xml`

- **FastDDS multi-participant discovery:** Created `generate_fastdds_configs.sh` computing explicit metatraffic ports via the DDS well-known port formula (`port = 7400 + 250 × DOMAIN_ID + 10 + 2 × PARTICIPANT_ID`). Generated 4 XML configs with 23 locator entries each.
- **UUID intent tracking:** Replaced deque FIFO with UUID-keyed dictionaries. Each `query_id` flows through the full brain→admin/action→brain pipeline. Backward compatible with plain-text responses.
- **Watchdog zombie fix:** Clear `_latest_pwm` and `_pwm_dirty` on the moving→stopped transition to prevent `_flush_serial` from resurrecting stale velocity commands.

### PR #2 — Add Knowledge Base Builder and CIE/AP Exam Timetable 2026
**Files:** `build_knowledge.py`, `cie_ap_exams_2026.md`

- Created `build_knowledge.py`: Markdown `##` header splitter → per-bullet chunks with section context → `BAAI/bge-small-en-v1.5` embeddings → ChromaDB `school_info` collection.
- Created `cie_ap_exams_2026.md` with 68 exam entries (IGCSE 29, AS/A Level 23, AP 16).

### PR #3 — Document NAV Architecture and UDP Buffer Requirements
**Files:** `brain_node.py`, `generate_fastdds_configs.sh`

- Documented Phase 1 NAV architecture: `waypoint_resolver` → `NavigateToPose` → nav2 → `/cmd_vel` → mecanum_driver.
- Added `sysctl` UDP buffer documentation (Linux ~212 KB default vs FastDDS 4 MB request).

### PR #4 — Fix URDF Frame Mismatch, Add RAG Relevance Filter, Document Odom Path
**Files:** `robot.urdf`, `admin_node.py`, `dummy_odom.py`, `mecanum_driver_node.py`

- **URDF fix:** Renamed LiDAR frame `base_laser` → `laser` to match slam.launch.py and ld19.yaml.
- **RAG filter:** Added `relevance_distance_threshold` (default 1.0) to discard irrelevant chunks before LLM. Out-of-scope queries short-circuit without Ollama.
- **Odom docs:** Documented 3-step migration path (rf2o → encoders → EKF) in dummy_odom.py.

### PR #5 — Add EKF Config and Encoder Serial Protocol Parser
**Files:** `ekf.yaml`, `mecanum_driver_node.py`

- Created `robot_bringup/config/ekf.yaml` fusing BNO055 IMU orientation + angular velocity with future wheel odom. IMU linear acceleration excluded (MEMS drift).
- Added `_parse_encoder_line()` defining the serial contract: `E <fl_ticks> <fr_ticks> <rl_ticks> <rr_ticks>\n`.

### PR #6 — Implement Mecanum Forward Kinematics and Encoder-Based Odometry
**Files:** `mecanum_driver_node.py`, `mecanum_motor_control.ino`

- Implemented `_update_odometry()`: tick deltas → wheel angular displacement → Mecanum FK (vx, vy, wz) → midpoint-theta integration → `nav_msgs/Odometry` + `odom→base_link` TF.
- Dual encoder sources: serial `E` protocol (future Arduino Mega) + ROS `/encoders/position` topic (existing RPi pigpio).
- Gated behind `publish_odom` parameter (default `False`).
- Documented Arduino Uno pin exhaustion (all 12 digital pins used by L293D) and 3 upgrade paths.

---

## Summary of Key Architecture Decisions

1. **4-device mesh:** Jetson Orin Nano (AI/cognition) + RPi 5 (sensors/audio) + RPi 4b (motor drivers) + Arduino Uno (PWM output), connected via FastDDS unicast.
2. **100% offline:** No cloud APIs. Gemma 3 270M (intent router) + Llama 3.2 (RAG inference) + Faster-Whisper (STT) + Piper (TTS), all on-device.
3. **VRAM management:** Gemma 3 uses `keep_alive=0` (immediate unload), Llama 3.2 uses `keep_alive=30s`, staggered launch prevents thundering herd on 8 GB shared memory.
4. **Encoders on RPi, not Arduino:** Uno has no free pins. Quadrature decoding via pigpio on RPi 5 GPIO.
5. **UUID intent tracking:** Eliminates FIFO response-question mispairing under concurrent LLM inference.
6. **RAG short-circuit:** Cosine distance threshold skips Ollama for out-of-scope queries (saves 5-30s per irrelevant question).

## New ROS Parameters (Phase 5)

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| `admin_node` | `relevance_distance_threshold` | 1.0 | Cosine distance cutoff for RAG chunks |
| `mecanum_driver` | `encoder_cpr` | 2400 | Encoder counts per revolution (4× quadrature) |
| `mecanum_driver` | `publish_odom` | False | Enable encoder-based odometry publishing |

## Remaining Steps to Full Autonomy

All remaining work requires **physical deployment** (no software-only changes left):

1. **Physical SLAM mapping run** — Drive robot through school corridors with slam_toolbox + teleop to generate `.pgm`/`.yaml` map files.
2. **Nav2 action client** — Connect brain_node NAV intent to nav2's `NavigateToPose` action server.
3. **Waypoint calibration** — Record `[x, y, θ]` coordinates for school locations in `nav_locations.json` using RViz.
4. **Nav2 parameter tuning** — Configure costmaps, trajectory planner, and obstacle inflation radius via physical testing.
