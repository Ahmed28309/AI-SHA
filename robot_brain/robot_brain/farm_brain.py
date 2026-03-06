#!/usr/bin/env python3
"""
AI-SHA Farm Brain - Autonomous Agricultural Robot Orchestrator

Main brain node that coordinates all subsystems for smart farming:
  - LLM interprets voice commands into intent (navigate, inspect, water, sow, patrol)
  - Converts intent to Nav2 goal locations
  - Sends goals to Nav2 for autonomous movement
  - Vision (YOLOv8 + plant disease) confirms presence and conditions
  - Isaac Sim trained model provides sim-to-real navigation policy
  - Aggregates all environmental sensors for decision making

State Machine:
  IDLE -> LISTENING -> PLANNING -> NAVIGATING -> INSPECTING -> ACTION -> IDLE
                                                                |
                                                          WATERING / SOWING

Runs on: Jetson Orin Nano (ROS 2 Humble)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, LaserScan, FluidPressure, Temperature, Imu
from nav_msgs.msg import Odometry

# Nav2 action interfaces - install nav2_msgs
try:
    from nav2_msgs.action import NavigateToPose
    _NAV2_AVAILABLE = True
except ImportError:
    _NAV2_AVAILABLE = False

import json
import threading
import time
import os
import math
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
class BrainState(Enum):
    IDLE = auto()
    LISTENING = auto()
    PLANNING = auto()
    NAVIGATING = auto()
    INSPECTING = auto()
    WATERING = auto()
    SOWING = auto()
    PATROLLING = auto()
    RETURNING_HOME = auto()
    ERROR = auto()


# ---------------------------------------------------------------------------
# Sensor snapshot
# ---------------------------------------------------------------------------
@dataclass
class SensorSnapshot:
    """Aggregated sensor readings at a point in time."""
    # Soil
    soil_moisture_pct: float = 0.0
    soil_is_dry: bool = False
    # Rain
    rain_intensity_pct: float = 0.0
    is_raining: bool = False
    # Environment
    temperature_c: float = 0.0
    pressure_pa: float = 0.0
    altitude_m: float = 0.0
    humidity_pct: float = 0.0          # DHT11 (future)
    # Position
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_alt: float = 0.0
    gps_fix: bool = False
    # IMU
    imu_orientation_yaw: float = 0.0
    # Odometry
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    # Vision
    detected_objects: List[Dict] = field(default_factory=list)
    detected_diseases: List[Dict] = field(default_factory=list)
    # Timestamps
    last_update: float = 0.0


# ---------------------------------------------------------------------------
# Intent from LLM
# ---------------------------------------------------------------------------
@dataclass
class FarmIntent:
    """Parsed intent from the LLM."""
    action: str = ''           # navigate, inspect, water, sow, patrol, status, stop
    target: str = ''           # e.g. "row 3", "tomato section", "home"
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    parameters: Dict = field(default_factory=dict)
    raw_command: str = ''
    confidence: float = 0.0


# ---------------------------------------------------------------------------
# Known farm locations (template - configure per farm)
# ---------------------------------------------------------------------------
DEFAULT_FARM_LOCATIONS = {
    'home':             {'x': 0.0,  'y': 0.0,  'yaw': 0.0},
    'charging_station': {'x': 0.0,  'y': 0.0,  'yaw': 0.0},
    'row_1':            {'x': 2.0,  'y': 0.0,  'yaw': 0.0},
    'row_2':            {'x': 2.0,  'y': 2.0,  'yaw': 0.0},
    'row_3':            {'x': 2.0,  'y': 4.0,  'yaw': 0.0},
    'tomato_section':   {'x': 4.0,  'y': 0.0,  'yaw': 0.0},
    'potato_section':   {'x': 4.0,  'y': 2.0,  'yaw': 0.0},
    'pepper_section':   {'x': 4.0,  'y': 4.0,  'yaw': 0.0},
    'water_source':     {'x': -1.0, 'y': 0.0,  'yaw': 3.14},
    'seed_storage':     {'x': -1.0, 'y': 2.0,  'yaw': 3.14},
}

# Patrol waypoints for autonomous monitoring
DEFAULT_PATROL_WAYPOINTS = [
    'row_1', 'row_2', 'row_3',
    'tomato_section', 'potato_section', 'pepper_section',
]


# ---------------------------------------------------------------------------
# Main brain node
# ---------------------------------------------------------------------------
class FarmBrain(Node):
    def __init__(self):
        super().__init__('farm_brain')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('farm_locations_file', '')
        self.declare_parameter('patrol_interval_sec', 300.0)
        self.declare_parameter('soil_dry_threshold', 30.0)
        self.declare_parameter('auto_water_enabled', False)
        self.declare_parameter('auto_patrol_enabled', False)
        self.declare_parameter('nav2_enabled', True)
        self.declare_parameter('isaac_model_path', '')
        self.declare_parameter('llm_topic', '/tts_text')
        self.declare_parameter('stt_topic', '/speech/text')

        self.patrol_interval = self.get_parameter('patrol_interval_sec').value
        self.soil_dry_threshold = self.get_parameter('soil_dry_threshold').value
        self.auto_water = self.get_parameter('auto_water_enabled').value
        self.auto_patrol = self.get_parameter('auto_patrol_enabled').value
        self.nav2_enabled = self.get_parameter('nav2_enabled').value
        self.isaac_model_path = self.get_parameter('isaac_model_path').value
        self.llm_topic = self.get_parameter('llm_topic').value
        self.stt_topic = self.get_parameter('stt_topic').value

        # ── State ─────────────────────────────────────────────────────────
        self.state = BrainState.IDLE
        self.state_lock = threading.Lock()
        self.sensors = SensorSnapshot()
        self.sensor_lock = threading.Lock()
        self.current_intent: Optional[FarmIntent] = None
        self.patrol_index = 0
        self.last_patrol_time = 0.0

        # Farm locations
        self.farm_locations = dict(DEFAULT_FARM_LOCATIONS)
        locations_file = self.get_parameter('farm_locations_file').value
        if locations_file and os.path.exists(locations_file):
            self._load_farm_locations(locations_file)

        # Isaac Sim model placeholder
        self.isaac_model = None
        if self.isaac_model_path and os.path.exists(self.isaac_model_path):
            self._load_isaac_model()

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tts_pub = self.create_publisher(String, self.llm_topic, 10)
        self.status_pub = self.create_publisher(String, '/farm_brain/status', 10)
        self.sensor_summary_pub = self.create_publisher(
            String, '/farm_brain/sensor_summary', 10)
        # Actuator commands (template - connect to actual hardware)
        self.water_cmd_pub = self.create_publisher(Bool, '/farm_brain/water_cmd', 10)
        self.seed_cmd_pub = self.create_publisher(Bool, '/farm_brain/seed_cmd', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        # Voice input (from STT node)
        self.create_subscription(
            String, self.stt_topic, self._on_speech, 10)

        # LLM parsed intent (from llm_node or robot_brain)
        self.create_subscription(
            String, '/farm_brain/intent', self._on_intent, 10)

        # Vision
        self.create_subscription(
            String, '/detection/objects_simple', self._on_detections, 10)
        self.create_subscription(
            String, '/detection/disease_simple', self._on_disease, 10)

        # Soil moisture (from RPi sensor node)
        self.create_subscription(
            Float32, '/soil_moisture/moisture', self._on_soil_moisture, 10)
        self.create_subscription(
            Bool, '/soil_moisture/dry', self._on_soil_dry, 10)

        # Rain sensor (from RPi sensor node)
        self.create_subscription(
            Float32, '/rain_sensor/intensity', self._on_rain_intensity, 10)
        self.create_subscription(
            Bool, '/rain_sensor/raining', self._on_raining, 10)

        # Environment (BMP180 on RPi)
        self.create_subscription(
            Temperature, '/bmp180/temperature', self._on_temperature, 10)
        self.create_subscription(
            FluidPressure, '/bmp180/pressure', self._on_pressure, 10)

        # GPS
        gps_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            NavSatFix, '/gps/fix', self._on_gps, gps_qos)

        # IMU (BNO055 on RPi)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)

        # Odometry (from motor encoders on RPi4)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # LiDAR scan (for proximity checks)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        # ── Nav2 action client ────────────────────────────────────────────
        self.nav2_client = None
        if _NAV2_AVAILABLE and self.nav2_enabled:
            self.nav2_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info('Nav2 action client created')

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._tick)                  # Main loop at 1 Hz
        self.create_timer(10.0, self._publish_sensor_summary)
        self.create_timer(5.0, self._publish_status)

        # ── Startup ───────────────────────────────────────────────────────
        self.get_logger().info('=' * 60)
        self.get_logger().info('  AI-SHA FARM BRAIN - Autonomous Agricultural Monitor')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Nav2:        {"enabled" if self.nav2_client else "disabled"}')
        self.get_logger().info(f'  Auto patrol: {"ON" if self.auto_patrol else "OFF"}')
        self.get_logger().info(f'  Auto water:  {"ON" if self.auto_water else "OFF"}')
        self.get_logger().info(f'  Isaac model: {self.isaac_model_path or "none"}')
        self.get_logger().info(f'  Locations:   {len(self.farm_locations)} defined')
        self.get_logger().info('=' * 60)

    # ======================================================================
    # Config helpers
    # ======================================================================
    def _load_farm_locations(self, path: str):
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            self.farm_locations.update(data)
            self.get_logger().info(
                f'Loaded {len(data)} farm locations from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load farm locations: {e}')

    def _load_isaac_model(self):
        """
        TODO: Load Isaac Sim trained navigation / control model.
        This could be an ONNX/TensorRT policy exported from Isaac Sim
        that provides learned locomotion or obstacle avoidance.
        """
        self.get_logger().info(
            f'TODO: Load Isaac Sim model from {self.isaac_model_path}')
        # Example placeholder:
        # import tensorrt as trt
        # self.isaac_model = load_trt_engine(self.isaac_model_path)

    # ======================================================================
    # Sensor callbacks
    # ======================================================================
    def _on_soil_moisture(self, msg: Float32):
        with self.sensor_lock:
            self.sensors.soil_moisture_pct = msg.data

    def _on_soil_dry(self, msg: Bool):
        with self.sensor_lock:
            self.sensors.soil_is_dry = msg.data

    def _on_rain_intensity(self, msg: Float32):
        with self.sensor_lock:
            self.sensors.rain_intensity_pct = msg.data

    def _on_raining(self, msg: Bool):
        with self.sensor_lock:
            self.sensors.is_raining = msg.data

    def _on_temperature(self, msg: Temperature):
        with self.sensor_lock:
            self.sensors.temperature_c = msg.temperature

    def _on_pressure(self, msg: FluidPressure):
        with self.sensor_lock:
            self.sensors.pressure_pa = msg.fluid_pressure

    def _on_gps(self, msg: NavSatFix):
        with self.sensor_lock:
            self.sensors.gps_lat = msg.latitude
            self.sensors.gps_lon = msg.longitude
            self.sensors.gps_alt = msg.altitude
            self.sensors.gps_fix = msg.status.status >= 0

    def _on_imu(self, msg: Imu):
        with self.sensor_lock:
            # Extract yaw from quaternion
            q = msg.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.sensors.imu_orientation_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_odom(self, msg: Odometry):
        with self.sensor_lock:
            self.sensors.odom_x = msg.pose.pose.position.x
            self.sensors.odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.sensors.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_scan(self, msg: LaserScan):
        # Store min range for proximity alerts (template)
        pass

    def _on_detections(self, msg: String):
        with self.sensor_lock:
            try:
                self.sensors.detected_objects = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                self.sensors.detected_objects = []

    def _on_disease(self, msg: String):
        with self.sensor_lock:
            try:
                data = json.loads(msg.data)
                if isinstance(data, list):
                    self.sensors.detected_diseases = data
                else:
                    self.sensors.detected_diseases = [data]
            except (json.JSONDecodeError, TypeError):
                self.sensors.detected_diseases = []

    # ======================================================================
    # Voice / intent handling
    # ======================================================================
    def _on_speech(self, msg: String):
        """
        Receive raw voice command from STT, forward to LLM for intent parsing.
        The LLM should return a JSON intent on /farm_brain/intent.
        If no separate LLM intent parser is running, do basic keyword matching.
        """
        command = msg.data.strip()
        if not command:
            return

        self.get_logger().info(f'Voice command: "{command}"')

        # Forward to LLM for parsing (the LLM node publishes parsed intent)
        # As fallback, do local keyword-based intent extraction
        intent = self._extract_intent_local(command)
        if intent:
            self._execute_intent(intent)

    def _on_intent(self, msg: String):
        """Receive parsed intent from LLM or external planner."""
        try:
            data = json.loads(msg.data)
            intent = FarmIntent(
                action=data.get('action', ''),
                target=data.get('target', ''),
                goal_x=data.get('goal_x', 0.0),
                goal_y=data.get('goal_y', 0.0),
                goal_yaw=data.get('goal_yaw', 0.0),
                parameters=data.get('parameters', {}),
                raw_command=data.get('raw_command', ''),
                confidence=data.get('confidence', 0.0),
            )
            self._execute_intent(intent)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f'Bad intent JSON: {e}')

    def _extract_intent_local(self, command: str) -> Optional[FarmIntent]:
        """
        Fallback keyword-based intent extraction when LLM parser is offline.
        In production, the LLM node should parse commands and publish to
        /farm_brain/intent as structured JSON.
        """
        cmd = command.lower()
        intent = FarmIntent(raw_command=command)

        # Stop
        if any(w in cmd for w in ['stop', 'halt', 'cancel', 'abort']):
            intent.action = 'stop'
            return intent

        # Status / report
        if any(w in cmd for w in ['status', 'report', 'sensor', 'condition',
                                   'how are', 'what is']):
            intent.action = 'status'
            return intent

        # Patrol
        if any(w in cmd for w in ['patrol', 'monitor', 'survey', 'scan all',
                                   'check all']):
            intent.action = 'patrol'
            return intent

        # Water
        if any(w in cmd for w in ['water', 'irrigate']):
            intent.action = 'water'
            for loc_name in self.farm_locations:
                if loc_name.replace('_', ' ') in cmd:
                    intent.target = loc_name
                    break
            return intent

        # Sow / plant
        if any(w in cmd for w in ['sow', 'plant', 'seed']):
            intent.action = 'sow'
            for loc_name in self.farm_locations:
                if loc_name.replace('_', ' ') in cmd:
                    intent.target = loc_name
                    break
            return intent

        # Navigate / go to
        if any(w in cmd for w in ['go to', 'navigate', 'move to', 'drive to',
                                   'head to', 'go home', 'return']):
            intent.action = 'navigate'
            if 'home' in cmd or 'return' in cmd:
                intent.target = 'home'
            else:
                for loc_name in self.farm_locations:
                    if loc_name.replace('_', ' ') in cmd:
                        intent.target = loc_name
                        break
            return intent

        # Inspect
        if any(w in cmd for w in ['inspect', 'check', 'look at', 'examine',
                                   'disease']):
            intent.action = 'inspect'
            for loc_name in self.farm_locations:
                if loc_name.replace('_', ' ') in cmd:
                    intent.target = loc_name
                    break
            return intent

        return None

    # ======================================================================
    # Intent execution
    # ======================================================================
    def _execute_intent(self, intent: FarmIntent):
        self.current_intent = intent
        self.get_logger().info(
            f'Executing intent: action={intent.action} target={intent.target}')

        if intent.action == 'stop':
            self._do_stop()
        elif intent.action == 'status':
            self._do_status_report()
        elif intent.action == 'navigate':
            self._do_navigate(intent.target)
        elif intent.action == 'inspect':
            self._do_navigate(intent.target, then_inspect=True)
        elif intent.action == 'water':
            self._do_navigate(intent.target, then_water=True)
        elif intent.action == 'sow':
            self._do_navigate(intent.target, then_sow=True)
        elif intent.action == 'patrol':
            self._do_start_patrol()
        else:
            self._speak(f'I do not understand the action: {intent.action}')

    def _do_stop(self):
        with self.state_lock:
            self.state = BrainState.IDLE
        # Cancel Nav2 goal if active
        if self.nav2_client and self.nav2_client.is_server_ready():
            self.nav2_client._cancel_goal_async()  # type: ignore
        # Stop motors
        self.cmd_vel_pub.publish(Twist())
        self._speak('Stopping. All movement cancelled.')

    def _do_status_report(self):
        with self.sensor_lock:
            s = self.sensors
        report = (
            f'Farm status report. '
            f'Soil moisture is {s.soil_moisture_pct:.0f} percent. '
            f'{"Soil is dry, watering may be needed. " if s.soil_is_dry else ""}'
            f'Temperature is {s.temperature_c:.1f} degrees. '
            f'{"It is raining. " if s.is_raining else "No rain detected. "}'
            f'GPS {"fix acquired" if s.gps_fix else "no fix"}. '
            f'Position: x={s.odom_x:.1f}, y={s.odom_y:.1f}. '
        )
        # Add disease info if any
        if s.detected_diseases:
            diseases = [d.get('disease', 'unknown') for d in s.detected_diseases]
            report += f'Detected diseases: {", ".join(diseases)}. '
        else:
            report += 'No plant diseases detected. '

        self._speak(report)

    def _do_navigate(self, target: str,
                     then_inspect: bool = False,
                     then_water: bool = False,
                     then_sow: bool = False):
        """Navigate to a named location, optionally followed by an action."""
        if target not in self.farm_locations:
            self._speak(f'Unknown location: {target}. '
                        f'Known locations are: {", ".join(self.farm_locations.keys())}')
            return

        loc = self.farm_locations[target]
        with self.state_lock:
            self.state = BrainState.NAVIGATING

        self._speak(f'Navigating to {target.replace("_", " ")}.')

        # Build Nav2 goal
        if self.nav2_client:
            self._send_nav2_goal(
                loc['x'], loc['y'], loc.get('yaw', 0.0),
                then_inspect=then_inspect,
                then_water=then_water,
                then_sow=then_sow,
            )
        else:
            # Fallback: simple proportional drive (no Nav2)
            self.get_logger().warn(
                'Nav2 not available, using simple drive fallback')
            threading.Thread(
                target=self._simple_drive_to,
                args=(loc['x'], loc['y'], then_inspect, then_water, then_sow),
                daemon=True,
            ).start()

    def _send_nav2_goal(self, x: float, y: float, yaw: float,
                        then_inspect=False, then_water=False, then_sow=False):
        """Send a goal to Nav2 navigate_to_pose action server."""
        if not _NAV2_AVAILABLE or not self.nav2_client:
            self.get_logger().error('Nav2 not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending Nav2 goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server not available')
            self._speak('Navigation system is not available.')
            with self.state_lock:
                self.state = BrainState.IDLE
            return

        future = self.nav2_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback)
        future.add_done_callback(
            lambda f: self._nav2_goal_response(
                f, then_inspect, then_water, then_sow))

    def _nav2_feedback(self, feedback_msg):
        """Nav2 progress feedback."""
        feedback = feedback_msg.feedback
        # Can extract distance_remaining, estimated_time, etc.
        self.get_logger().debug(
            f'Nav2 feedback: distance_remaining='
            f'{feedback.distance_remaining:.2f}m', throttle_duration_sec=5.0)

    def _nav2_goal_response(self, future, then_inspect, then_water, then_sow):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self._speak('Navigation goal was rejected.')
            with self.state_lock:
                self.state = BrainState.IDLE
            return

        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav2_result(f, then_inspect, then_water, then_sow))

    def _nav2_result(self, future, then_inspect, then_water, then_sow):
        """Handle Nav2 goal completion."""
        result = future.result()
        # result.status: SUCCEEDED=4, ABORTED=6, CANCELED=5
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation complete')
            self._speak('Arrived at destination.')
            self._on_arrival(then_inspect, then_water, then_sow)
        else:
            self.get_logger().warn(f'Navigation ended with status {status}')
            self._speak('Navigation was not successful.')
            with self.state_lock:
                self.state = BrainState.IDLE

    def _on_arrival(self, inspect: bool, water: bool, sow: bool):
        """Execute post-navigation action."""
        if inspect:
            self._do_inspect()
        elif water:
            self._do_water()
        elif sow:
            self._do_sow()
        else:
            with self.state_lock:
                self.state = BrainState.IDLE

    # ── Simple fallback drive (no Nav2) ──────────────────────────────────
    def _simple_drive_to(self, target_x, target_y,
                         then_inspect, then_water, then_sow):
        """
        Very basic proportional controller fallback.
        In production, use Nav2 or the Isaac Sim policy.
        """
        rate = 10  # Hz
        kp_linear = 0.5
        kp_angular = 1.0

        for _ in range(rate * 60):  # max 60 seconds
            with self.state_lock:
                if self.state != BrainState.NAVIGATING:
                    return

            with self.sensor_lock:
                dx = target_x - self.sensors.odom_x
                dy = target_y - self.sensors.odom_y

            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.15:
                break

            target_yaw = math.atan2(dy, dx)
            with self.sensor_lock:
                yaw_err = target_yaw - self.sensors.odom_yaw
            # Normalize to [-pi, pi]
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            twist = Twist()
            twist.linear.x = min(kp_linear * dist, 0.3)
            twist.angular.z = kp_angular * yaw_err
            self.cmd_vel_pub.publish(twist)
            time.sleep(1.0 / rate)

        # Stop
        self.cmd_vel_pub.publish(Twist())
        self._on_arrival(then_inspect, then_water, then_sow)

    # ======================================================================
    # Farm actions (templates - connect to real actuators)
    # ======================================================================
    def _do_inspect(self):
        """Inspect current location using vision."""
        with self.state_lock:
            self.state = BrainState.INSPECTING
        self._speak('Inspecting area. Scanning for plant diseases and conditions.')

        # Wait a few seconds for vision to update
        time.sleep(3.0)

        with self.sensor_lock:
            diseases = list(self.sensors.detected_diseases)
            objects = list(self.sensors.detected_objects)
            soil = self.sensors.soil_moisture_pct
            dry = self.sensors.soil_is_dry

        report = 'Inspection complete. '
        if diseases:
            for d in diseases:
                report += (f'Found {d.get("disease", "unknown")} '
                           f'on {d.get("class", "plant")} '
                           f'with {d.get("conf", 0)*100:.0f}% confidence. ')
        else:
            report += 'No plant diseases detected. '

        if objects:
            report += f'I see {len(objects)} objects in the area. '

        report += f'Soil moisture is {soil:.0f} percent. '
        if dry:
            report += 'The soil is dry, watering is recommended. '

        self._speak(report)
        with self.state_lock:
            self.state = BrainState.IDLE

    def _do_water(self):
        """Activate watering mechanism."""
        with self.state_lock:
            self.state = BrainState.WATERING
        self._speak('Starting watering sequence.')

        # TODO: Replace with actual watering hardware control
        # Publish command to water pump controller
        water_msg = Bool()
        water_msg.data = True
        self.water_cmd_pub.publish(water_msg)

        # Wait for watering duration (template)
        time.sleep(5.0)

        water_msg.data = False
        self.water_cmd_pub.publish(water_msg)

        self._speak('Watering complete.')
        with self.state_lock:
            self.state = BrainState.IDLE

    def _do_sow(self):
        """Activate seed sowing mechanism."""
        with self.state_lock:
            self.state = BrainState.SOWING
        self._speak('Starting seed sowing sequence.')

        # TODO: Replace with actual sowing hardware control
        seed_msg = Bool()
        seed_msg.data = True
        self.seed_cmd_pub.publish(seed_msg)

        time.sleep(3.0)

        seed_msg.data = False
        self.seed_cmd_pub.publish(seed_msg)

        self._speak('Sowing complete.')
        with self.state_lock:
            self.state = BrainState.IDLE

    # ======================================================================
    # Patrol (autonomous monitoring)
    # ======================================================================
    def _do_start_patrol(self):
        with self.state_lock:
            self.state = BrainState.PATROLLING
        self.patrol_index = 0
        self._speak('Starting autonomous patrol of all farm sections.')
        self._patrol_next()

    def _patrol_next(self):
        with self.state_lock:
            if self.state != BrainState.PATROLLING:
                return

        if self.patrol_index >= len(DEFAULT_PATROL_WAYPOINTS):
            self._speak('Patrol complete. Returning home.')
            with self.state_lock:
                self.state = BrainState.RETURNING_HOME
            self._do_navigate('home')
            return

        waypoint = DEFAULT_PATROL_WAYPOINTS[self.patrol_index]
        self.patrol_index += 1
        self.get_logger().info(
            f'Patrol waypoint {self.patrol_index}/{len(DEFAULT_PATROL_WAYPOINTS)}: {waypoint}')

        # Navigate then inspect
        self._do_navigate(waypoint, then_inspect=True)

    # ======================================================================
    # Main tick (1 Hz)
    # ======================================================================
    def _tick(self):
        """Main brain loop - runs at 1 Hz."""
        with self.sensor_lock:
            self.sensors.last_update = time.time()

        with self.state_lock:
            state = self.state

        # Auto-water check (when idle)
        if state == BrainState.IDLE and self.auto_water:
            with self.sensor_lock:
                if (self.sensors.soil_is_dry
                        and self.sensors.soil_moisture_pct < self.soil_dry_threshold
                        and not self.sensors.is_raining):
                    self.get_logger().info(
                        'Auto-water triggered: soil is dry')
                    self._speak('Soil is dry. Starting automatic watering.')
                    self._do_water()

        # Auto-patrol check (when idle)
        if state == BrainState.IDLE and self.auto_patrol:
            now = time.time()
            if now - self.last_patrol_time > self.patrol_interval:
                self.last_patrol_time = now
                self.get_logger().info('Auto-patrol triggered')
                self._do_start_patrol()

        # After inspection during patrol, move to next waypoint
        if state == BrainState.IDLE and self.current_intent:
            if self.current_intent.action == 'patrol':
                self._patrol_next()

    # ======================================================================
    # Publishing helpers
    # ======================================================================
    def _speak(self, text: str):
        """Publish text to TTS."""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f'TTS: "{text}"')

    def _publish_status(self):
        """Publish brain status."""
        with self.state_lock:
            state_name = self.state.name
        msg = String()
        msg.data = json.dumps({
            'state': state_name,
            'target': self.current_intent.target if self.current_intent else '',
            'action': self.current_intent.action if self.current_intent else '',
            'timestamp': time.time(),
        })
        self.status_pub.publish(msg)

    def _publish_sensor_summary(self):
        """Publish aggregated sensor data as JSON."""
        with self.sensor_lock:
            s = self.sensors
            summary = {
                'soil_moisture_pct': round(s.soil_moisture_pct, 1),
                'soil_dry': s.soil_is_dry,
                'rain_intensity_pct': round(s.rain_intensity_pct, 1),
                'is_raining': s.is_raining,
                'temperature_c': round(s.temperature_c, 1),
                'pressure_hpa': round(s.pressure_pa / 100.0, 1),
                'gps': {
                    'lat': round(s.gps_lat, 7),
                    'lon': round(s.gps_lon, 7),
                    'fix': s.gps_fix,
                },
                'odom': {
                    'x': round(s.odom_x, 2),
                    'y': round(s.odom_y, 2),
                    'yaw_deg': round(math.degrees(s.odom_yaw), 1),
                },
                'detections': len(s.detected_objects),
                'diseases': len(s.detected_diseases),
                'timestamp': time.time(),
            }
        msg = String()
        msg.data = json.dumps(summary)
        self.sensor_summary_pub.publish(msg)

    # ======================================================================
    # Isaac Sim integration placeholder
    # ======================================================================
    def _query_isaac_policy(self, observation: dict) -> Tuple[float, float]:
        """
        TODO: Query the Isaac Sim trained RL policy for velocity commands.
        The model takes sensor observations and outputs (linear_vel, angular_vel).

        Args:
            observation: dict with lidar, imu, goal_relative, etc.

        Returns:
            (linear_x, angular_z) velocity commands
        """
        if self.isaac_model is None:
            return (0.0, 0.0)

        # Placeholder for TensorRT / ONNX inference
        # input_tensor = self._build_isaac_observation(observation)
        # output = self.isaac_model.infer(input_tensor)
        # return (output[0], output[1])
        return (0.0, 0.0)

    # ======================================================================
    # Cleanup
    # ======================================================================
    def destroy_node(self):
        self.cmd_vel_pub.publish(Twist())
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────
def main(args=None):
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID', '0')

    rclpy.init(args=args)
    try:
        node = FarmBrain()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
