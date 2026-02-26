#!/usr/bin/env python3
"""
Mecanum Robot Driver Node (ROS2 Humble)

Subscribes to /cmd_vel and converts to 4 mecanum wheel speeds
using inverse kinematics, then sends to Arduino over serial.

Mecanum Inverse Kinematics:
  v_fl = vx - vy - (lx + ly) * wz
  v_fr = vx + vy + (lx + ly) * wz
  v_rl = vx + vy - (lx + ly) * wz
  v_rr = vx - vy + (lx + ly) * wz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import time
import threading


class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('mecanum_driver')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('robot_width', 0.20)
        self.declare_parameter('robot_length', 0.15)
        self.declare_parameter('max_motor_pwm', 255)
        self.declare_parameter('max_wheel_speed', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('serial_timeout', 0.1)

        # Read parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.robot_width = self.get_parameter('robot_width').value
        self.robot_length = self.get_parameter('robot_length').value
        self.max_pwm = self.get_parameter('max_motor_pwm').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.serial_timeout = self.get_parameter('serial_timeout').value

        # Half-widths for kinematics
        self.lx = self.robot_width / 2.0
        self.ly = self.robot_length / 2.0

        # Serial connection
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publisher (debug)
        self.wheel_pub = self.create_publisher(
            Float32MultiArray, 'wheel_speeds', 10)

        # Watchdog timer
        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        # Serial reader thread
        self.running = True
        self.reader_thread = threading.Thread(
            target=self.serial_reader, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(
            f'Mecanum driver started on {self.serial_port} @ {self.baud_rate}')
        self.get_logger().info(
            f'Robot dims: W={self.robot_width}m, L={self.robot_length}m, '
            f'wheel_r={self.wheel_radius}m')

    def connect_serial(self):
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.serial_timeout)
                time.sleep(2.0)
                self.ser.reset_input_buffer()
                self.get_logger().info('Serial connection established')
                return
            except serial.SerialException as e:
                self.get_logger().warn(
                    f'Serial attempt {attempt+1}/{max_retries} failed: {e}')
                time.sleep(1.0)
        self.get_logger().error(
            'Could not open serial port! Motors will not work.')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Mecanum Inverse Kinematics
        k = self.lx + self.ly
        v_fl = vx - vy - k * wz
        v_fr = vx + vy + k * wz
        v_rl = vx + vy - k * wz
        v_rr = vx - vy + k * wz

        # Convert to PWM
        pwm_fl = self.speed_to_pwm(v_fl)
        pwm_fr = self.speed_to_pwm(v_fr)
        pwm_rl = self.speed_to_pwm(v_rl)
        pwm_rr = self.speed_to_pwm(v_rr)

        self.send_motor_command(pwm_fl, pwm_fr, pwm_rl, pwm_rr)

        # Publish debug
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [float(v_fl), float(v_fr),
                          float(v_rl), float(v_rr)]
        self.wheel_pub.publish(wheel_msg)

    def speed_to_pwm(self, speed_ms: float) -> int:
        speed_ms = max(-self.max_wheel_speed,
                       min(self.max_wheel_speed, speed_ms))
        pwm = int((speed_ms / self.max_wheel_speed) * self.max_pwm)
        return max(-255, min(255, pwm))

    def send_motor_command(self, fl: int, fr: int, rl: int, rr: int):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(
                'Serial not connected, attempting reconnect...')
            self.connect_serial()
            return
        cmd = f"M {fl} {fr} {rl} {rr}\n"
        try:
            with self.serial_lock:
                self.ser.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.ser = None

    def send_stop(self):
        if self.ser and self.ser.is_open:
            try:
                with self.serial_lock:
                    self.ser.write(b"S\n")
            except serial.SerialException:
                pass

    def watchdog_callback(self):
        if time.time() - self.last_cmd_time > self.cmd_vel_timeout:
            self.send_motor_command(0, 0, 0, 0)

    def serial_reader(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    with self.serial_lock:
                        if self.ser.in_waiting:
                            line = self.ser.readline().decode(
                                'utf-8', errors='ignore').strip()
                            if line and not line.startswith('OK'):
                                self.get_logger().debug(
                                    f'Arduino: {line}')
                except serial.SerialException:
                    pass
            time.sleep(0.01)

    def destroy_node(self):
        self.running = False
        self.send_stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
