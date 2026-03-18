# ─────────────────────────────────────────────────────────────────────────────
# Mecanum Driver Launch — runs on Raspberry Pi 4b
#
# ENCODER NOTE: This launch file does NOT start encoder_node.  The encoders
# are physically wired to the RPi 5 GPIO (not the Pi 4b), so encoder_node.py
# runs on the RPi 5 and publishes /encoders/position over FastDDS.
# mecanum_driver_node subscribes to that topic when publish_odom=True.
#
# If you migrate encoders to the Pi 4b GPIO or upgrade to Arduino Mega with
# encoder pins, add encoder_node to this launch file and set publish_odom: true
# in mecanum_params.yaml.
# ─────────────────────────────────────────────────────────────────────────────
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mecanum_driver'),
        'config',
        'mecanum_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mecanum_driver',
            executable='mecanum_driver',
            name='mecanum_driver',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
