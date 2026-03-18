#!/usr/bin/env python3
"""
Dummy odometry publisher for SLAM without wheel encoders.
Publishes identity transform from odom to base_link at high frequency.
This populates the TF buffer with historical data for slam_toolbox.

KNOWN LIMITATION:
  This node claims the robot is always at the origin.  slam_toolbox relies
  entirely on scan-matching to estimate pose, which works at low speed but
  causes map shearing under fast Mecanum movement (strafing, spinning) —
  the scan matcher sees walls shifting while odom insists the robot hasn't
  moved, creating contradictory constraints.

MIGRATION PATH (in order of increasing accuracy):
  1. rf2o_laser_odometry (implemented in jetson_launch.py, odom_source='laser'):
     Estimates odom→base_link from consecutive /scan frame wall shifts.
     Works without encoders but drifts in featureless corridors.
  2. Wheel encoder odometry (requires Arduino firmware changes):
     Arduino reads encoder ticks → sends RPM/tick data over serial →
     mecanum_driver_node parses feedback, applies forward kinematics,
     publishes nav_msgs/Odometry + odom→base_link TF.  This is the
     correct long-term solution for closed-loop control.
  3. IMU-fused odometry (robot_localization EKF):
     Fuse wheel encoder odom + BNO055 IMU for drift correction.
     Requires step 2 as a prerequisite.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DummyOdomPublisher(Node):
    def __init__(self):
        super().__init__('dummy_odom_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish at 50 Hz to fill TF buffer
        self.timer = self.create_timer(0.02, self.publish_odom)

        self.get_logger().info('Dummy odometry publisher started (odom -> base_link)')

    def publish_odom(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Identity transform (no movement)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
