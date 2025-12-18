#!/usr/bin/env python3
"""
Publish Initial Pose for FAST-LIO Localization
Usage: python3 publish_initial_pose.py x y z yaw pitch roll
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self, x, y, z, yaw, pitch, roll):
        super().__init__('initial_pose_publisher')

        # Create publisher
        self.pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Set position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = z

        # Set orientation from Euler angles (in radians)
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        q = r.as_quat()  # Returns [x, y, z, w]
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        # Set covariance (diagonal matrix with small values)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.25   # x variance
        pose_msg.pose.covariance[7] = 0.25   # y variance
        pose_msg.pose.covariance[14] = 0.25  # z variance
        pose_msg.pose.covariance[21] = 0.068 # roll variance
        pose_msg.pose.covariance[28] = 0.068 # pitch variance
        pose_msg.pose.covariance[35] = 0.068 # yaw variance

        # Wait for subscribers
        self.get_logger().info('Waiting for subscribers...')
        while self.pub_initial_pose.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Publish initial pose
        self.get_logger().info(f'Publishing initial pose: x={x:.2f}, y={y:.2f}, z={z:.2f}, '
                              f'yaw={yaw:.2f}, pitch={pitch:.2f}, roll={roll:.2f}')
        self.pub_initial_pose.publish(pose_msg)

        # Publish a few times to ensure it's received
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
            self.pub_initial_pose.publish(pose_msg)

        self.get_logger().info('Initial pose published successfully!')


def main(args=None):
    # Parse command line arguments
    if len(sys.argv) < 7:
        print('Usage: python3 publish_initial_pose.py x y z yaw pitch roll')
        print('  x, y, z: Position in meters')
        print('  yaw, pitch, roll: Orientation in radians')
        print('\nExample:')
        print('  python3 publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0')
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        yaw = float(sys.argv[4])
        pitch = float(sys.argv[5])
        roll = float(sys.argv[6])
    except ValueError:
        print('Error: All arguments must be numbers')
        sys.exit(1)

    # Initialize ROS2
    rclpy.init(args=args)

    # Create and run node
    node = InitialPosePublisher(x, y, z, yaw, pitch, roll)

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
