#!/usr/bin/env python3
"""
Transform Fusion Node for FAST-LIO ROS2
Fuses high-frequency odometry with low-frequency global localization
Publishes fused pose and TF transforms
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def pose_to_mat(pose_msg):
    """Convert ROS Pose/Odometry to 4x4 transformation matrix"""
    if isinstance(pose_msg, Odometry):
        pose = pose_msg.pose.pose
    else:
        pose = pose_msg

    # Position
    t = np.array([pose.position.x, pose.position.y, pose.position.z])

    # Rotation
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    R = Rotation.from_quat(q).as_matrix()

    # Transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T


def mat_to_odom_msg(T, frame_id, child_frame_id, stamp):
    """Convert 4x4 transformation matrix to ROS Odometry message"""
    odom_msg = Odometry()
    odom_msg.header.frame_id = frame_id
    odom_msg.header.stamp = stamp
    odom_msg.child_frame_id = child_frame_id

    # Position
    odom_msg.pose.pose.position.x = T[0, 3]
    odom_msg.pose.pose.position.y = T[1, 3]
    odom_msg.pose.pose.position.z = T[2, 3]

    # Rotation
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    return odom_msg


def mat_to_transform_msg(T, frame_id, child_frame_id, stamp):
    """Convert 4x4 transformation matrix to TransformStamped"""
    transform_msg = TransformStamped()
    transform_msg.header.frame_id = frame_id
    transform_msg.header.stamp = stamp
    transform_msg.child_frame_id = child_frame_id

    # Translation
    transform_msg.transform.translation.x = T[0, 3]
    transform_msg.transform.translation.y = T[1, 3]
    transform_msg.transform.translation.z = T[2, 3]

    # Rotation
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    transform_msg.transform.rotation.x = q[0]
    transform_msg.transform.rotation.y = q[1]
    transform_msg.transform.rotation.z = q[2]
    transform_msg.transform.rotation.w = q[3]

    return transform_msg


class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion_node')

        # State variables
        self.cur_odom = None
        self.cur_map_to_odom = None
        self.last_map_to_odom_time = self.get_clock().now()

        # Declare parameters
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('fusion_rate', 50.0)  # Hz

        # Get parameters
        self.publish_tf = self.get_parameter('publish_tf').value
        fusion_rate = self.get_parameter('fusion_rate').value

        # QoS profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry',
            self.odom_callback, sensor_qos
        )
        self.sub_map_to_odom = self.create_subscription(
            Odometry, '/map_to_odom',
            self.map_to_odom_callback, 10
        )

        # Publishers
        self.pub_localization = self.create_publisher(
            Odometry, '/localization', 10
        )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for fusion
        self.fusion_timer = self.create_timer(
            1.0 / fusion_rate,
            self.fusion_callback
        )

        self.get_logger().info('Transform Fusion Node initialized')

    def odom_callback(self, msg):
        """Callback for high-frequency odometry"""
        self.cur_odom = msg

    def map_to_odom_callback(self, msg):
        """Callback for low-frequency global localization result"""
        self.cur_map_to_odom = msg
        self.last_map_to_odom_time = self.get_clock().now()
        self.get_logger().info('Received updated map_to_odom transform',
                              throttle_duration_sec=2.0)

    def fusion_callback(self):
        """Fuse odometry with global localization"""
        if self.cur_odom is None:
            self.get_logger().warn('Waiting for odometry...',
                                  throttle_duration_sec=5.0)
            return

        # Get current timestamp
        current_time = self.get_clock().now()

        # If we have global localization, fuse it
        if self.cur_map_to_odom is not None:
            # T_map_to_base = T_map_to_odom * T_odom_to_base
            T_map_to_odom = pose_to_mat(self.cur_map_to_odom)
            T_odom_to_base = pose_to_mat(self.cur_odom)
            T_map_to_base = T_map_to_odom @ T_odom_to_base

            # Publish fused localization
            localization_msg = mat_to_odom_msg(
                T_map_to_base, 'map', 'body',
                current_time.to_msg()
            )
            self.pub_localization.publish(localization_msg)

            # Publish TF: map -> camera_init
            if self.publish_tf:
                tf_msg = mat_to_transform_msg(
                    T_map_to_odom, 'map', 'camera_init',
                    current_time.to_msg()
                )
                self.tf_broadcaster.sendTransform(tf_msg)

        else:
            # No global localization yet, just republish odometry
            # This assumes camera_init is the odom frame
            self.get_logger().warn('No global localization available yet',
                                  throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
