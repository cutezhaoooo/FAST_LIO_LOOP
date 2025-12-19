#!/usr/bin/env python3
"""
Transform Fusion Node
TF Tree: map -> odom -> base_link
需要外部发布 base_link -> livox_frame 的静态变换
"""

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 状态量
        self.T_map_to_odom = np.eye(4)  # map到odom的变换（由重定位更新）
        self.received_loc = False

        # 订阅FAST-LIO里程计 (camera_init -> body)
        # body等同于base_link
        self.sub_odom = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        # 订阅重定位结果 (修正map->odom)
        self.sub_map_loc = self.create_subscription(
            Odometry,
            '/map_to_odom',
            self.map_loc_callback,
            10
        )

        self.get_logger().info('Transform Fusion Node Initialized')
        self.get_logger().info('TF Tree: map -> odom -> base_link')
        self.get_logger().info('Expecting external TF: base_link -> livox_frame')

    def map_loc_callback(self, msg):
        """接收Global Localization计算出的map->odom变换"""
        p = msg.pose.pose
        t = [p.position.x, p.position.y, p.position.z]
        r = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

        T = np.eye(4)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = t

        self.T_map_to_odom = T
        self.received_loc = True
        self.get_logger().info('Updated map->odom transform')

    def odom_callback(self, msg):
        """
        高频回调：
        1. 接收FAST-LIO odom (camera_init -> body)
        2. 发布map -> odom (固定/低频更新)
        3. 发布odom -> base_link (高频，来自FAST-LIO)
        """
        current_time = msg.header.stamp

        # msg是camera_init -> body
        # 我们把camera_init当作odom，body当作base_link
        p = msg.pose.pose

        # --- 发布 odom -> base_link ---
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = current_time
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'

        t_odom_base.transform.translation.x = p.position.x
        t_odom_base.transform.translation.y = p.position.y
        t_odom_base.transform.translation.z = p.position.z
        t_odom_base.transform.rotation = p.orientation

        self.tf_broadcaster.sendTransform(t_odom_base)

        # --- 发布 map -> odom ---
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = current_time
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'

        # 从T_map_to_odom提取平移和旋转
        trans = self.T_map_to_odom[:3, 3]
        rot_mat = self.T_map_to_odom[:3, :3]
        quat = Rotation.from_matrix(rot_mat).as_quat()

        t_map_odom.transform.translation.x = trans[0]
        t_map_odom.transform.translation.y = trans[1]
        t_map_odom.transform.translation.z = trans[2]
        t_map_odom.transform.rotation.x = quat[0]
        t_map_odom.transform.rotation.y = quat[1]
        t_map_odom.transform.rotation.z = quat[2]
        t_map_odom.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t_map_odom)

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