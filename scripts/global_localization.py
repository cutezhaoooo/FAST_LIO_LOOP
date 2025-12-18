#!/usr/bin/env python3
"""
Global Localization Node for FAST-LIO ROS2
Performs ICP-based global localization in a pre-built map
"""

import numpy as np
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# Global variables
global_map = None
global_map_o3d = None
cur_scan = None
cur_odom = None
initialized = False
T_map_to_odom = np.eye(4)

# Parameters (can be configured via ROS parameters)
MAP_VOXEL_SIZE = 0.4
SCAN_VOXEL_SIZE = 0.15
FREQ_LOCALIZATION = 0.5  # Hz
LOCALIZATION_TH = 0.93   # Fitness threshold for successful localization
FOV = np.deg2rad(360)    # Field of view (360 for MID360)
FOV_FAR = 150.0          # Maximum range in meters
USE_INITIAL_GUESS = True


def voxel_down_sample(pcd, voxel_size):
    """Downsample point cloud using voxel grid filter"""
    if len(pcd.points) == 0:
        return pcd
    return pcd.voxel_down_sample(voxel_size=voxel_size)


def pointcloud2_to_xyz_array(cloud_msg):
    """Convert ROS2 PointCloud2 to numpy array"""
    # Get cloud data
    points = []
    point_step = cloud_msg.point_step

    for i in range(0, len(cloud_msg.data), point_step):
        x = np.frombuffer(cloud_msg.data[i:i+4], dtype=np.float32)[0]
        y = np.frombuffer(cloud_msg.data[i+4:i+8], dtype=np.float32)[0]
        z = np.frombuffer(cloud_msg.data[i+8:i+12], dtype=np.float32)[0]
        points.append([x, y, z])

    return np.array(points, dtype=np.float32)


def pose_to_mat(pose_msg):
    """Convert ROS Pose to 4x4 transformation matrix"""
    if isinstance(pose_msg, PoseWithCovarianceStamped):
        pose = pose_msg.pose.pose
    elif isinstance(pose_msg, Odometry):
        pose = pose_msg.pose.pose
    elif isinstance(pose_msg, PoseStamped):
        pose = pose_msg.pose
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


def mat_to_pose_msg(T, frame_id, stamp):
    """Convert 4x4 transformation matrix to ROS Odometry message"""
    odom_msg = Odometry()
    odom_msg.header.frame_id = frame_id
    odom_msg.header.stamp = stamp
    odom_msg.child_frame_id = 'camera_init'

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


def inverse_se3(T):
    """Compute inverse of SE(3) transformation matrix"""
    T_inv = np.eye(4)
    T_inv[:3, :3] = T[:3, :3].T
    T_inv[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return T_inv


def crop_global_map_in_FOV(global_map_pcd, pose_estimation, cur_odom_pose):
    """
    Crop global map based on current FOV
    Args:
        global_map_pcd: Open3D point cloud of global map
        pose_estimation: Current estimated pose in map frame (4x4 matrix)
        cur_odom_pose: Current odometry pose
    Returns:
        Cropped point cloud in FOV
    """
    if global_map_pcd is None or len(global_map_pcd.points) == 0:
        return o3d.geometry.PointCloud()

    # Transform: map -> base_link
    T_odom_to_base_link = pose_to_mat(cur_odom_pose)
    T_map_to_base_link = pose_estimation @ T_odom_to_base_link
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # Transform global map to base_link frame
    global_map_points = np.asarray(global_map_pcd.points)
    global_map_homo = np.column_stack([global_map_points, np.ones(len(global_map_points))])
    global_map_in_base_link = (T_base_link_to_map @ global_map_homo.T).T

    # Filter based on FOV
    x = global_map_in_base_link[:, 0]
    y = global_map_in_base_link[:, 1]

    if FOV > np.pi:  # 360-degree LiDAR
        valid_indices = np.where(
            (x < FOV_FAR) &
            (np.abs(np.arctan2(y, x)) < FOV / 2.0)
        )[0]
    else:  # Forward-facing LiDAR
        valid_indices = np.where(
            (x > 0) &
            (x < FOV_FAR) &
            (np.abs(np.arctan2(y, x)) < FOV / 2.0)
        )[0]

    # Create cropped point cloud
    cropped_pcd = o3d.geometry.PointCloud()
    if len(valid_indices) > 0:
        cropped_pcd.points = o3d.utility.Vector3dVector(global_map_points[valid_indices])

    return cropped_pcd


def registration_at_scale(pc_scan, pc_map, initial, scale):
    """
    Perform ICP registration at specific scale
    Args:
        pc_scan: Source point cloud
        pc_map: Target point cloud
        initial: Initial transformation guess
        scale: Downsampling scale factor
    Returns:
        transformation: Resulting transformation matrix
        fitness: Registration fitness score
    """
    if len(pc_scan.points) == 0 or len(pc_map.points) == 0:
        return initial, 0.0

    # Downsample
    scan_down = voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale)
    map_down = voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale)

    if len(scan_down.points) < 10 or len(map_down.points) < 10:
        return initial, 0.0

    # ICP registration
    reg_result = o3d.pipelines.registration.registration_icp(
        scan_down, map_down,
        max_correspondence_distance=1.0 * scale,
        init=initial,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=30
        )
    )

    return reg_result.transformation, reg_result.fitness


def global_localization(pose_estimation):
    """
    Perform global localization using ICP
    Args:
        pose_estimation: Initial pose estimate in map frame
    Returns:
        success: True if localization succeeded
    """
    global T_map_to_odom, cur_scan, cur_odom, global_map_o3d

    if cur_scan is None or cur_odom is None or global_map_o3d is None:
        return False

    # Crop global map based on FOV
    global_map_in_FOV = crop_global_map_in_FOV(global_map_o3d, pose_estimation, cur_odom)

    if len(global_map_in_FOV.points) < 100:
        return False

    # Multi-scale ICP
    # Coarse registration (scale=5)
    transformation, _ = registration_at_scale(
        cur_scan, global_map_in_FOV,
        pose_estimation, scale=5
    )

    # Fine registration (scale=1)
    transformation, fitness = registration_at_scale(
        cur_scan, global_map_in_FOV,
        transformation, scale=1
    )

    # Check if localization succeeded
    if fitness > LOCALIZATION_TH:
        # Update map to odom transform
        T_odom_to_base = pose_to_mat(cur_odom)
        T_map_to_odom = transformation @ inverse_se3(T_odom_to_base)
        return True

    return False


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')

        # Declare global variables first
        global MAP_VOXEL_SIZE, SCAN_VOXEL_SIZE, FREQ_LOCALIZATION
        global LOCALIZATION_TH, FOV, FOV_FAR

        # Declare parameters
        self.declare_parameter('map_path', '')
        self.declare_parameter('map_voxel_size', MAP_VOXEL_SIZE)
        self.declare_parameter('scan_voxel_size', SCAN_VOXEL_SIZE)
        self.declare_parameter('localization_freq', FREQ_LOCALIZATION)
        self.declare_parameter('localization_threshold', LOCALIZATION_TH)
        self.declare_parameter('fov_degree', np.rad2deg(FOV))
        self.declare_parameter('fov_far', FOV_FAR)

        # Get parameters
        map_path = self.get_parameter('map_path').value
        MAP_VOXEL_SIZE = self.get_parameter('map_voxel_size').value
        SCAN_VOXEL_SIZE = self.get_parameter('scan_voxel_size').value
        FREQ_LOCALIZATION = self.get_parameter('localization_freq').value
        LOCALIZATION_TH = self.get_parameter('localization_threshold').value
        FOV = np.deg2rad(self.get_parameter('fov_degree').value)
        FOV_FAR = self.get_parameter('fov_far').value

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_scan = self.create_subscription(
            PointCloud2, '/cloud_registered',
            self.scan_callback, sensor_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry',
            self.odom_callback, 10
        )
        self.sub_initial_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.initial_pose_callback, 10
        )

        # Publishers
        self.pub_map_to_odom = self.create_publisher(
            Odometry, '/map_to_odom', 10
        )
        self.pub_cur_scan_in_map = self.create_publisher(
            PointCloud2, '/cur_scan_in_map', 10
        )
        self.pub_submap = self.create_publisher(
            PointCloud2, '/submap', 10
        )

        # Timer for localization
        self.localization_timer = self.create_timer(
            1.0 / FREQ_LOCALIZATION,
            self.localization_callback
        )

        # Load global map
        if map_path and map_path != '':
            self.load_global_map(map_path)
        else:
            self.get_logger().warn('No map path provided. Waiting for map...')

        self.get_logger().info('Global Localization Node initialized')

    def load_global_map(self, map_path):
        """Load global map from PCD file"""
        global global_map_o3d, global_map

        try:
            self.get_logger().info(f'Loading global map from: {map_path}')
            global_map_o3d = o3d.io.read_point_cloud(map_path)

            if len(global_map_o3d.points) == 0:
                self.get_logger().error('Loaded map is empty!')
                return

            # Downsample map
            self.get_logger().info(f'Original map points: {len(global_map_o3d.points)}')
            global_map_o3d = voxel_down_sample(global_map_o3d, MAP_VOXEL_SIZE)
            self.get_logger().info(f'Downsampled map points: {len(global_map_o3d.points)}')

            global_map = global_map_o3d
            self.get_logger().info('Global map loaded successfully!')

        except Exception as e:
            self.get_logger().error(f'Failed to load map: {str(e)}')

    def scan_callback(self, msg):
        """Callback for received scan"""
        global cur_scan

        # Convert to Open3D point cloud
        points = pointcloud2_to_xyz_array(msg)
        if len(points) > 0:
            cur_scan = o3d.geometry.PointCloud()
            cur_scan.points = o3d.utility.Vector3dVector(points)

    def odom_callback(self, msg):
        """Callback for received odometry"""
        global cur_odom
        cur_odom = msg

    def initial_pose_callback(self, msg):
        """Callback for initial pose estimate"""
        global initialized, T_map_to_odom

        if global_map_o3d is None:
            self.get_logger().warn('Global map not loaded yet!')
            return

        self.get_logger().info('Received initial pose. Starting localization...')

        # Get initial pose
        initial_pose = pose_to_mat(msg)

        # Try to localize
        if cur_scan is not None and cur_odom is not None:
            success = global_localization(initial_pose)
            if success:
                initialized = True
                self.get_logger().info('Initial localization succeeded!')

                # Publish result
                map_to_odom_msg = mat_to_pose_msg(
                    T_map_to_odom, 'map', self.get_clock().now().to_msg()
                )
                self.pub_map_to_odom.publish(map_to_odom_msg)
            else:
                self.get_logger().warn('Initial localization failed. Please try again.')
        else:
            self.get_logger().warn('Waiting for scan and odom data...')

    def localization_callback(self):
        """Periodic localization callback"""
        global initialized, T_map_to_odom, cur_scan, cur_odom

        if not initialized:
            self.get_logger().warn('Waiting for initial pose...', throttle_duration_sec=5.0)
            return

        if cur_scan is None or cur_odom is None or global_map_o3d is None:
            return

        # Use current map_to_odom estimate
        T_odom_to_base = pose_to_mat(cur_odom)
        pose_estimation = T_map_to_odom @ T_odom_to_base

        # Perform localization
        success = global_localization(pose_estimation)

        if success:
            # Publish map to odom transform
            map_to_odom_msg = mat_to_pose_msg(
                T_map_to_odom, 'map', self.get_clock().now().to_msg()
            )
            self.pub_map_to_odom.publish(map_to_odom_msg)
        else:
            self.get_logger().warn('Localization failed in this iteration',
                                  throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
