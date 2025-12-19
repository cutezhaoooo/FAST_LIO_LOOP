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

import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
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

def pointcloud2_to_o3d(cloud_msg):
    """Efficiently convert PointCloud2 to Open3D PointCloud"""
    # 使用 sensor_msgs_py 读取点云数据
    gen = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    points_list = list(gen)

    if not points_list:
        return o3d.geometry.PointCloud()

    # 提取xyz坐标 - 正确处理结构化数组
    points_array = np.array(points_list)
    if points_array.dtype.names:  # 结构化数组
        xyz = np.zeros((len(points_array), 3), dtype=np.float64)
        xyz[:, 0] = points_array['x']
        xyz[:, 1] = points_array['y']
        xyz[:, 2] = points_array['z']
    else:  # 普通数组
        xyz = points_array.astype(np.float64)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd

def o3d_to_pointcloud2(pcd, frame_id, stamp):
    """Convert Open3D PointCloud to ROS2 PointCloud2"""
    points = np.asarray(pcd.points)
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    return pc2.create_cloud_xyz32(header, points)

# 全局变量定义（与之前类似，略微调整）
global_map_o3d = None
cur_scan_o3d = None
cur_odom = None
initialized = False
# T_map_to_odom 存储 map 到 odom (camera_init) 的变换
T_map_to_odom = np.eye(4) 

# 参数默认值
MAP_VOXEL_SIZE = 0.4
SCAN_VOXEL_SIZE = 0.15
LOCALIZATION_TH = 0.90

class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')

        # 参数声明
        self.declare_parameter('map_path', '')
        self.declare_parameter('map_voxel_size', MAP_VOXEL_SIZE)
        self.declare_parameter('scan_voxel_size', SCAN_VOXEL_SIZE)
        self.declare_parameter('localization_threshold', LOCALIZATION_TH)

        map_path = self.get_parameter('map_path').value
        self.map_voxel_size = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size = self.get_parameter('scan_voxel_size').value
        self.loc_th = self.get_parameter('localization_threshold').value

        # QoS
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # 2. 其他高频话题可以用 Best Effort (如 scan)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )

        # Subscribers
        self.sub_scan = self.create_subscription(PointCloud2, '/cloud_registered', self.scan_callback, sensor_qos)
        self.sub_odom = self.create_subscription(Odometry, '/Odometry', self.odom_callback, sensor_qos)
        self.sub_initial = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        # Publishers
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 10) 
        self.pub_cur_scan_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 10) 
        

        self.pub_global_map = self.create_publisher(PointCloud2, '/global_map_viz', latching_qos)

        # Load Map
        if map_path:
            self.load_map(map_path)

        self.get_logger().info('Global Localization Node Ready.')

    def load_map(self, path):
        global global_map_o3d
        try:
            pcd = o3d.io.read_point_cloud(path)
            global_map_o3d = pcd.voxel_down_sample(self.map_voxel_size)
            # 发布一次用于 Rviz 显示
            msg = o3d_to_pointcloud2(global_map_o3d, 'map', self.get_clock().now().to_msg())
            self.pub_global_map.publish(msg)
            self.get_logger().info(f'Map loaded: {len(global_map_o3d.points)} points')
        except Exception as e:
            self.get_logger().error(f'Map load failed: {e}')

    def scan_callback(self, msg):
        global cur_scan_o3d
        # 将 ROS msg 转为 Open3D
        cur_scan_o3d = pointcloud2_to_o3d(msg)

    def odom_callback(self, msg):
        global cur_odom
        cur_odom = msg
        
        # 如果已经初始化成功，持续发布可视化点云
        if initialized and cur_scan_o3d is not None:
             self.publish_visuals()

    def initial_pose_callback(self, msg):
        global initialized, T_map_to_odom, cur_odom, cur_scan_o3d

        if global_map_o3d is None or cur_scan_o3d is None or cur_odom is None:
            self.get_logger().warn('Data not ready for localization')
            return

        self.get_logger().info('Executing Global Localization...')

        # 1. 获取初始猜测 (T_map_to_livox_guess)
        T_initial_guess = self.pose_to_mat_msg(msg) # 这是一个 map -> livox 的猜测

        # 2. 获取当前 Odom (T_odom_to_livox)
        T_odom_to_livox = self.pose_to_mat_msg(cur_odom)

        # 注意：FAST-LIO 的 cloud_registered 通常是在 odom (camera_init) 坐标系下的
        # 如果我们直接用 cloud_registered 做匹配，我们要找的是 T_map_to_odom
        # 初始猜测 T_map_to_odom = T_initial_guess * T_odom_to_livox^-1
        
        T_map_to_odom_guess = T_initial_guess @ np.linalg.inv(T_odom_to_livox)

        # 3. 准备 ICP 数据
        # source: 当前点云 (在 odom 坐标系下)
        source = cur_scan_o3d
        # target: 全局地图 (在 map 坐标系下)
        target = global_map_o3d

        # 4. 执行 ICP
        # 这里的 trans 是应用在 source 上的，即 T_map_to_odom
        reg = o3d.pipelines.registration.registration_icp(
            source, target, 2.0, T_map_to_odom_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        if reg.fitness > self.loc_th:
            T_map_to_odom = reg.transformation
            initialized = True
            self.get_logger().info(f'Localization Success! Fitness: {reg.fitness}')
            
            # 发布变换给 Fusion Node
            self.publish_map_to_odom()
        else:
            self.get_logger().warn(f'Localization Failed. Fitness: {reg.fitness}')

    def publish_map_to_odom(self):
        # 将矩阵转为 Odometry 消息发送给 TF 融合节点
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'odom' # 这里对应 camera_init
        
        t = T_map_to_odom[:3, 3]
        r = Rotation.from_matrix(T_map_to_odom[:3, :3])
        q = r.as_quat()

        msg.pose.pose.position.x = t[0]
        msg.pose.pose.position.y = t[1]
        msg.pose.pose.position.z = t[2]
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        self.pub_map_to_odom.publish(msg)

    def publish_visuals(self):
        """核心可视化功能：发布当前 scan 在 map 中的位置"""
        # cur_scan_o3d 目前是在 odom (camera_init) 坐标系下的 (假设来自 cloud_registered)
        # 我们需要把它变换到 map 坐标系
        
        # 1. 拷贝点云防止修改原数据
        pcd_viz = o3d.geometry.PointCloud(cur_scan_o3d)
        
        # 2. 变换: points_map = T_map_to_odom * points_odom
        pcd_viz.transform(T_map_to_odom)
        
        # 3. 转换回 ROS 消息并发布
        # 注意 frame_id 必须是 map
        msg = o3d_to_pointcloud2(pcd_viz, 'map', self.get_clock().now().to_msg())
        self.pub_cur_scan_in_map.publish(msg)

    def pose_to_mat_msg(self, pose_msg):
        # (复用之前的逻辑)
        if isinstance(pose_msg, Odometry):
            p = pose_msg.pose.pose
        elif isinstance(pose_msg, PoseWithCovarianceStamped):
            p = pose_msg.pose.pose
        else:
            p = pose_msg
        
        t = [p.position.x, p.position.y, p.position.z]
        r = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        T = np.eye(4)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = t
        return T

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
