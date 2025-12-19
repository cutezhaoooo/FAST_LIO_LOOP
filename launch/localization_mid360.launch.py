import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    map_path = LaunchConfiguration('map')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # Localization parameters
    map_voxel_size = LaunchConfiguration('map_voxel_size')
    scan_voxel_size = LaunchConfiguration('scan_voxel_size')
    localization_freq = LaunchConfiguration('localization_freq')
    localization_threshold = LaunchConfiguration('localization_threshold')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file for FAST-LIO'
    )
    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to the global map PCD file (required)'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_map_voxel_size_cmd = DeclareLaunchArgument(
        'map_voxel_size', default_value='0.4',
        description='Voxel size for downsampling global map'
    )
    declare_scan_voxel_size_cmd = DeclareLaunchArgument(
        'scan_voxel_size', default_value='0.15',
        description='Voxel size for downsampling scan'
    )
    declare_localization_freq_cmd = DeclareLaunchArgument(
        'localization_freq', default_value='0.5',
        description='Frequency of global localization (Hz)'
    )
    declare_localization_threshold_cmd = DeclareLaunchArgument(
        'localization_threshold', default_value='0.93',
        description='Fitness threshold for successful localization'
    )

    # FAST-LIO mapping node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Global localization node
    global_localization_node = Node(
        package='fast_lio',
        executable='global_localization.py',
        name='global_localization_node',
        parameters=[
            {'map_path': map_path},
            {'map_voxel_size': map_voxel_size},
            {'scan_voxel_size': scan_voxel_size},
            {'localization_freq': localization_freq},
            {'localization_threshold': localization_threshold},
            {'fov_degree': 360.0},
            {'fov_far': 150.0},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Transform fusion node
    transform_fusion_node = Node(
        package='fast_lio',
        executable='transform_fusion.py',
        name='transform_fusion_node',
        parameters=[
            {'publish_tf': True},
            {'fusion_rate': 50.0},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # Static TF: base_link -> livox_frame
    # 如果你的雷达相对于base_link有偏移，修改这里的参数
    static_tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
        # 格式: x y z yaw pitch roll parent_frame child_frame
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_voxel_size_cmd)
    ld.add_action(declare_scan_voxel_size_cmd)
    ld.add_action(declare_localization_freq_cmd)
    ld.add_action(declare_localization_threshold_cmd)

    # Add nodes
    ld.add_action(fast_lio_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(static_tf_base_to_livox)
    ld.add_action(rviz_node)

    return ld
