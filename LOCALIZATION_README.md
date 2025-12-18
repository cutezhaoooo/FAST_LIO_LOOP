# FAST-LIO ROS2 重定位功能

这是FAST-LIO ROS2版本的扩展，添加了基于预建地图的全局定位（重定位）功能。

## 功能特点

- ✅ **实时全局定位**：在预建的点云地图中进行实时定位
- ✅ **高频里程计 + 低频全局定位融合**：结合FAST-LIO高频里程计（~100Hz）和全局ICP定位（~0.5Hz）
- ✅ **消除累积误差**：通过定期全局定位校正，消除里程计的累积漂移
- ✅ **灵活的初始化**：支持通过命令行或RVIZ手动设置初始位姿
- ✅ **支持多种雷达**：Livox MID360, Avia, Velodyne, Ouster等

## 系统架构

系统由三个节点组成：

1. **fastlio_mapping** (C++)：高频里程计，提供实时位姿估计
2. **global_localization_node** (Python)：低频全局定位，使用ICP在全局地图中配准
3. **transform_fusion_node** (Python)：位姿融合，将里程计和全局定位结果融合

## 依赖项

### ROS2依赖
```bash
sudo apt install ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-geometry-msgs
```

### Python依赖
```bash
pip3 install numpy scipy open3d==0.18.0
```

**注意**：
- Open3D版本建议使用0.18.0，与ROS2兼容性较好
- 如果遇到安装问题，可以尝试：`pip3 install open3d --upgrade`

## 编译

```bash
cd ~/fastlio_location
colcon build --packages-select fast_lio
source install/setup.bash
```

## 使用方法

### 1. 准备地图文件

首先需要使用FAST-LIO建图模式生成地图：

```bash
# 启动建图
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 播放数据包或启动传感器
ros2 bag play your_mapping_data.bag

# 保存地图 (服务调用)
ros2 service call /map_save std_srvs/srv/Trigger
```

地图会保存在配置文件指定的路径（默认：`./test.pcd`）。

### 2. 启动定位

```bash
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/path/to/your/map.pcd \
    config_file:=mid360_localization.yaml
```

**可选参数**：
- `map`: 全局地图文件路径（必需）
- `config_file`: 配置文件名称（默认：mid360.yaml）
- `map_voxel_size`: 地图降采样尺寸（默认：0.4m）
- `scan_voxel_size`: 扫描降采样尺寸（默认：0.15m）
- `localization_freq`: 全局定位频率（默认：0.5Hz）
- `localization_threshold`: 定位成功阈值（默认：0.93）
- `rviz`: 是否启动RVIZ（默认：true）

**示例**：
```bash
# 基本用法
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/home/user/maps/warehouse.pcd

# 自定义参数
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/home/user/maps/warehouse.pcd \
    map_voxel_size:=0.3 \
    localization_freq:=1.0 \
    localization_threshold:=0.95
```

### 3. 提供初始位姿

系统启动后，需要提供一个粗略的初始位姿。有两种方式：

#### 方法1：使用命令行工具（推荐）

```bash
ros2 run fast_lio publish_initial_pose.py <x> <y> <z> <yaw> <pitch> <roll>
```

- `x, y, z`: 位置（米）
- `yaw, pitch, roll`: 姿态角（弧度）

**示例**：
```bash
# 位置 (14.5, -7.5, 0)，偏航角 -0.25 弧度
ros2 run fast_lio publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0
```

#### 方法2：使用RVIZ

1. 在RVIZ中点击工具栏的 "2D Pose Estimate" 按钮
2. 在地图上点击并拖动鼠标设置初始位置和方向

### 4. 播放数据或启动传感器

```bash
# 播放rosbag
ros2 bag play your_localization_data.bag

# 或启动实时传感器
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**注意**：初始化阶段建议让机器人保持静止，或者先播放数据包约0.5秒后暂停，等待初始化成功后再继续。

## 话题说明

### 输入话题
- `/livox/lidar` (sensor_msgs/PointCloud2 或 livox_ros_driver2/CustomMsg)：激光点云数据
- `/livox/imu` (sensor_msgs/Imu)：IMU数据
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped)：初始位姿估计

### 输出话题
- `/Odometry` (nav_msgs/Odometry)：FAST-LIO里程计（在camera_init坐标系下）
- `/localization` (nav_msgs/Odometry)：融合后的全局位姿（在map坐标系下）
- `/map_to_odom` (nav_msgs/Odometry)：地图到里程计的变换
- `/cloud_registered` (sensor_msgs/PointCloud2)：配准后的点云
- `/path` (nav_msgs/Path)：轨迹路径

### TF变换
- `map` → `camera_init`：地图到里程计坐标系的变换
- `camera_init` → `body`：里程计坐标系到机器人本体的变换
- `body` → `lidar`：机器人本体到激光雷达的变换

## 参数调优

### 全局定位参数

在 `localization_mid360.launch.py` 中可以调整以下参数：

- **map_voxel_size** (默认：0.4)
  - 全局地图的体素降采样大小
  - 更小的值提供更高精度但消耗更多内存和计算
  - 建议范围：0.2-0.5m

- **scan_voxel_size** (默认：0.15)
  - 当前扫描的体素降采样大小
  - 更小的值提供更多特征但增加计算负担
  - 建议范围：0.1-0.3m

- **localization_freq** (默认：0.5)
  - 全局定位的频率（Hz）
  - 更高频率提供更频繁的校正但增加CPU负载
  - 建议范围：0.2-1.0Hz

- **localization_threshold** (默认：0.93)
  - ICP配准的fitness阈值（0-1）
  - 只有超过此阈值的配准结果才会被接受
  - 更高的值更保守，降低错误匹配
  - 建议范围：0.85-0.98

### FAST-LIO参数

在 `config/mid360_localization.yaml` 中：

- **extrinsic_est_en**: 建议设置为 `false`（使用固定外参，提高稳定性）
- **pcd_save_en**: 建议设置为 `false`（定位模式不需要保存地图）
- **map_en**: 建议设置为 `false`（不发布局部地图以节省带宽）

## 常见问题

### 1. 定位失败或精度差

**可能原因**：
- 初始位姿偏差过大
- 地图质量不佳
- 环境变化（动态物体、几何变化）

**解决方案**：
- 提供更准确的初始位姿
- 调低 `localization_threshold` 至 0.85-0.90
- 增加 `localization_freq` 至 0.8-1.0Hz
- 重新建图，确保地图质量

### 2. "Waiting for initial pose" 一直等待

**原因**：未提供初始位姿

**解决方案**：
使用命令行工具或RVIZ提供初始位姿（参见上文"提供初始位姿"部分）

### 3. "Global map not loaded" 错误

**原因**：地图文件路径错误或文件损坏

**解决方案**：
- 检查地图文件路径是否正确
- 确认PCD文件存在且可读
- 尝试用Open3D或CloudCompare打开PCD文件验证

### 4. Open3D导入错误

**错误信息**：`ModuleNotFoundError: No module named 'open3d'`

**解决方案**：
```bash
pip3 install open3d==0.18.0
# 或者
pip3 install open3d --upgrade
```

### 5. 定位结果漂移

**可能原因**：
- 全局定位频率过低
- 里程计质量差
- 地图与当前环境不匹配

**解决方案**：
- 增加 `localization_freq` 至 0.8-1.0Hz
- 检查IMU数据质量
- 调整 `scan_voxel_size` 至 0.1-0.15m以获得更多特征

### 6. CPU占用率过高

**解决方案**：
- 增加 `map_voxel_size` 至 0.5-0.6m
- 降低 `localization_freq` 至 0.3-0.4Hz
- 增加 `scan_voxel_size` 至 0.2-0.3m

## 与ROS1版本的区别

| 特性 | ROS1版本 | ROS2版本 |
|------|---------|---------|
| **Python版本** | Python 2.7 | Python 3 |
| **Open3D版本** | 0.9 | 0.18.0 |
| **节点通信** | rospy | rclpy |
| **Launch文件** | XML | Python |
| **TF库** | tf | tf2_ros |
| **QoS配置** | 无 | 支持QoS配置 |

## 性能指标

在典型场景下的性能表现：

- **定位频率**：全局定位 ~0.5Hz + 里程计 ~100Hz
- **定位精度**：平移 <0.1m，旋转 <2°（良好条件下）
- **初始化时间**：1-3秒
- **CPU占用**：单核 30-50%（取决于参数设置）
- **内存占用**：500MB-2GB（取决于地图大小）

## 技术支持

如遇问题，请检查：

1. **终端输出**：查看错误信息和警告
2. **话题通信**：`ros2 topic list` 和 `ros2 topic echo`
3. **TF树**：`ros2 run tf2_tools view_frames`
4. **节点状态**：`ros2 node list` 和 `ros2 node info`

## 参考资料

- [FAST-LIO原版](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO-LOCALIZATION (ROS1参考实现)](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
- [Open3D文档](http://www.open3d.org/docs/release/)

## 致谢

本实现参考了以下项目：
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO) - 原始FAST-LIO算法
- [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) - ROS1重定位实现
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) - 高效的增量KD树

## 许可证

本项目遵循原FAST-LIO的BSD许可证。
