# FAST-LIO 重定位系统 - TF树结构说明

## 当前TF树结构

```
map (全局地图坐标系)
 └─> odom (里程计坐标系，第一次定位位置作为原点)
      └─> livox_frame (雷达实时位置)

camera_init (独立树：FAST-LIO上电位置)
 └─> body (FAST-LIO实时位置，与livox_frame位置相同)
```

## 节点功能

### 1. fastlio_mapping (C++ - FAST-LIO原生节点)
- **输入**:
  - `/livox/lidar/pointcloud` (sensor_msgs/PointCloud2)
  - `/livox/imu` (sensor_msgs/Imu)
- **输出**:
  - `/Odometry` (nav_msgs/Odometry) - frame_id: camera_init, child_frame_id: body
  - `/cloud_registered` (sensor_msgs/PointCloud2) - 在camera_init坐标系下的配准点云
- **TF发布**: camera_init -> body

### 2. global_localization_node (Python)
- **功能**: 使用ICP在全局地图中定位，计算map到odom的变换
- **输入**:
  - `/cloud_registered` - 当前扫描（odom坐标系）
  - `/Odometry` - 里程计位姿
  - `/initialpose` - 初始位姿估计（来自RVIZ或命令行）
- **输出**:
  - `/map_to_odom` (nav_msgs/Odometry) - map到odom的变换矩阵
  - `/cur_scan_in_map` (sensor_msgs/PointCloud2) - **当前扫描在map坐标系下的可视化**
  - `/global_map_viz` (sensor_msgs/PointCloud2) - 全局地图可视化
- **核心功能**:
  ```python
  # 1. 接收初始位姿估计 T_map_to_livox_guess
  # 2. 获取当前里程计 T_odom_to_livox
  # 3. 计算ICP初始猜测: T_map_to_odom = T_map_to_livox * inv(T_odom_to_livox)
  # 4. 执行ICP配准: source(odom系点云) -> target(map系地图)
  # 5. 发布结果到 /map_to_odom
  # 6. 实时发布当前扫描在map中的位置 (pub_cur_scan_in_map)
  ```

### 3. transform_fusion_node (Python)
- **功能**: 融合FAST-LIO里程计和全局定位，维护完整TF树
- **输入**:
  - `/Odometry` - FAST-LIO高频里程计 (~100Hz)
  - `/map_to_odom` - 全局定位结果 (~0.5Hz)
- **TF发布**:
  - `map -> odom` - 全局定位校正变换（固定，仅在重定位时更新）
  - `odom -> livox_frame` - 实时雷达位姿（高频更新）

## 坐标系定义

| 坐标系 | 说明 | 特点 |
|--------|------|------|
| **map** | 全局地图坐标系 | 固定不变，对应预建地图的坐标系 |
| **odom** | 里程计坐标系 | 第一次成功定位的位置作为原点，之后通过全局定位修正 |
| **livox_frame** | 雷达当前位置 | 实时更新，由FAST-LIO里程计驱动 |
| **camera_init** | FAST-LIO初始坐标系 | 雷达上电时的位置，FAST-LIO内部使用 |
| **body** | FAST-LIO机器人本体 | 与livox_frame位置相同，FAST-LIO内部使用 |

## 使用流程

### 第一步：启动定位系统

```bash
cd /home/z/fastlio_location
source install/setup.bash

ros2 launch fast_lio localization_mid360.launch.py \
    map:=/home/z/rm_sim/src/rm_simulation/FAST_LIO/PCD/test.pcd
```

**启动后应该看到的日志**:
```
[INFO] [fastlio_mapping]: FAST-LIO node started
[INFO] [global_localization_node]: Map loaded: XXXXX points
[INFO] [global_localization_node]: Global Localization Node Ready.
[INFO] [transform_fusion_node]: Transform Fusion Node Initialized
```

### 第二步：启动传感器/播放数据

```bash
# 新终端
ros2 bag play your_data.bag

# 或启动实时传感器
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**注意**: 确保话题名称匹配:
- 点云话题: `/livox/lidar/pointcloud`
- IMU话题: `/livox/imu`

### 第三步：提供初始位姿

**方法1: 命令行**（假设机器人在原点）
```bash
ros2 run fast_lio publish_initial_pose.py 0 0 0 0 0 0
```

**方法2: RVIZ**
1. 在RVIZ中点击 "2D Pose Estimate" 工具
2. 在地图上点击并拖动设置初始位置和方向
3. **注意**: RVIZ发布的frame_id是camera_init，系统会自动转换

**成功定位后的日志**:
```
[INFO] [global_localization_node]: Executing Global Localization...
[INFO] [global_localization_node]: Localization Success! Fitness: 0.95
[INFO] [transform_fusion_node]: Updated map->odom transform
```

## RVIZ配置

### 必需的Display配置

1. **全局地图** (PointCloud2)
   - Topic: `/global_map_viz`
   - Frame: `map`
   - Style: Points
   - Size: 0.05
   - Color: 白色/灰色

2. **当前扫描在地图中的位置** (PointCloud2) ⭐ **核心可视化**
   - Topic: `/cur_scan_in_map`
   - Frame: `map`
   - Style: Points
   - Size: 0.03
   - Color: 红色/绿色（与地图区分）

3. **配准点云** (PointCloud2)
   - Topic: `/cloud_registered`
   - Frame: `camera_init`
   - Style: Points
   - Color: 蓝色

4. **TF树** (TF)
   - 显示所有坐标系关系
   - 验证TF树结构正确

5. **里程计轨迹** (Odometry)
   - Topic: `/Odometry`
   - Frame: `camera_init`
   - Arrow Length: 0.3
   - Color: 黄色

### Fixed Frame设置
- 设置为 `map` 以便在全局坐标系下观察

## 验证系统工作状态

### 1. 检查话题

```bash
# 查看所有话题
ros2 topic list

# 应该包含:
# /Odometry                  # FAST-LIO里程计
# /cloud_registered          # 配准点云
# /map_to_odom              # 全局定位结果
# /cur_scan_in_map          # 当前扫描在地图中 ⭐
# /global_map_viz           # 全局地图
# /initialpose              # 初始位姿输入

# 检查话题频率
ros2 topic hz /Odometry              # ~100Hz
ros2 topic hz /cur_scan_in_map       # ~100Hz (与里程计同步)
ros2 topic hz /map_to_odom           # 仅在重定位时发布
```

### 2. 检查TF树

```bash
# 查看TF树结构
ros2 run tf2_tools view_frames

# 生成的pdf中应该显示:
# map -> odom -> livox_frame
# camera_init -> body (独立树)

# 实时查看TF
ros2 run tf2_ros tf2_echo map livox_frame
```

### 3. 可视化检查

在RVIZ中检查 `/cur_scan_in_map`:
- ✅ **对齐**: 红色当前扫描应该与白色全局地图对齐
- ✅ **实时更新**: 扫描应该随机器人移动而移动
- ✅ **无漂移**: 长时间运行后扫描仍应与地图对齐（全局定位在修正）

## 故障排查

### 问题1: /cur_scan_in_map 没有数据

**原因**: 未成功定位或未初始化

**解决**:
```bash
# 检查是否成功定位
ros2 topic echo /map_to_odom --once

# 如果没有输出，重新提供初始位姿
ros2 run fast_lio publish_initial_pose.py 0 0 0 0 0 0
```

### 问题2: 当前扫描与地图不对齐

**原因**: 初始位姿偏差过大或定位失败

**解决**:
```bash
# 查看fitness分数
# 在global_localization_node终端查看日志

# 调低阈值重新启动
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/path/to/map.pcd \
    localization_threshold:=0.85

# 或提供更准确的初始位姿
```

### 问题3: TF树断裂

**检查**:
```bash
# 查看所有TF关系
ros2 run tf2_ros tf2_monitor

# 检查特定变换
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom livox_frame
```

### 问题4: sensor_msgs_py导入错误

**错误**: `ModuleNotFoundError: No module named 'sensor_msgs_py'`

**解决**:
```bash
# 安装依赖
pip3 install sensor_msgs_py

# 或使用系统包
sudo apt install ros-$ROS_DISTRO-sensor-msgs-py
```

## 参数调优

### global_localization_node 参数

```bash
# 启动时自定义参数
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/path/to/map.pcd \
    map_voxel_size:=0.3 \
    scan_voxel_size:=0.1 \
    localization_threshold:=0.90
```

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|---------|
| map_voxel_size | 0.4 | 地图降采样 | 精度高用0.2-0.3，速度快用0.5-0.6 |
| scan_voxel_size | 0.15 | 扫描降采样 | 特征多用0.1，速度快用0.2-0.3 |
| localization_threshold | 0.90 | ICP fitness阈值 | 环境好用0.93-0.95，差用0.85-0.88 |

## 关键代码位置

### 当前扫描可视化 (global_localization.py 第405-419行)

```python
def publish_visuals(self):
    """核心可视化功能：发布当前 scan 在 map 中的位置"""
    # 1. 拷贝点云防止修改原数据
    pcd_viz = o3d.geometry.PointCloud(cur_scan_o3d)

    # 2. 变换: points_map = T_map_to_odom * points_odom
    pcd_viz.transform(T_map_to_odom)

    # 3. 转换回 ROS 消息并发布 (frame_id = 'map')
    msg = o3d_to_pointcloud2(pcd_viz, 'map', self.get_clock().now().to_msg())
    self.pub_cur_scan_in_map.publish(msg)
```

### TF发布 (transform_fusion.py 第59-111行)

```python
def odom_callback(self, msg):
    # 1. 发布 odom -> livox_frame (高频)
    t_odom_livox = TransformStamped()
    t_odom_livox.header.frame_id = 'odom'
    t_odom_livox.child_frame_id = 'livox_frame'
    # ... 设置变换
    self.tf_broadcaster.sendTransform(t_odom_livox)

    # 2. 发布 map -> odom (修正变换)
    t_map_odom = TransformStamped()
    t_map_odom.header.frame_id = 'map'
    t_map_odom.child_frame_id = 'odom'
    # ... 从 T_map_to_odom 提取
    self.tf_broadcaster.sendTransform(t_map_odom)
```

## 完整测试示例

```bash
# Terminal 1: 启动系统
cd /home/z/fastlio_location
source install/setup.bash
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/home/z/rm_sim/src/rm_simulation/FAST_LIO/PCD/test.pcd

# Terminal 2: 播放数据
ros2 bag play your_data.bag

# Terminal 3: 初始化定位
ros2 run fast_lio publish_initial_pose.py 0 0 0 0 0 0

# Terminal 4: 监控状态
ros2 topic hz /cur_scan_in_map
ros2 run tf2_ros tf2_echo map livox_frame

# Terminal 5: 查看RVIZ
# 在RVIZ中添加 /cur_scan_in_map 和 /global_map_viz
# 验证红色扫描与白色地图对齐
```

## 总结

当前系统实现了：
1. ✅ **TF树结构**: map->odom->livox_frame + camera_init->body
2. ✅ **全局定位**: ICP配准计算map到odom的变换
3. ✅ **实时可视化**: `/cur_scan_in_map` 显示当前扫描在地图中的位置
4. ✅ **高频融合**: transform_fusion维护完整TF树

关键特性：
- **odom坐标系**: 第一次成功定位的位置作为原点
- **实时校正**: 全局定位持续修正map->odom变换
- **可视化验证**: `/cur_scan_in_map` 方便检查定位精度
- **独立TF树**: camera_init->body保持FAST-LIO原有逻辑
