# 快速开始指南

## 第一步：安装依赖

```bash
cd /home/z/fastlio_location/src/FAST_LIO_ROS2
./install_dependencies.sh
```

## 第二步：编译

```bash
cd /home/z/fastlio_location
colcon build --packages-select fast_lio
source install/setup.bash
```

## 第三步：准备地图

### 选项A：使用现有地图
如果你已经有PCD格式的地图文件，直接使用即可。

### 选项B：先建图
```bash
# 启动建图
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 播放数据包或启动传感器
ros2 bag play your_data.bag

# 建图完成后保存
ros2 service call /map_save std_srvs/srv/Trigger
```

## 第四步：启动定位

```bash
# 启动定位系统（替换为你的地图路径）
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/path/to/your/map.pcd
```

## 第五步：提供初始位姿

在新终端中：

```bash
# 使用命令行（替换为实际坐标）
ros2 run fast_lio publish_initial_pose.py 0 0 0 0 0 0

# 或者在RVIZ中使用"2D Pose Estimate"工具
```

## 第六步：运行数据

```bash
# 播放rosbag
ros2 bag play your_localization_data.bag

# 或启动实时传感器
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 验证定位是否成功

检查以下内容：

1. **终端输出**：应该看到 "Initial localization succeeded!"
2. **RVIZ可视化**：点云应该与地图对齐
3. **话题检查**：
   ```bash
   # 查看融合后的位姿
   ros2 topic echo /localization

   # 查看全局定位结果
   ros2 topic echo /map_to_odom
   ```

## 常见命令

```bash
# 列出所有话题
ros2 topic list

# 查看TF树
ros2 run tf2_tools view_frames

# 查看节点信息
ros2 node list
ros2 node info /global_localization_node

# 检查参数
ros2 param list /global_localization_node
ros2 param get /global_localization_node localization_threshold
```

## 故障排查

### 问题1：找不到地图文件
```bash
# 检查文件是否存在
ls -lh /path/to/your/map.pcd
```

### 问题2：定位一直失败
```bash
# 尝试降低阈值
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/path/to/your/map.pcd \
    localization_threshold:=0.85
```

### 问题3：Python模块导入错误
```bash
# 验证安装
python3 -c "import open3d; print(open3d.__version__)"

# 重新安装
pip3 install open3d==0.18.0 --force-reinstall
```

## 完整示例

```bash
# Terminal 1: 启动定位
ros2 launch fast_lio localization_mid360.launch.py \
    map:=/home/z/maps/warehouse.pcd

# Terminal 2: 提供初始位姿 (假设机器人在原点)
ros2 run fast_lio publish_initial_pose.py 0 0 0 0 0 0

# Terminal 3: 播放数据
ros2 bag play warehouse_test.bag

# Terminal 4: 监控位姿
ros2 topic echo /localization
```

详细文档请参考：[LOCALIZATION_README.md](LOCALIZATION_README.md)
