#!/bin/bash
# 一键测试FAST-LIO重定位系统

echo "=========================================="
echo "FAST-LIO 重定位系统测试脚本"
echo "=========================================="
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: 未检测到ROS2环境"
    echo "请先运行: source /opt/ros/<your_distro>/setup.bash"
    exit 1
fi

# 检查工作空间
if [ ! -f "install/setup.bash" ]; then
    echo "❌ 错误: 请在工作空间根目录运行此脚本"
    echo "cd /home/z/fastlio_location"
    exit 1
fi

# Source工作空间
source install/setup.bash

echo "✅ 环境检查完成"
echo ""

# 检查地图文件
MAP_PATH="/home/z/rm_sim/src/rm_simulation/FAST_LIO/PCD/test.pcd"
if [ ! -f "$MAP_PATH" ]; then
    echo "⚠️  警告: 默认地图文件不存在: $MAP_PATH"
    echo "请修改脚本中的MAP_PATH变量或提供地图路径"
    read -p "输入地图文件路径 (或按Enter跳过): " USER_MAP
    if [ -n "$USER_MAP" ]; then
        MAP_PATH="$USER_MAP"
    fi
fi

# 检查Python依赖
echo "检查Python依赖..."
python3 -c "import open3d" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少依赖: open3d"
    echo "安装命令: pip3 install open3d==0.18.0"
    exit 1
fi

python3 -c "import sensor_msgs_py" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少依赖: sensor_msgs_py"
    echo "安装命令: pip3 install sensor_msgs_py"
    exit 1
fi

echo "✅ 依赖检查完成"
echo ""

# 询问测试模式
echo "请选择测试模式:"
echo "1) 完整测试 (启动所有节点 + RVIZ)"
echo "2) 仅启动节点 (不启动RVIZ)"
echo "3) 检查系统状态"
echo ""
read -p "选择 [1-3]: " MODE

case $MODE in
    1)
        echo ""
        echo "=========================================="
        echo "启动完整系统..."
        echo "=========================================="
        echo ""
        echo "地图文件: $MAP_PATH"
        echo ""
        echo "提示:"
        echo "1. 启动后在新终端播放数据包或启动传感器"
        echo "2. 使用以下命令提供初始位姿:"
        echo "   ros2 run fast_lio publish_initial_pose.py 0 0 0 0 0 0"
        echo "3. 或在RVIZ中使用 '2D Pose Estimate' 工具"
        echo ""
        read -p "按Enter继续..."

        ros2 launch fast_lio localization_mid360.launch.py \
            map:="$MAP_PATH"
        ;;

    2)
        echo ""
        echo "=========================================="
        echo "启动节点 (无RVIZ)..."
        echo "=========================================="
        echo ""

        ros2 launch fast_lio localization_mid360.launch.py \
            map:="$MAP_PATH" \
            rviz:=false
        ;;

    3)
        echo ""
        echo "=========================================="
        echo "系统状态检查"
        echo "=========================================="
        echo ""

        echo "--- ROS2节点 ---"
        ros2 node list
        echo ""

        echo "--- ROS2话题 ---"
        ros2 topic list | grep -E "(Odometry|cloud|map|initial)"
        echo ""

        echo "--- TF树 ---"
        timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "✅ map->odom 变换正常"
        else
            echo "⚠️  map->odom 变换不存在 (可能未初始化)"
        fi
        echo ""

        timeout 2 ros2 run tf2_ros tf2_echo odom livox_frame 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "✅ odom->livox_frame 变换正常"
        else
            echo "⚠️  odom->livox_frame 变换不存在"
        fi
        echo ""

        echo "--- 话题频率 ---"
        echo "检查 /Odometry 频率..."
        timeout 3 ros2 topic hz /Odometry 2>/dev/null | head -3
        echo ""

        echo "检查 /cur_scan_in_map 频率..."
        timeout 3 ros2 topic hz /cur_scan_in_map 2>/dev/null | head -3
        echo ""

        echo "=========================================="
        echo "检查完成"
        echo "=========================================="
        ;;

    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
