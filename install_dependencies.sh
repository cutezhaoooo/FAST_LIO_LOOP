#!/bin/bash
# 安装FAST-LIO重定位功能的依赖项

echo "=========================================="
echo "安装FAST-LIO重定位功能依赖项"
echo "=========================================="

# 检测ROS2版本
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到ROS2环境，请先source ROS2 setup.bash"
    exit 1
fi

echo "检测到ROS2版本: $ROS_DISTRO"
echo ""

# 安装ROS2依赖
echo "1. 安装ROS2依赖包..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    python3-pip

if [ $? -ne 0 ]; then
    echo "错误: ROS2依赖包安装失败"
    exit 1
fi

echo "✓ ROS2依赖包安装成功"
echo ""

# 安装Python依赖
echo "2. 安装Python依赖..."
pip3 install --upgrade pip

echo "  - 安装numpy..."
pip3 install numpy

echo "  - 安装scipy..."
pip3 install scipy

echo "  - 安装open3d..."
pip3 install open3d==0.18.0

if [ $? -ne 0 ]; then
    echo "警告: Open3D 0.18.0安装失败，尝试安装最新版本..."
    pip3 install open3d --upgrade
fi

echo ""
echo "3. 验证安装..."

# 验证Python包
python3 -c "import numpy; print('✓ numpy版本:', numpy.__version__)"
python3 -c "import scipy; print('✓ scipy版本:', scipy.__version__)"
python3 -c "import open3d; print('✓ open3d版本:', open3d.__version__)"

echo ""
echo "=========================================="
echo "依赖安装完成！"
echo "=========================================="
echo ""
echo "下一步："
echo "1. 编译工作空间:"
echo "   cd ~/fastlio_location"
echo "   colcon build --packages-select fast_lio"
echo "   source install/setup.bash"
echo ""
echo "2. 查看使用文档:"
echo "   cat ~/fastlio_location/src/FAST_LIO_ROS2/LOCALIZATION_README.md"
echo ""
