#!/bin/bash

# Local Planner C++ 依赖安装脚本
# 对应Python版本的依赖安装

set -e

echo "=========================================="
echo "Local Planner C++ 依赖安装"
echo "=========================================="

# 检测操作系统
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "检测到Linux系统"
    
    # 检测Ubuntu版本
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$NAME
        VER=$VERSION_ID
        echo "操作系统: $OS $VER"
    fi
    
    # 检测ROS版本
    if [ -z "$ROS_DISTRO" ]; then
        echo "错误: 未检测到ROS环境，请先安装ROS"
        echo "参考: http://wiki.ros.org/Installation"
        exit 1
    else
        echo "检测到ROS版本: $ROS_DISTRO"
    fi
    
else
    echo "错误: 不支持的操作系统 $OSTYPE"
    exit 1
fi

echo "=========================================="
echo "更新包管理器"
echo "=========================================="
sudo apt update

echo "=========================================="
echo "安装基础编译工具"
echo "=========================================="
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git

echo "=========================================="
echo "安装ROS依赖 (对应Python rospy等)"
echo "=========================================="
sudo apt install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs

echo "=========================================="
echo "安装线性代数库 (对应Python numpy)"
echo "=========================================="
sudo apt install -y libeigen3-dev

echo "=========================================="
echo "安装CasADi优化库 (对应Python casadi)"
echo "=========================================="

# 检查是否已安装CasADi
if pkg-config --exists casadi; then
    echo "CasADi已安装，版本: $(pkg-config --modversion casadi)"
else
    echo "正在安装CasADi..."
    
    # 尝试从apt安装
    if sudo apt install -y libcasadi-dev; then
        echo "CasADi从apt安装成功"
    else
        echo "apt安装失败，从源码编译CasADi..."
        
        # 创建临时目录
        TEMP_DIR=$(mktemp -d)
        cd $TEMP_DIR
        
        # 下载CasADi源码
        git clone https://github.com/casadi/casadi.git
        cd casadi
        
        # 创建编译目录
        mkdir build
        cd build
        
        # 配置编译选项
        cmake .. \
            -DCMAKE_BUILD_TYPE=Release \
            -DWITH_PYTHON=OFF \
            -DWITH_MATLAB=OFF \
            -DWITH_EXAMPLES=OFF \
            -DWITH_DOC=OFF
        
        # 编译并安装
        make -j$(nproc)
        sudo make install
        
        # 更新动态链接库
        sudo ldconfig
        
        # 清理临时文件
        cd ~
        rm -rf $TEMP_DIR
        
        echo "CasADi从源码编译安装完成"
    fi
fi

echo "=========================================="
echo "验证安装"
echo "=========================================="

# 验证ROS
echo "验证ROS安装..."
if command -v roscore &> /dev/null; then
    echo "✓ ROS已正确安装"
else
    echo "✗ ROS安装失败"
    exit 1
fi

# 验证Eigen3
echo "验证Eigen3安装..."
if pkg-config --exists eigen3; then
    echo "✓ Eigen3已安装，版本: $(pkg-config --modversion eigen3)"
else
    echo "✗ Eigen3安装失败"
    exit 1
fi

# 验证CasADi
echo "验证CasADi安装..."
if pkg-config --exists casadi; then
    echo "✓ CasADi已安装，版本: $(pkg-config --modversion casadi)"
else
    echo "✗ CasADi安装失败"
    exit 1
fi

# 验证编译工具
echo "验证编译工具..."
if command -v cmake &> /dev/null && command -v make &> /dev/null; then
    echo "✓ 编译工具已安装"
    echo "  - CMake版本: $(cmake --version | head -n1)"
    echo "  - Make版本: $(make --version | head -n1)"
else
    echo "✗ 编译工具安装失败"
    exit 1
fi

echo "=========================================="
echo "安装完成"
echo "=========================================="
echo ""
echo "所有依赖已成功安装！"
echo ""
echo "下一步操作:"
echo "1. cd ~/catkin_ws/src"
echo "2. git clone <your-repo> local_planner_cpp"
echo "3. cd ~/catkin_ws && catkin build local_planner_cpp"
echo "4. source devel/setup.bash"
echo "5. roslaunch local_planner_cpp local_planner.launch"
echo ""
echo "如果遇到问题，请查看README.md中的故障排除部分"