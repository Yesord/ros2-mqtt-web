#!/bin/bash

# WebCom虚拟环境激活脚本
# 使用conda激活WebCom虚拟环境

echo "正在激活WebCom虚拟环境..."

# 检查conda是否已安装
if ! command -v conda &> /dev/null; then
    echo "错误: conda未安装或不在PATH中"
    echo "请先安装conda或miniconda"
    return 1
fi

# 初始化conda (必须在激活前执行)
echo "初始化conda环境..."
eval "$(conda shell.bash hook)"

# 检查WebCom环境是否存在
if conda env list | grep -q "WebCom"; then
    echo "找到WebCom环境，正在激活..."
    conda activate WebCom
    echo "WebCom虚拟环境已激活！"
    echo "当前环境: $CONDA_DEFAULT_ENV"
else
    echo "警告: WebCom环境不存在"
    echo "可用的conda环境:"
    conda env list
    echo ""
    echo "要创建WebCom环境，请运行:"
    echo "conda create -n WebCom python=3.8"
    return 1
fi

# 显示Python版本和路径
echo "Python版本: $(python --version)"
echo "Python路径: $(which python)"

# 进入该脚本所在目录的子目录dev_ws
echo "进入dev_ws工作目录..."

# 获取脚本的真实路径
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    # 脚本被source执行
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    # 脚本被直接执行
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

echo "脚本目录: $SCRIPT_DIR"

cd "$SCRIPT_DIR/dev_ws" || {
    echo "错误: 无法进入dev_ws目录 ($SCRIPT_DIR/dev_ws)"
    echo "请检查dev_ws目录是否存在"
    return 1
}

echo "当前工作目录: $(pwd)"

# 如果存在ROS2环境，则source相关配置
if [ -f "install/setup.bash" ]; then
    echo "找到ROS2安装环境，正在加载..."
    source install/setup.bash
    echo "ROS2环境已加载"
elif [ -f "devel/setup.bash" ]; then
    echo "找到ROS开发环境，正在加载..."
    source devel/setup.bash
    echo "ROS开发环境已加载"
else
    echo "未找到ROS2环境配置文件"
    echo "如果这是一个ROS2工作空间，请先构建项目："
    echo "colcon build"
fi

echo "WebCom开发环境已完全激活！"
echo "=========================================="
echo "环境信息:"
echo "- Conda环境: $CONDA_DEFAULT_ENV"
echo "- Python版本: $(python --version)"
echo "- 工作目录: $(pwd)"
if [ -n "$ROS_DISTRO" ]; then
    echo "- ROS版本: $ROS_DISTRO"
    echo "- ROS Domain ID: ${ROS_DOMAIN_ID:-0}"
fi
if [ -n "$AMENT_PREFIX_PATH" ]; then
    echo "- ROS2环境: 已激活"
fi
echo "=========================================="

# 显示一些ROS2常用命令提示
# if [ -n "$ROS_DISTRO" ]; then
#     echo "常用ROS2命令:"
#     echo "- ros2 node list        # 列出所有节点"
#     echo "- ros2 topic list       # 列出所有话题"
#     echo "- ros2 service list     # 列出所有服务"
#     echo "- colcon build          # 构建工作空间"
#     echo "- colcon build --symlink-install  # 符号链接安装"
#     echo "=========================================="
# fi
