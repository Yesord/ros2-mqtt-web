#!/bin/bash

# 简化的Hello World MQTT Bridge运行脚本
# 使用系统Python以避免版本冲突

echo "=== Hello World MQTT Bridge 启动脚本 ==="
echo ""

# 确保不在conda环境中
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "检测到conda环境，正在退出以使用系统Python..."
    conda deactivate
fi

# 设置ROS2环境
echo "设置ROS2环境..."
source /opt/ros/foxy/setup.bash

# 检查是否已构建包
if [ ! -d "install/ros_mqtt_bridge_node" ]; then
    echo "包尚未构建，正在构建..."
    ./build_mqtt_bridge.sh
    if [ $? -ne 0 ]; then
        echo "构建失败，退出。"
        exit 1
    fi
fi

# 源工作空间设置
echo "加载工作空间..."
source install/setup.bash

# 显示运行信息
echo ""
echo "启动Hello World MQTT Bridge节点..."
echo "配置信息:"
echo "  - MQTT服务器: localhost:1883"
echo "  - 主题前缀: ros2"
echo "  - 客户端ID: hello_world_bridge"
echo ""
echo "MQTT消息将发布到: ros2/hello_world"
echo "可以使用以下命令测试接收:"
echo "  mosquitto_sub -h localhost -t 'ros2/hello_world'"
echo ""

# 运行节点
echo "启动节点..."
ros2 run ros_mqtt_bridge_node hello_world_mqtt_bridge
