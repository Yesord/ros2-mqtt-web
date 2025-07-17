#!/bin/bash

# 构建ROS2包的脚本

echo "正在构建ros_mqtt_bridge_node包..."

# 确保使用系统Python环境
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "检测到conda环境，临时禁用以避免Python版本冲突..."
    conda deactivate
fi

# 安装paho-mqtt到系统Python（如果没有安装）
echo "检查并安装paho-mqtt..."
python3 -c "import paho.mqtt.client" 2>/dev/null || {
    echo "安装paho-mqtt..."
    pip3 install paho-mqtt || {
        echo "尝试使用用户目录安装..."
        pip3 install --user paho-mqtt
    }
}

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 进入工作空间
cd /home/tp/Project/WebCom/dev_ws

# 使用系统Python进行构建
echo "使用系统Python进行构建..."
colcon build --packages-select ros_mqtt_bridge_node --symlink-install

# 检查构建结果
if [ $? -eq 0 ]; then
    echo "构建成功！"
    echo "请运行以下命令来源环境设置："
    echo "source install/setup.bash"
    echo ""
    echo "然后可以运行节点："
    echo "ros2 run ros_mqtt_bridge_node hello_world_mqtt_bridge"
    echo ""
    echo "或者使用启动文件："
    echo "ros2 launch ros_mqtt_bridge_node hello_world_mqtt_bridge.launch.py"
else
    echo "构建失败！请检查错误信息。"
fi
