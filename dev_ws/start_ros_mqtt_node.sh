#!/bin/bash
# 保证同domain
export ROS_DOMAIN_ID=66

# 启动 ROS2-MQTT 桥接节点脚本
set -e

# 进入工作区
cd "$(dirname "$0")"

source install/setup.bash

# 启动ros_mqtt_bridge_node
ros2 run ros_mqtt_bridge_node multi_bridge_manager
