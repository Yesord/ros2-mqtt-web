#!/bin/bash
 
# 激活conda环境
source ~/anaconda3/etc/profile.d/conda.sh  # 根据您的conda安装路径调整
conda activate WebCom  # 替换为您的conda环境名
 
# 设置ROS2环境
source /opt/ros/foxy/setup.bash
source install/setup.bash
 
# 设置PYTHONPATH包含conda环境的库
export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"
 
# 运行节点
ros2 run ros_mqtt_bridge_node hello_world_mqtt_bridge

