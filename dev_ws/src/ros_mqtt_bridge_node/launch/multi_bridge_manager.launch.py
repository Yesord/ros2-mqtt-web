#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包的共享目录
    package_dir = get_package_share_directory('ros_mqtt_bridge_node')
    
    # 默认配置文件路径
    default_config_file = os.path.join(package_dir, 'config', 'multi_bridge_config.yaml')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='多话题桥接配置文件路径'
    )
    
    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='localhost',
        description='MQTT服务器地址'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT服务器端口'
    )
    
    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix',
        default_value='ros2',
        description='MQTT主题前缀'
    )
    
    # 多话题桥接管理器节点
    multi_bridge_node = Node(
        package='ros_mqtt_bridge_node',
        executable='multi_bridge_manager',
        name='multi_bridge_manager',
        output='screen',
        parameters=[{
            'config_file_path': LaunchConfiguration('config_file'),
            'mqtt_broker_host': LaunchConfiguration('mqtt_host'),
            'mqtt_broker_port': LaunchConfiguration('mqtt_port'),
            'mqtt_topic_prefix': LaunchConfiguration('topic_prefix'),
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        mqtt_host_arg,
        mqtt_port_arg,
        topic_prefix_arg,
        multi_bridge_node
    ])
