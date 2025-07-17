#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('ros_mqtt_bridge_node')
    
    # 声明启动参数
    mqtt_broker_host_arg = DeclareLaunchArgument(
        'mqtt_broker_host',
        default_value='localhost',
        description='MQTT broker hostname or IP address'
    )
    
    mqtt_broker_port_arg = DeclareLaunchArgument(
        'mqtt_broker_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    mqtt_topic_prefix_arg = DeclareLaunchArgument(
        'mqtt_topic_prefix',
        default_value='ros2',
        description='MQTT topic prefix'
    )
    
    mqtt_client_id_arg = DeclareLaunchArgument(
        'mqtt_client_id',
        default_value='hello_world_bridge',
        description='MQTT client ID'
    )
    
    mqtt_username_arg = DeclareLaunchArgument(
        'mqtt_username',
        default_value='',
        description='MQTT username (optional)'
    )
    
    mqtt_keepalive_arg = DeclareLaunchArgument(
        'mqtt_keepalive',
        default_value='60',
        description='MQTT keepalive interval in seconds'
    )
    
    mqtt_qos_arg = DeclareLaunchArgument(
        'mqtt_qos',
        default_value='1',
        description='MQTT Quality of Service level'
    )
    
    # 创建hello_world_mqtt_bridge节点
    hello_world_mqtt_bridge_node = Node(
        package='ros_mqtt_bridge_node',
        executable='hello_world_mqtt_bridge',
        name='hello_world_mqtt_bridge',
        output='screen',
        parameters=[{
            'mqtt_broker_host': LaunchConfiguration('mqtt_broker_host'),
            'mqtt_broker_port': LaunchConfiguration('mqtt_broker_port'),
            'mqtt_topic_prefix': LaunchConfiguration('mqtt_topic_prefix'),
            'mqtt_client_id': LaunchConfiguration('mqtt_client_id'),
            'mqtt_username': LaunchConfiguration('mqtt_username'),
            'mqtt_password': LaunchConfiguration('mqtt_password'),
            'mqtt_keepalive': LaunchConfiguration('mqtt_keepalive'),
            'mqtt_qos': LaunchConfiguration('mqtt_qos'),
        }]
    )
    
    return LaunchDescription([
        mqtt_broker_host_arg,
        mqtt_broker_port_arg,
        mqtt_topic_prefix_arg,
        mqtt_client_id_arg,
        mqtt_username_arg,
        mqtt_password_arg,
        mqtt_keepalive_arg,
        mqtt_qos_arg,
        hello_world_mqtt_bridge_node
    ])
