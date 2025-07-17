#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from datetime import datetime
from .mqtt_interface import MQTTInterface, MQTTMessageBuilder
from .ros_node_data_reader import HelloWorldNodeDataReader


class HelloWorldMQTTBridge(Node):
    """Hello World MQTT桥接节点 - 使用模块化设计"""
    
    def __init__(self):
        super().__init__('hello_world_mqtt_bridge')
        
        # 声明ROS2参数
        self._declare_parameters()
        
        # 获取MQTT配置
        mqtt_config = self._get_mqtt_config()
        
        # 初始化MQTT接口
        self.mqtt_interface = MQTTInterface(mqtt_config, self.get_logger())
        
        # 设置MQTT回调函数
        self.mqtt_interface.set_on_connect_callback(self._on_mqtt_connect)
        self.mqtt_interface.set_on_disconnect_callback(self._on_mqtt_disconnect)
        self.mqtt_interface.set_on_publish_callback(self._on_mqtt_publish)
        
        # 初始化ROS节点数据读取器
        self.data_reader = HelloWorldNodeDataReader(self)
        
        # 注册Hello World消息回调函数
        self.data_reader.register_hello_world_callback(self._on_hello_world_message)
        
        # 连接状态和统计信息
        self.mqtt_connected = False
        self.total_published_messages = 0
        self.start_time = datetime.now()
        
        # 连接到MQTT服务器
        self._connect_mqtt()
        
        # 创建定时器定期检查连接状态
        self.connection_check_timer = self.create_timer(10.0, self._check_mqtt_connection)
        
        # 创建定时器定期发布统计信息
        self.stats_timer = self.create_timer(30.0, self._publish_statistics)
        
        # 创建定时器定期发布心跳
        self.heartbeat_timer = self.create_timer(60.0, self._publish_heartbeat)
        
        self.get_logger().info('=== Hello World MQTT Bridge 节点已启动 ===')
        self.get_logger().info(f'MQTT服务器: {mqtt_config["broker_host"]}:{mqtt_config["broker_port"]}')
        self.get_logger().info(f'主题前缀: {mqtt_config.get("topic_prefix", "ros2")}')
        self.get_logger().info(f'客户端ID: {mqtt_config.get("client_id", "hello_world_bridge")}')
    
    def _declare_parameters(self):
        """声明ROS2参数"""
        self.declare_parameter('mqtt_broker_host', 'localhost')
        self.declare_parameter('mqtt_broker_port', 1883) # MQTT默认1883端口转发
        self.declare_parameter('mqtt_topic_prefix', 'ros2')
        self.declare_parameter('mqtt_client_id', 'hello_world_bridge')
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('mqtt_qos', 1)
        self.declare_parameter('enable_statistics', True)
        self.declare_parameter('enable_heartbeat', True)
        self.declare_parameter('message_history_size', 100)
    
    def _get_mqtt_config(self) -> dict:
        """获取MQTT配置"""
        config = {
            'broker_host': self.get_parameter('mqtt_broker_host').value,
            'broker_port': self.get_parameter('mqtt_broker_port').value,
            'topic_prefix': self.get_parameter('mqtt_topic_prefix').value,
            'client_id': self.get_parameter('mqtt_client_id').value,
            'keepalive': self.get_parameter('mqtt_keepalive').value,
            'qos': self.get_parameter('mqtt_qos').value
        }
        
        username = self.get_parameter('mqtt_username').value
        password = self.get_parameter('mqtt_password').value
        
        if username and password:
            config['username'] = username
            config['password'] = password
        
        return config
    
    def _connect_mqtt(self):
        """连接到MQTT服务器"""
        self.get_logger().info('正在连接到MQTT服务器...')
        success = self.mqtt_interface.connect()
        if not success:
            self.get_logger().error('初始MQTT连接失败，将在后台重试')
    
    def _on_mqtt_connect(self, rc: int):
        """MQTT连接回调"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info('✓ MQTT连接成功')
            
            # 发布节点启动消息
            self._publish_node_status('connected')
            
            # 订阅控制话题（如果需要）
            self._subscribe_control_topics()
        else:
            self.mqtt_connected = False
            self.get_logger().error(f'✗ MQTT连接失败，返回码: {rc}')
    
    def _on_mqtt_disconnect(self, rc: int):
        """MQTT断开连接回调"""
        self.mqtt_connected = False
        if rc != 0:
            self.get_logger().error(f'✗ MQTT意外断开，返回码: {rc}')
        else:
            self.get_logger().info('MQTT正常断开')
    
    def _on_mqtt_publish(self, mid: int):
        """MQTT发布回调"""
        self.get_logger().debug(f'MQTT消息发布成功，消息ID: {mid}')
    
    def _subscribe_control_topics(self):
        """订阅控制话题"""
        config = self.mqtt_interface.get_config()
        control_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/control'
        
        # 设置消息接收回调
        self.mqtt_interface.set_on_message_callback(self._on_control_message)
        
        # 订阅控制话题
        success = self.mqtt_interface.subscribe(control_topic, qos=config.get('qos', 1))
        if success:
            self.get_logger().info(f'已订阅控制话题: {control_topic}')
    
    def _on_control_message(self, topic: str, payload: bytes):
        """处理控制消息"""
        try:
            message = json.loads(payload.decode('utf-8'))
            command = message.get('command')
            
            if command == 'get_stats':
                self._publish_statistics()
            elif command == 'get_history':
                self._publish_message_history()
            elif command == 'reset_stats':
                self._reset_statistics()
            else:
                self.get_logger().warning(f'未知控制命令: {command}')
                
        except Exception as e:
            self.get_logger().error(f'处理控制消息时发生错误: {str(e)}')
    
    def _on_hello_world_message(self, topic_name: str, message_data: str, message_count: int):
        """处理Hello World消息"""
        if not self.mqtt_connected:
            self.get_logger().debug('MQTT未连接，跳过消息发布')
            return
        
        try:
            # 使用消息构建器创建标准化消息
            mqtt_message = MQTTMessageBuilder.create_ros_message(
                source_node='hello_world_publisher',
                source_topic=topic_name,
                data=message_data,
                message_id=message_count,
                frame_id='hello_world_frame'
            )
            
            # 添加额外的元数据
            mqtt_message.update({
                'bridge_node': self.get_name(),
                'bridge_message_count': self.total_published_messages + 1,
                'bridge_uptime_seconds': (datetime.now() - self.start_time).total_seconds()
            })
            
            # 构造MQTT主题
            config = self.mqtt_interface.get_config()
            mqtt_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/data'
            
            # 发布到MQTT
            success = self.mqtt_interface.publish(
                mqtt_topic,
                mqtt_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.total_published_messages += 1
                self.get_logger().info(f'✓ 消息已转发到MQTT - 主题: {mqtt_topic}')
                self.get_logger().info(f'  内容: {message_data}')
            else:
                self.get_logger().error('✗ MQTT消息发布失败')
                
        except Exception as e:
            self.get_logger().error(f'处理Hello World消息时发生错误: {str(e)}')
    
    def _check_mqtt_connection(self):
        """检查MQTT连接状态"""
        if not self.mqtt_interface.is_connected():
            self.get_logger().warning('MQTT连接丢失，尝试重新连接...')
            self._connect_mqtt()
        else:
            # 检查Hello World话题是否活跃
            if not self.data_reader.is_topic_active('hello_world', timeout_seconds=10.0):
                self.get_logger().warning('Hello World话题似乎不活跃，请检查发布者是否运行')
    
    def _publish_statistics(self):
        """发布统计信息"""
        if not self.mqtt_connected:
            return
        
        try:
            # 获取Hello World节点统计信息
            hello_world_stats = self.data_reader.get_hello_world_statistics()
            
            # 获取系统统计信息
            uptime = (datetime.now() - self.start_time).total_seconds()
            
            # 构建完整的统计信息
            stats_message = MQTTMessageBuilder.create_status_message(
                node_name='hello_world_mqtt_bridge',
                status='running',
                uptime_seconds=uptime,
                hello_world_stats=hello_world_stats,
                total_published_messages=self.total_published_messages,
                mqtt_connected=self.mqtt_connected,
                mqtt_config=self.mqtt_interface.get_config()
            )
            
            # 发布统计信息
            config = self.mqtt_interface.get_config()
            stats_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/statistics'
            
            success = self.mqtt_interface.publish(
                stats_topic,
                stats_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().debug('统计信息已发布')
            
        except Exception as e:
            self.get_logger().error(f'发布统计信息时发生错误: {str(e)}')
    
    def _publish_heartbeat(self):
        """发布心跳信息"""
        if not self.mqtt_connected:
            return
        
        if not self.get_parameter('enable_heartbeat').value:
            return
        
        try:
            heartbeat_message = MQTTMessageBuilder.create_status_message(
                node_name='hello_world_mqtt_bridge',
                status='alive',
                uptime_seconds=(datetime.now() - self.start_time).total_seconds(),
                total_published_messages=self.total_published_messages,
                last_hello_world_message=self.data_reader.get_current_hello_world_message()
            )
            
            config = self.mqtt_interface.get_config()
            heartbeat_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/heartbeat'
            
            success = self.mqtt_interface.publish(
                heartbeat_topic,
                heartbeat_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().debug('心跳信息已发布')
            
        except Exception as e:
            self.get_logger().error(f'发布心跳信息时发生错误: {str(e)}')
    
    def _publish_message_history(self):
        """发布消息历史"""
        if not self.mqtt_connected:
            return
        
        try:
            history = self.data_reader.get_hello_world_history(limit=10)  # 获取最近10条消息
            
            history_message = MQTTMessageBuilder.create_status_message(
                node_name='hello_world_mqtt_bridge',
                status='history',
                message_history=history,
                total_history_size=len(self.data_reader.get_hello_world_history())
            )
            
            config = self.mqtt_interface.get_config()
            history_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/history'
            
            success = self.mqtt_interface.publish(
                history_topic,
                history_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().info('消息历史已发布')
            
        except Exception as e:
            self.get_logger().error(f'发布消息历史时发生错误: {str(e)}')
    
    def _reset_statistics(self):
        """重置统计信息"""
        self.total_published_messages = 0
        self.start_time = datetime.now()
        self.get_logger().info('统计信息已重置')
        
        # 发布重置确认
        if self.mqtt_connected:
            reset_message = MQTTMessageBuilder.create_status_message(
                node_name='hello_world_mqtt_bridge',
                status='statistics_reset',
                reset_time=datetime.now().isoformat()
            )
            
            config = self.mqtt_interface.get_config()
            reset_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/reset_confirm'
            
            self.mqtt_interface.publish(reset_topic, reset_message, qos=config.get('qos', 1))
    
    def _publish_node_status(self, status: str):
        """发布节点状态"""
        if not self.mqtt_connected:
            return
        
        try:
            status_message = MQTTMessageBuilder.create_status_message(
                node_name='hello_world_mqtt_bridge',
                status=status,
                mqtt_connected=self.mqtt_connected,
                total_published_messages=self.total_published_messages,
                uptime_seconds=(datetime.now() - self.start_time).total_seconds()
            )
            
            config = self.mqtt_interface.get_config()
            status_topic = f'{config.get("topic_prefix", "ros2")}/hello_world/bridge_status'
            
            success = self.mqtt_interface.publish(
                status_topic,
                status_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().debug(f'节点状态已发布: {status}')
            
        except Exception as e:
            self.get_logger().error(f'发布节点状态时发生错误: {str(e)}')
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info('正在关闭Hello World MQTT Bridge节点...')
        
        # 发布节点关闭状态
        self._publish_node_status('shutdown')
        
        # 给一点时间让最后的消息发送完成
        time.sleep(1.0)
        
        # 销毁数据读取器
        if hasattr(self, 'data_reader'):
            self.data_reader.destroy()
        
        # 断开MQTT连接
        if hasattr(self, 'mqtt_interface'):
            self.mqtt_interface.disconnect()
        
        # 调用父类的销毁方法
        super().destroy_node()
        
        self.get_logger().info('✓ Hello World MQTT Bridge节点已关闭')


def main(args=None):
    """主函数"""
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建节点
    hello_world_mqtt_bridge = HelloWorldMQTTBridge()
    
    try:
        # 运行节点
        rclpy.spin(hello_world_mqtt_bridge)
    except KeyboardInterrupt:
        hello_world_mqtt_bridge.get_logger().info('收到键盘中断信号')
    except Exception as e:
        hello_world_mqtt_bridge.get_logger().error(f'节点运行时发生错误: {str(e)}')
    finally:
        # 清理资源
        hello_world_mqtt_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
