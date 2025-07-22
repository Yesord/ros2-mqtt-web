#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import time
from datetime import datetime
from typing import Dict, List, Any, Optional
from .mqtt_interface import MQTTInterface, MQTTMessageBuilder
from .config_loader import BridgeConfigLoader, get_default_config_path
from .topic_bridge import TopicBridge


class MultiBridgeManager(Node):
    """多话题MQTT桥接管理器"""
    
    def __init__(self, config_file_path: str = None):
        super().__init__('multi_bridge_manager')
        
        # 配置加载器
        self.config_loader = BridgeConfigLoader()
        self.config_file_path = config_file_path or get_default_config_path()
        
        # 桥接器管理
        self.bridges: Dict[str, TopicBridge] = {}
        
        # 统计信息
        self.start_time = datetime.now()
        self.total_published_messages = 0
        
        # 初始化
        if not self._load_and_initialize():
            self.get_logger().error('初始化失败，节点将退出')
            raise RuntimeError('初始化失败')
    
    def _load_and_initialize(self) -> bool:
        """加载配置并初始化"""
        try:
            # 加载配置文件
            self.get_logger().info(f'尝试加载配置文件: {self.config_file_path}')
            if not self.config_loader.load_config(self.config_file_path):
                # 如果指定路径失败，尝试其他路径
                self.get_logger().warning(f'指定路径加载失败，尝试其他路径...')
                fallback_path = get_default_config_path()
                self.get_logger().info(f'尝试备用路径: {fallback_path}')
                
                if fallback_path != self.config_file_path and self.config_loader.load_config(fallback_path):
                    self.config_file_path = fallback_path
                    self.get_logger().info(f'✓ 使用备用配置文件: {fallback_path}')
                else:
                    self.get_logger().error('所有配置文件路径都失败')
                    return False
            
            # 打印配置摘要
            self.config_loader.print_config_summary()
            
            # 声明ROS参数（可覆盖配置文件设置）
            self._declare_parameters()
            
            # 初始化MQTT接口
            if not self._initialize_mqtt():
                return False
            
            # 创建桥接器
            if not self._create_bridges():
                return False
            
            # 启动定时器
            self._start_timers()
            
            self.get_logger().info('=== 多话题MQTT桥接管理器已启动 ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'初始化过程中发生错误: {str(e)}')
            return False
    
    def _declare_parameters(self):
        """声明ROS参数"""
        # 从配置文件获取默认值
        mqtt_config = self.config_loader.get_mqtt_global_config()
        bridge_config = self.config_loader.get_bridge_global_config()
        
        # MQTT连接参数
        self.declare_parameter('mqtt_broker_host', mqtt_config.get('broker_host', 'localhost'))
        self.declare_parameter('mqtt_broker_port', mqtt_config.get('broker_port', 1883))
        self.declare_parameter('mqtt_topic_prefix', mqtt_config.get('topic_prefix', 'ros2'))
        self.declare_parameter('mqtt_client_id', f"{mqtt_config.get('client_id_prefix', 'ros_mqtt_bridge')}_{int(time.time())}")
        self.declare_parameter('mqtt_username', mqtt_config.get('username', ''))
        self.declare_parameter('mqtt_password', mqtt_config.get('password', ''))
        self.declare_parameter('mqtt_keepalive', mqtt_config.get('keepalive', 60))
        self.declare_parameter('mqtt_qos', mqtt_config.get('qos', 1))
        
        # 桥接控制参数
        self.declare_parameter('enable_statistics', bridge_config.get('enable_statistics', True))
        self.declare_parameter('enable_heartbeat', bridge_config.get('enable_heartbeat', True))
        self.declare_parameter('message_history_size', bridge_config.get('message_history_size', 100))
        self.declare_parameter('connection_check_interval', bridge_config.get('connection_check_interval', 10.0))
        self.declare_parameter('stats_publish_interval', bridge_config.get('stats_publish_interval', 30.0))
        self.declare_parameter('heartbeat_interval', bridge_config.get('heartbeat_interval', 60.0))
        
        # 配置文件路径参数
        self.declare_parameter('config_file_path', self.config_file_path)
    
    def _initialize_mqtt(self) -> bool:
        """初始化MQTT接口"""
        try:
            # 获取MQTT配置（参数优先于配置文件）
            mqtt_config = {
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
                mqtt_config['username'] = username
                mqtt_config['password'] = password
            
            # 创建MQTT接口
            self.mqtt_interface = MQTTInterface(mqtt_config, self.get_logger())
            
            # 设置回调函数
            self.mqtt_interface.set_on_connect_callback(self._on_mqtt_connect)
            self.mqtt_interface.set_on_disconnect_callback(self._on_mqtt_disconnect)
            self.mqtt_interface.set_on_publish_callback(self._on_mqtt_publish)
            
            # 连接MQTT服务器
            self.mqtt_connected = False
            self._connect_mqtt()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'初始化MQTT接口失败: {str(e)}')
            return False
    
    def _create_bridges(self) -> bool:
        """创建所有桥接器"""
        try:
            enabled_bridges = self.config_loader.get_enabled_bridge_configs()
            
            if not enabled_bridges:
                self.get_logger().warning('没有找到启用的桥接配置')
                return True
            
            for bridge_config in enabled_bridges:
                bridge_name = bridge_config.get('name')
                
                try:
                    bridge = TopicBridge(
                        bridge_config=bridge_config,
                        node=self,
                        mqtt_interface=self.mqtt_interface,
                        message_builder=MQTTMessageBuilder,
                        logger=self.get_logger()
                    )
                    
                    if bridge.start():
                        self.bridges[bridge_name] = bridge
                        self.get_logger().info(f'桥接器 {bridge_name} 创建成功')
                    else:
                        self.get_logger().error(f'桥接器 {bridge_name} 启动失败')
                        
                except Exception as e:
                    self.get_logger().error(f'创建桥接器 {bridge_name} 时发生错误: {str(e)}')
            
            active_bridges = len(self.bridges)
            total_bridges = len(enabled_bridges)
            
            self.get_logger().info(f'成功创建 {active_bridges}/{total_bridges} 个桥接器')
            return active_bridges > 0
            
        except Exception as e:
            self.get_logger().error(f'创建桥接器时发生错误: {str(e)}')
            return False
    
    def _start_timers(self):
        """启动定时器"""
        try:
            # 连接检查定时器
            check_interval = self.get_parameter('connection_check_interval').value
            self.connection_check_timer = self.create_timer(check_interval, self._check_connections)
            
            # 统计信息发布定时器
            if self.get_parameter('enable_statistics').value:
                stats_interval = self.get_parameter('stats_publish_interval').value
                self.stats_timer = self.create_timer(stats_interval, self._publish_statistics)
            
            # 心跳定时器
            if self.get_parameter('enable_heartbeat').value:
                heartbeat_interval = self.get_parameter('heartbeat_interval').value
                self.heartbeat_timer = self.create_timer(heartbeat_interval, self._publish_heartbeat)
            
            self.get_logger().debug('定时器启动完成')
            
        except Exception as e:
            self.get_logger().error(f'启动定时器时发生错误: {str(e)}')
    
    def _connect_mqtt(self):
        """连接MQTT服务器"""
        self.get_logger().info('正在连接到MQTT服务器...')
        success = self.mqtt_interface.connect()
        if not success:
            self.get_logger().error('初始MQTT连接失败，将在后台重试')
    
    def _on_mqtt_connect(self, rc: int):
        """MQTT连接回调"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info('✓ MQTT连接成功')
            self._publish_manager_status('connected')
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
        try:
            config = self.mqtt_interface.get_config()
            control_topic = f'{config.get("topic_prefix", "ros2")}/multi_bridge/control'
            
            self.mqtt_interface.set_on_message_callback(self._on_control_message)
            success = self.mqtt_interface.subscribe(control_topic, qos=config.get('qos', 1))
            
            if success:
                self.get_logger().info(f'已订阅控制话题: {control_topic}')
                
        except Exception as e:
            self.get_logger().error(f'订阅控制话题时发生错误: {str(e)}')
    
    def _on_control_message(self, topic: str, payload: bytes):
        """处理控制消息"""
        try:
            message = json.loads(payload.decode('utf-8'))
            command = message.get('command')
            
            if command == 'get_stats':
                self._publish_statistics()
            elif command == 'get_bridge_list':
                self._publish_bridge_list()
            elif command == 'get_bridge_stats':
                bridge_name = message.get('bridge_name')
                self._publish_bridge_statistics(bridge_name)
            elif command == 'reload_config':
                self._reload_config()
            elif command == 'reset_stats':
                self._reset_statistics()
            else:
                self.get_logger().warning(f'未知控制命令: {command}')
                
        except Exception as e:
            self.get_logger().error(f'处理控制消息时发生错误: {str(e)}')
    
    def _check_connections(self):
        """检查连接状态"""
        try:
            # 检查MQTT连接
            if not self.mqtt_interface.is_connected():
                self.get_logger().warning('MQTT连接丢失，尝试重新连接...')
                self._connect_mqtt()
            
            # 检查桥接器状态
            inactive_bridges = []
            for name, bridge in self.bridges.items():
                if not bridge.is_active(timeout_seconds=15.0):
                    inactive_bridges.append(name)
            
            if inactive_bridges:
                self.get_logger().warning(f'检测到不活跃的桥接器: {", ".join(inactive_bridges)}')
                
        except Exception as e:
            self.get_logger().error(f'检查连接状态时发生错误: {str(e)}')
    
    def _publish_statistics(self):
        """发布统计信息"""
        if not self.mqtt_connected:
            return
        
        try:
            # 收集所有桥接器的统计信息
            bridge_stats = {}
            total_messages = 0
            
            for name, bridge in self.bridges.items():
                stats = bridge.get_statistics()
                bridge_stats[name] = stats
                total_messages += stats.get('message_count', 0)
            
            uptime = (datetime.now() - self.start_time).total_seconds()
            
            stats_message = MQTTMessageBuilder.create_status_message(
                node_name='multi_bridge_manager',
                status='running',
                uptime_seconds=uptime,
                total_bridges=len(self.bridges),
                total_messages=total_messages,
                mqtt_connected=self.mqtt_connected,
                bridge_statistics=bridge_stats,
                config_file=self.config_file_path,
                config_version=self.config_loader.get_config_version()
            )
            
            config = self.mqtt_interface.get_config()
            stats_topic = f'{config.get("topic_prefix", "ros2")}/multi_bridge/statistics'
            
            success = self.mqtt_interface.publish(
                stats_topic,
                stats_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().debug('📊 统计信息已发布')
                
        except Exception as e:
            self.get_logger().error(f'发布统计信息时发生错误: {str(e)}')
    
    def _publish_heartbeat(self):
        """发布心跳信息"""
        if not self.mqtt_connected:
            return
        
        try:
            active_bridges = [name for name, bridge in self.bridges.items() if bridge.is_active()]
            
            heartbeat_message = MQTTMessageBuilder.create_status_message(
                node_name='multi_bridge_manager',
                status='alive',
                uptime_seconds=(datetime.now() - self.start_time).total_seconds(),
                total_bridges=len(self.bridges),
                active_bridges=len(active_bridges),
                bridge_names=list(self.bridges.keys()),
                active_bridge_names=active_bridges
            )
            
            config = self.mqtt_interface.get_config()
            heartbeat_topic = f'{config.get("topic_prefix", "ros2")}/multi_bridge/heartbeat'
            
            success = self.mqtt_interface.publish(
                heartbeat_topic,
                heartbeat_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().debug('💓 心跳信息已发布')
                
        except Exception as e:
            self.get_logger().error(f'发布心跳信息时发生错误: {str(e)}')
    
    def _publish_bridge_list(self):
        """发布桥接器列表"""
        if not self.mqtt_connected:
            return
        
        try:
            bridge_list = []
            for name, bridge in self.bridges.items():
                bridge_info = {
                    'name': name,
                    'enabled': bridge.enabled,
                    'ros_topic': bridge.ros_config.get('topic'),
                    'message_type': bridge.ros_config.get('message_type'),
                    'mqtt_topic': bridge._build_mqtt_topic(),
                    'message_count': bridge.message_count,
                    'is_active': bridge.is_active()
                }
                bridge_list.append(bridge_info)
            
            list_message = MQTTMessageBuilder.create_status_message(
                node_name='multi_bridge_manager',
                status='bridge_list',
                total_bridges=len(bridge_list),
                bridges=bridge_list
            )
            
            config = self.mqtt_interface.get_config()
            list_topic = f'{config.get("topic_prefix", "ros2")}/multi_bridge/bridge_list'
            
            success = self.mqtt_interface.publish(
                list_topic,
                list_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().info('📋 桥接器列表已发布')
                
        except Exception as e:
            self.get_logger().error(f'发布桥接器列表时发生错误: {str(e)}')
    
    def _publish_bridge_statistics(self, bridge_name: str = None):
        """发布特定桥接器的统计信息"""
        if not self.mqtt_connected:
            return
        
        try:
            if bridge_name and bridge_name in self.bridges:
                bridges_to_publish = {bridge_name: self.bridges[bridge_name]}
            else:
                bridges_to_publish = self.bridges
            
            for name, bridge in bridges_to_publish.items():
                stats = bridge.get_statistics()
                
                config = self.mqtt_interface.get_config()
                stats_topic = f'{config.get("topic_prefix", "ros2")}/multi_bridge/bridges/{name}/statistics'
                
                success = self.mqtt_interface.publish(
                    stats_topic,
                    stats,
                    qos=config.get('qos', 1)
                )
                
                if success:
                    self.get_logger().debug(f'📊 桥接器 {name} 统计信息已发布')
                    
        except Exception as e:
            self.get_logger().error(f'发布桥接器统计信息时发生错误: {str(e)}')
    
    def _reload_config(self):
        """重新加载配置"""
        try:
            self.get_logger().info('开始重新加载配置...')
            
            # 停止所有现有桥接器
            for bridge in self.bridges.values():
                bridge.stop()
            self.bridges.clear()
            
            # 重新加载配置文件
            if self.config_loader.load_config(self.config_file_path):
                self.config_loader.print_config_summary()
                
                # 重新创建桥接器
                if self._create_bridges():
                    self.get_logger().info('✓ 配置重新加载成功')
                    self._publish_manager_status('config_reloaded')
                else:
                    self.get_logger().error('✗ 重新创建桥接器失败')
            else:
                self.get_logger().error('✗ 重新加载配置文件失败')
                
        except Exception as e:
            self.get_logger().error(f'重新加载配置时发生错误: {str(e)}')
    
    def _reset_statistics(self):
        """重置统计信息"""
        try:
            self.start_time = datetime.now()
            
            for bridge in self.bridges.values():
                bridge.message_count = 0
                bridge.start_time = datetime.now()
                bridge.last_message_time = None
            
            self.get_logger().info('📊 统计信息已重置')
            self._publish_manager_status('statistics_reset')
            
        except Exception as e:
            self.get_logger().error(f'重置统计信息时发生错误: {str(e)}')
    
    def _publish_manager_status(self, status: str):
        """发布管理器状态"""
        if not self.mqtt_connected:
            return
        
        try:
            status_message = MQTTMessageBuilder.create_status_message(
                node_name='multi_bridge_manager',
                status=status,
                mqtt_connected=self.mqtt_connected,
                total_bridges=len(self.bridges),
                uptime_seconds=(datetime.now() - self.start_time).total_seconds(),
                config_file=self.config_file_path
            )
            
            config = self.mqtt_interface.get_config()
            status_topic = f'{config.get("topic_prefix", "ros2")}/multi_bridge/manager_status'
            
            success = self.mqtt_interface.publish(
                status_topic,
                status_message,
                qos=config.get('qos', 1)
            )
            
            if success:
                self.get_logger().debug(f'📡 管理器状态已发布: {status}')
                
        except Exception as e:
            self.get_logger().error(f'发布管理器状态时发生错误: {str(e)}')
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info('正在关闭多话题MQTT桥接管理器...')
        
        # 发布关闭状态
        self._publish_manager_status('shutdown')
        
        # 给一点时间让最后的消息发送完成
        time.sleep(1.0)
        
        # 停止所有桥接器
        for bridge in self.bridges.values():
            bridge.stop()
        self.bridges.clear()
        
        # 断开MQTT连接
        if hasattr(self, 'mqtt_interface'):
            self.mqtt_interface.disconnect()
        
        # 调用父类的销毁方法
        super().destroy_node()
        
        self.get_logger().info('✓ 多话题MQTT桥接管理器已关闭')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 从命令行参数获取配置文件路径
    import sys
    config_file_path = None
    if len(sys.argv) > 1:
        config_file_path = sys.argv[1]
    
    try:
        # 创建节点
        manager = MultiBridgeManager(config_file_path)
        
        # 运行节点
        rclpy.spin(manager)
        
    except KeyboardInterrupt:
        manager.get_logger().info('收到键盘中断信号')
    except Exception as e:
        print(f'节点运行时发生错误: {str(e)}')
    finally:
        # 清理资源
        if 'manager' in locals():
            manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
