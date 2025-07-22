#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
import importlib
import json
from typing import Dict, Any, Optional, Callable
from datetime import datetime


class TopicBridge:
    """通用话题桥接器"""
    
    def __init__(self, 
                 bridge_config: Dict[str, Any], 
                 node: Node,
                 mqtt_interface,
                 message_builder,
                 logger):
        self.config = bridge_config
        self.node = node
        self.mqtt_interface = mqtt_interface
        self.message_builder = message_builder
        self.logger = logger
        
        # 桥接状态
        self.name = bridge_config.get('name', 'unknown_bridge')
        self.enabled = bridge_config.get('enabled', True)
        self.subscription: Optional[Subscription] = None
        self.message_count = 0
        self.last_message_time = None
        self.start_time = datetime.now()
        
        # 解析配置
        self.ros_config = bridge_config.get('ros_config', {})
        self.mqtt_config = bridge_config.get('mqtt_config', {})
        self.metadata = bridge_config.get('metadata', {})
        
        # 缓存解析的消息类型
        self._message_class = None
        
    def start(self) -> bool:
        """启动桥接器"""
        if not self.enabled:
            self.logger.info(f'桥接器 {self.name} 已禁用，跳过启动')
            return True
            
        try:
            # 动态导入消息类型
            message_class = self._get_message_class()
            if not message_class:
                return False
            
            # 创建订阅者
            ros_topic = self.ros_config.get('topic')
            queue_size = self.ros_config.get('queue_size', 10)
            
            self.subscription = self.node.create_subscription(
                message_class,
                ros_topic,
                self._message_callback,
                queue_size
            )
            
            self.logger.info(f'✓ 桥接器 {self.name} 已启动')
            self.logger.info(f'  ROS话题: {ros_topic}')
            self.logger.info(f'  消息类型: {self.ros_config.get("message_type")}')
            self.logger.info(f'  队列大小: {queue_size}')
            return True
            
        except Exception as e:
            self.logger.error(f'启动桥接器 {self.name} 失败: {str(e)}')
            return False
    
    def stop(self):
        """停止桥接器"""
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
            self.logger.info(f'✓ 桥接器 {self.name} 已停止')
    
    def _get_message_class(self):
        """动态获取消息类型"""
        if self._message_class:
            return self._message_class
            
        try:
            message_type = self.ros_config.get('message_type')
            if not message_type:
                self.logger.error(f'桥接器 {self.name} 未指定消息类型')
                return None
            
            # 解析消息类型 例如: "std_msgs/String" -> ("std_msgs.msg", "String")
            parts = message_type.split('/')
            if len(parts) != 2:
                self.logger.error(f'无效的消息类型格式: {message_type}')
                return None
            
            package_name, class_name = parts
            module_name = f'{package_name}.msg'
            
            # 动态导入模块
            module = importlib.import_module(module_name)
            message_class = getattr(module, class_name)
            
            self._message_class = message_class
            self.logger.debug(f'成功加载消息类型: {message_type}')
            return message_class
            
        except ImportError as e:
            self.logger.error(f'无法导入消息类型 {message_type}: {str(e)}')
            return None
        except AttributeError as e:
            self.logger.error(f'消息类型 {message_type} 中找不到类 {class_name}: {str(e)}')
            return None
        except Exception as e:
            self.logger.error(f'加载消息类型时发生错误: {str(e)}')
            return None
    
    def _message_callback(self, msg):
        """ROS消息回调函数"""
        try:
            # 更新统计信息
            self.message_count += 1
            self.last_message_time = datetime.now()
            
            # 提取数据
            data = self._extract_message_data(msg)
            if data is None:
                self.logger.warning(f'桥接器 {self.name} 无法提取消息数据')
                return
            
            # 构建MQTT消息
            mqtt_message = self._build_mqtt_message(data)
            
            # 构建MQTT主题
            mqtt_topic = self._build_mqtt_topic()
            
            # 发布到MQTT
            qos = self.mqtt_config.get('qos', 1)
            retain = self.mqtt_config.get('retain', False)
            
            success = self.mqtt_interface.publish(
                mqtt_topic,
                mqtt_message,
                qos=qos,
                retain=retain
            )
            
            if success:
                self.logger.debug(f'✓ 桥接器 {self.name} 消息已转发')
                self.logger.debug(f'  主题: {mqtt_topic}')
                self.logger.debug(f'  数据: {data}')
            else:
                self.logger.error(f'✗ 桥接器 {self.name} MQTT发布失败')
                
        except Exception as e:
            self.logger.error(f'桥接器 {self.name} 处理消息时发生错误: {str(e)}')
    
    def _extract_message_data(self, msg) -> Any:
        """提取消息数据"""
        try:
            data_field = self.ros_config.get('data_field', 'data')
            
            # 首先检查多字段提取（优先级更高）
            if ',' in data_field:
                # 多字段提取，如 "linear.x,angular.z"
                fields = [field.strip() for field in data_field.split(',')]
                result = {}
                for field in fields:
                    if '.' in field:
                        # 嵌套访问
                        obj = msg
                        for field_part in field.split('.'):
                            if hasattr(obj, field_part):
                                obj = getattr(obj, field_part)
                            else:
                                self.logger.error(f'消息中找不到字段: {field_part}')
                                return None
                        result[field] = obj
                    else:
                        # 简单字段
                        if hasattr(msg, field):
                            result[field] = getattr(msg, field)
                        else:
                            self.logger.error(f'消息中找不到字段: {field}')
                            return None
                return result
            elif '.' in data_field:
                # 单一嵌套字段访问，如 "pose.position.x"
                obj = msg
                for field_part in data_field.split('.'):
                    if hasattr(obj, field_part):
                        obj = getattr(obj, field_part)
                    else:
                        self.logger.error(f'消息中找不到字段: {field_part}')
                        return None
                return obj
            else:
                # 简单字段访问
                if hasattr(msg, data_field):
                    return getattr(msg, data_field)
                else:
                    self.logger.error(f'消息中找不到字段: {data_field}')
                    return None
                    
        except Exception as e:
            self.logger.error(f'提取消息数据时发生错误: {str(e)}')
            return None
    
    def _process_field_data(self, data, field_name: str) -> Any:
        """处理字段数据，对二进制数据进行Base64编码"""
        try:
            # 获取消息类型信息
            message_type = self.ros_config.get('message_type', '')
            
            # 检查是否是图像相关的数据字段
            is_image_data = (
                field_name == 'data' and 
                message_type in ['sensor_msgs/Image', 'sensor_msgs/CompressedImage']
            )
            
            # 如果是图像数据，进行特殊处理
            if is_image_data:
                self.logger.debug(f'检测到图像数据字段: {field_name}, 消息类型: {message_type}')
                
                # 处理不同类型的数据
                if isinstance(data, (list, tuple)):
                    # ROS消息中的数组通常是列表或元组
                    if len(data) > 0 and all(isinstance(x, int) and 0 <= x <= 255 for x in data[:100]):
                        # 确认是uint8数组
                        byte_data = bytes(data)
                        encoded_data = base64.b64encode(byte_data).decode('utf-8')
                        
                        self.logger.info(f'图像数据编码完成 - 字段: {field_name}, 原始大小: {len(data)} 字节, 编码后大小: {len(encoded_data)} 字符')
                        
                        return {
                            'encoding': 'base64',
                            'data_type': 'uint8_array',
                            'original_size': len(data),
                            'data': encoded_data,
                            'timestamp': datetime.now().isoformat()
                        }
                        
                elif isinstance(data, np.ndarray):
                    # numpy数组
                    byte_data = data.tobytes()
                    encoded_data = base64.b64encode(byte_data).decode('utf-8')
                    
                    self.logger.info(f'NumPy图像数据编码完成 - 字段: {field_name}, 形状: {data.shape}, 编码后大小: {len(encoded_data)} 字符')
                    
                    return {
                        'encoding': 'base64',
                        'data_type': 'numpy_array',
                        'shape': list(data.shape),
                        'dtype': str(data.dtype),
                        'data': encoded_data,
                        'timestamp': datetime.now().isoformat()
                    }
                    
                elif isinstance(data, bytes):
                    # 已经是字节数据
                    encoded_data = base64.b64encode(data).decode('utf-8')
                    
                    self.logger.info(f'字节图像数据编码完成 - 字段: {field_name}, 大小: {len(data)} 字节')
                    
                    return {
                        'encoding': 'base64',
                        'data_type': 'bytes',
                        'original_size': len(data),
                        'data': encoded_data,
                        'timestamp': datetime.now().isoformat()
                    }
                else:
                    self.logger.warning(f'未知的图像数据类型: {type(data)} for field {field_name}')
                    # 尝试转换为字符串
                    return str(data)
            else:
                # 非图像数据，直接返回
                return data
                
        except Exception as e:
            self.logger.error(f'处理字段数据时发生错误: {str(e)}')
            # 出错时返回原始数据的字符串表示
            return str(data)
    
    def _build_mqtt_message(self, data) -> Dict[str, Any]:
        """构建MQTT消息"""
        try:
            # 获取元数据
            source_node = self.metadata.get('source_node', 'unknown_node')
            frame_id = self.metadata.get('frame_id', 'unknown_frame')
            
            # 使用消息构建器
            mqtt_message = self.message_builder.create_ros_message(
                source_node=source_node,
                source_topic=self.ros_config.get('topic'),
                data=data,
                message_id=self.message_count,
                frame_id=frame_id
            )
            
            # 添加桥接器特定的元数据
            mqtt_message.update({
                'bridge_name': self.name,
                'bridge_message_count': self.message_count,
                'bridge_uptime_seconds': (datetime.now() - self.start_time).total_seconds(),
                'bridge_config': {
                    'ros_topic': self.ros_config.get('topic'),
                    'message_type': self.ros_config.get('message_type'),
                    'data_field': self.ros_config.get('data_field')
                }
            })
            
            return mqtt_message
            
        except Exception as e:
            self.logger.error(f'构建MQTT消息时发生错误: {str(e)}')
            return {}
    
    def _build_mqtt_topic(self) -> str:
        """构建MQTT主题路径"""
        try:
            # 从全局配置获取前缀
            topic_prefix = self.mqtt_interface.get_config().get('topic_prefix', 'ros2')
            topic_name = self.mqtt_config.get('topic_name', 'unknown')
            topic_suffix = self.mqtt_config.get('topic_suffix', 'data')
            
            return f'{topic_prefix}/{topic_name}/{topic_suffix}'
            
        except Exception as e:
            self.logger.error(f'构建MQTT主题时发生错误: {str(e)}')
            return 'ros2/unknown/data'
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取桥接器统计信息"""
        uptime = (datetime.now() - self.start_time).total_seconds()
        
        return {
            'bridge_name': self.name,
            'enabled': self.enabled,
            'message_count': self.message_count,
            'uptime_seconds': uptime,
            'last_message_time': self.last_message_time.isoformat() if self.last_message_time else None,
            'message_rate': self.message_count / uptime if uptime > 0 else 0.0,
            'ros_config': self.ros_config,
            'mqtt_config': self.mqtt_config,
            'metadata': self.metadata
        }
    
    def is_active(self, timeout_seconds: float = 10.0) -> bool:
        """检查桥接器是否活跃"""
        if not self.last_message_time:
            return False
        
        elapsed = (datetime.now() - self.last_message_time).total_seconds()
        return elapsed < timeout_seconds
