#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Dict, Any, Callable, Optional
import threading
from datetime import datetime


class ROSNodeDataReader:
    """ROS节点数据读取器接口，提供从ROS话题读取数据的通用功能"""
    
    def __init__(self, node: Node, topic_config: Dict[str, Any]):
        """
        初始化ROS节点数据读取器
        
        Args:
            node: ROS节点实例
            topic_config: 话题配置字典，包含话题名称、消息类型等信息
        """
        self.node = node
        self.topic_config = topic_config
        self.subscribers = {}
        self.message_callbacks = {}
        self.message_count = {}
        self.last_message_time = {}
        self.lock = threading.Lock()
        
        # 初始化订阅者
        self._initialize_subscribers()
    
    def _initialize_subscribers(self):
        """初始化ROS话题订阅者"""
        for topic_name, config in self.topic_config.items():
            try:
                msg_type = config.get('msg_type', String)
                queue_size = config.get('queue_size', 10)
                
                # 创建订阅者
                subscriber = self.node.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, topic=topic_name: self._message_callback(topic, msg),
                    queue_size
                )
                
                self.subscribers[topic_name] = subscriber
                self.message_count[topic_name] = 0
                self.last_message_time[topic_name] = None
                
                self.node.get_logger().info(f"已订阅话题: {topic_name}")
                
            except Exception as e:
                self.node.get_logger().error(f"创建订阅者失败 - 话题: {topic_name}, 错误: {str(e)}")
    
    def _message_callback(self, topic_name: str, msg: Any):
        """内部消息回调函数"""
        with self.lock:
            self.message_count[topic_name] += 1
            self.last_message_time[topic_name] = datetime.now()
        
        # 调用用户注册的回调函数
        if topic_name in self.message_callbacks:
            try:
                callback = self.message_callbacks[topic_name]
                callback(topic_name, msg, self.message_count[topic_name])
            except Exception as e:
                self.node.get_logger().error(f"消息回调函数执行失败 - 话题: {topic_name}, 错误: {str(e)}")
    
    def register_message_callback(self, topic_name: str, callback: Callable[[str, Any, int], None]):
        """
        注册消息回调函数
        
        Args:
            topic_name: 话题名称
            callback: 回调函数，参数为(topic_name, message, message_count)
        """
        if topic_name not in self.subscribers:
            self.node.get_logger().warning(f"话题 {topic_name} 未被订阅，无法注册回调函数")
            return
        
        self.message_callbacks[topic_name] = callback
        self.node.get_logger().info(f"已为话题 {topic_name} 注册回调函数")
    
    def unregister_message_callback(self, topic_name: str):
        """
        取消注册消息回调函数
        
        Args:
            topic_name: 话题名称
        """
        if topic_name in self.message_callbacks:
            del self.message_callbacks[topic_name]
            self.node.get_logger().info(f"已取消话题 {topic_name} 的回调函数")
    
    def get_message_count(self, topic_name: str) -> int:
        """获取话题的消息计数"""
        with self.lock:
            return self.message_count.get(topic_name, 0)
    
    def get_last_message_time(self, topic_name: str) -> Optional[datetime]:
        """获取话题的最后消息时间"""
        with self.lock:
            return self.last_message_time.get(topic_name)
    
    def get_statistics(self, topic_name: str = None) -> Dict[str, Any]:
        """
        获取统计信息
        
        Args:
            topic_name: 话题名称，如果为None则返回所有话题的统计信息
            
        Returns:
            Dict: 统计信息字典
        """
        with self.lock:
            if topic_name:
                if topic_name in self.message_count:
                    return {
                        'topic_name': topic_name,
                        'message_count': self.message_count[topic_name],
                        'last_message_time': self.last_message_time[topic_name].isoformat() if self.last_message_time[topic_name] else None
                    }
                else:
                    return {}
            else:
                stats = {}
                for topic in self.message_count:
                    stats[topic] = {
                        'message_count': self.message_count[topic],
                        'last_message_time': self.last_message_time[topic].isoformat() if self.last_message_time[topic] else None
                    }
                return stats
    
    def is_topic_active(self, topic_name: str, timeout_seconds: float = 5.0) -> bool:
        """
        检查话题是否活跃
        
        Args:
            topic_name: 话题名称
            timeout_seconds: 超时时间（秒）
            
        Returns:
            bool: 话题是否活跃
        """
        with self.lock:
            last_time = self.last_message_time.get(topic_name)
            if last_time is None:
                return False
            
            time_diff = (datetime.now() - last_time).total_seconds()
            return time_diff <= timeout_seconds
    
    def get_subscribed_topics(self) -> list:
        """获取已订阅的话题列表"""
        return list(self.subscribers.keys())
    
    def destroy(self):
        """销毁资源"""
        self.node.get_logger().info("正在销毁ROS节点数据读取器...")
        
        # 清理订阅者
        for topic_name, subscriber in self.subscribers.items():
            try:
                self.node.destroy_subscription(subscriber)
                self.node.get_logger().info(f"已销毁话题 {topic_name} 的订阅者")
            except Exception as e:
                self.node.get_logger().error(f"销毁话题 {topic_name} 的订阅者失败: {str(e)}")
        
        # 清理数据
        self.subscribers.clear()
        self.message_callbacks.clear()
        self.message_count.clear()
        self.last_message_time.clear()


class HelloWorldNodeDataReader(ROSNodeDataReader):
    """Hello World节点专用数据读取器"""
    
    def __init__(self, node: Node):
        """
        初始化Hello World节点数据读取器
        
        Args:
            node: ROS节点实例
        """
        # Hello World节点的话题配置
        topic_config = {
            'hello_world': {
                'msg_type': String,
                'queue_size': 10,
                'description': 'Hello World消息话题'
            }
        }
        
        super().__init__(node, topic_config)
        
        # Hello World特定的数据处理
        self.hello_world_data = {
            'current_message': None,
            'message_history': [],
            'max_history_size': 100
        }
    
    def register_hello_world_callback(self, callback: Callable[[str, str, int], None]):
        """
        注册Hello World消息回调函数
        
        Args:
            callback: 回调函数，参数为(topic_name, message_data, message_count)
        """
        def hello_world_wrapper(topic_name: str, msg: String, count: int):
            # 更新Hello World数据
            self.hello_world_data['current_message'] = msg.data
            self.hello_world_data['message_history'].append({
                'timestamp': datetime.now().isoformat(),
                'data': msg.data,
                'count': count
            })
            
            # 保持历史记录大小限制
            if len(self.hello_world_data['message_history']) > self.hello_world_data['max_history_size']:
                self.hello_world_data['message_history'].pop(0)
            
            # 调用用户回调函数
            callback(topic_name, msg.data, count)
        
        self.register_message_callback('hello_world', hello_world_wrapper)
    
    def get_current_hello_world_message(self) -> Optional[str]:
        """获取当前Hello World消息"""
        return self.hello_world_data['current_message']
    
    def get_hello_world_history(self, limit: int = None) -> list:
        """
        获取Hello World消息历史
        
        Args:
            limit: 返回消息数量限制
            
        Returns:
            list: 消息历史列表
        """
        history = self.hello_world_data['message_history']
        if limit:
            return history[-limit:]
        return history.copy()
    
    def get_hello_world_statistics(self) -> Dict[str, Any]:
        """获取Hello World节点的详细统计信息"""
        base_stats = self.get_statistics('hello_world')
        
        if base_stats:
            base_stats.update({
                'current_message': self.hello_world_data['current_message'],
                'history_size': len(self.hello_world_data['message_history']),
                'is_active': self.is_topic_active('hello_world')
            })
        
        return base_stats
