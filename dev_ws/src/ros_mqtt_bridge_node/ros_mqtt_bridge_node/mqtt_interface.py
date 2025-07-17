#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
import threading
from datetime import datetime
from typing import Dict, Any, Callable, Optional, Union, List
import logging


class MQTTInterface:
    """通用MQTT接口类，提供MQTT连接、发布、订阅等基础功能"""
    
    def __init__(self, config: Dict[str, Any], logger: Optional[logging.Logger] = None):
        """
        初始化MQTT接口
        
        Args:
            config: MQTT配置字典，包含broker_host, broker_port, client_id等
            logger: 日志记录器
        """
        self.config = config
        self.logger = logger or logging.getLogger(__name__)
        self.connected = False
        self.client = None
        self.lock = threading.Lock()
        
        # 回调函数注册
        self.on_connect_callback = None
        self.on_disconnect_callback = None
        self.on_publish_callback = None
        self.on_message_callback = None
        
        self._initialize_client()
    
    def _initialize_client(self):
        """初始化MQTT客户端"""
        try:
            self.client = mqtt.Client(self.config.get('client_id', 'mqtt_client'))
            
            # 设置回调函数
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_publish = self._on_publish
            self.client.on_message = self._on_message
            
            # 设置认证信息
            username = self.config.get('username')
            password = self.config.get('password')
            if username and password:
                self.client.username_pw_set(username, password)
                
            self.logger.info("MQTT客户端初始化成功")
            
        except Exception as e:
            self.logger.error(f"MQTT客户端初始化失败: {str(e)}")
            raise
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        with self.lock:
            if rc == 0:
                self.connected = True
                self.logger.info("成功连接到MQTT服务器")
            else:
                self.connected = False
                self.logger.error(f"MQTT连接失败，返回码: {rc}")
        
        if self.on_connect_callback:
            self.on_connect_callback(rc)
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        with self.lock:
            self.connected = False
            
        if rc != 0:
            self.logger.error(f"意外断开连接，返回码: {rc}")
        else:
            self.logger.info("正常断开MQTT连接")
            
        if self.on_disconnect_callback:
            self.on_disconnect_callback(rc)
    
    def _on_publish(self, client, userdata, mid):
        """MQTT发布回调"""
        self.logger.debug(f"消息发布成功，消息ID: {mid}")
        
        if self.on_publish_callback:
            self.on_publish_callback(mid)
    
    def _on_message(self, client, userdata, msg):
        """MQTT消息接收回调"""
        self.logger.debug(f"收到消息 - 主题: {msg.topic}, 负载: {msg.payload}")
        
        if self.on_message_callback:
            self.on_message_callback(msg.topic, msg.payload)
    
    def connect(self) -> bool:
        """连接到MQTT服务器"""
        try:
            self.logger.info(f"正在连接到MQTT服务器 {self.config['broker_host']}:{self.config['broker_port']}...")
            
            self.client.connect(
                self.config['broker_host'],
                self.config.get('broker_port', 1883),
                self.config.get('keepalive', 60)
            )
            
            # 启动网络循环
            self.client.loop_start()
            return True
            
        except Exception as e:
            self.logger.error(f"连接MQTT服务器失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开MQTT连接"""
        try:
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
                self.logger.info("MQTT连接已断开")
        except Exception as e:
            self.logger.error(f"断开MQTT连接时发生错误: {str(e)}")
    
    def publish(self, topic: str, payload: Any, qos: int = 0, retain: bool = False) -> bool:
        """
        发布消息到MQTT
        
        Args:
            topic: MQTT主题
            payload: 消息负载
            qos: 服务质量等级
            retain: 是否保留消息
            
        Returns:
            bool: 发布是否成功
        """
        if not self.is_connected():
            self.logger.warning("MQTT未连接，无法发布消息")
            return False
        
        try:
            # 如果payload是字典或列表，转换为JSON字符串
            if isinstance(payload, (dict, list)):
                payload = json.dumps(payload, ensure_ascii=False)
            
            result = self.client.publish(topic, payload, qos, retain)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"消息已发布到主题: {topic}")
                return True
            else:
                self.logger.error(f"消息发布失败，错误码: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"发布消息时发生错误: {str(e)}")
            return False
    
    def subscribe(self, topic: str, qos: int = 0) -> bool:
        """
        订阅MQTT主题
        
        Args:
            topic: MQTT主题
            qos: 服务质量等级
            
        Returns:
            bool: 订阅是否成功
        """
        if not self.is_connected():
            self.logger.warning("MQTT未连接，无法订阅主题")
            return False
        
        try:
            result = self.client.subscribe(topic, qos)
            
            if result[0] == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"已订阅主题: {topic}")
                return True
            else:
                self.logger.error(f"订阅主题失败，错误码: {result[0]}")
                return False
                
        except Exception as e:
            self.logger.error(f"订阅主题时发生错误: {str(e)}")
            return False
    
    def unsubscribe(self, topic: str) -> bool:
        """
        取消订阅MQTT主题
        
        Args:
            topic: MQTT主题
            
        Returns:
            bool: 取消订阅是否成功
        """
        if not self.is_connected():
            self.logger.warning("MQTT未连接，无法取消订阅主题")
            return False
        
        try:
            result = self.client.unsubscribe(topic)
            
            if result[0] == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"已取消订阅主题: {topic}")
                return True
            else:
                self.logger.error(f"取消订阅主题失败，错误码: {result[0]}")
                return False
                
        except Exception as e:
            self.logger.error(f"取消订阅主题时发生错误: {str(e)}")
            return False
    
    def is_connected(self) -> bool:
        """检查MQTT连接状态"""
        with self.lock:
            return self.connected
    
    def set_on_connect_callback(self, callback: Callable[[int], None]):
        """设置连接回调函数"""
        self.on_connect_callback = callback
    
    def set_on_disconnect_callback(self, callback: Callable[[int], None]):
        """设置断开连接回调函数"""
        self.on_disconnect_callback = callback
    
    def set_on_publish_callback(self, callback: Callable[[int], None]):
        """设置发布回调函数"""
        self.on_publish_callback = callback
    
    def set_on_message_callback(self, callback: Callable[[str, bytes], None]):
        """设置消息接收回调函数"""
        self.on_message_callback = callback
    
    def get_config(self) -> Dict[str, Any]:
        """获取配置信息"""
        return self.config.copy()
    
    def update_config(self, new_config: Dict[str, Any]):
        """更新配置信息"""
        self.config.update(new_config)
        self.logger.info("MQTT配置已更新")


class MQTTMessageBuilder:
    """MQTT消息构建器，用于构建标准化的MQTT消息"""
    
    @staticmethod
    def create_ros_message(source_node: str, source_topic: str, data: Any, 
                          message_id: int = None, frame_id: str = None) -> Dict[str, Any]:
        """
        创建ROS消息格式的MQTT消息
        
        Args:
            source_node: 源节点名称
            source_topic: 源话题名称
            data: 消息数据
            message_id: 消息ID
            frame_id: 帧ID
            
        Returns:
            Dict: 格式化的消息字典
        """
        message = {
            'timestamp': datetime.now().isoformat(),
            'source_node': source_node,
            'source_topic': source_topic,
            'data': data
        }
        
        if message_id is not None:
            message['message_id'] = message_id
            
        if frame_id is not None:
            message['frame_id'] = frame_id
            
        return message
    
    @staticmethod
    def create_status_message(node_name: str, status: str, **kwargs) -> Dict[str, Any]:
        """
        创建状态消息
        
        Args:
            node_name: 节点名称
            status: 状态信息
            **kwargs: 额外的状态数据
            
        Returns:
            Dict: 状态消息字典
        """
        message = {
            'timestamp': datetime.now().isoformat(),
            'node_name': node_name,
            'status': status
        }
        
        message.update(kwargs)
        return message
