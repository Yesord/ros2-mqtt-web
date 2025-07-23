#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 图像到MQTT桥接节点示例
支持摄像头图像、深度图、点云等多媒体数据的MQTT转发
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import base64
import json
import paho.mqtt.client as mqtt
from cv_bridge import CvBridge
import threading
import time
from datetime import datetime

class MediaMQTTBridge(Node):
    def __init__(self):
        super().__init__('media_mqtt_bridge')
        
        # 参数声明
        self.declare_parameter('mqtt_broker_host', 'localhost')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_topic_prefix', 'ros2/camera')
        self.declare_parameter('image_quality', 80)
        self.declare_parameter('max_fps', 10.0)
        
        # 获取参数
        self.mqtt_host = self.get_parameter('mqtt_broker_host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_broker_port').get_parameter_value().integer_value
        self.topic_prefix = self.get_parameter('mqtt_topic_prefix').get_parameter_value().string_value
        self.image_quality = self.get_parameter('image_quality').get_parameter_value().integer_value
        self.max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 初始化MQTT客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # 帧率控制
        self.last_publish_time = {}
        self.min_interval = 1.0 / self.max_fps
        
        # 统计信息
        self.stats = {
            'images_processed': 0,
            'bytes_sent': 0,
            'start_time': time.time()
        }
        
        self.get_logger().info(f"媒体MQTT桥接节点已启动")
        self.get_logger().info(f"MQTT服务器: {self.mqtt_host}:{self.mqtt_port}")
        self.get_logger().info(f"话题前缀: {self.topic_prefix}")
        
        # 连接MQTT
        self.connect_mqtt()
        
        # 创建订阅者
        self.create_subscribers()
        
        # 创建定时器发布统计信息
        self.create_timer(5.0, self.publish_statistics)
        
        # 创建测试图像发布器（用于演示）
        self.create_test_publishers()

    def connect_mqtt(self):
        """连接到MQTT服务器"""
        try:
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("正在连接MQTT服务器...")
        except Exception as e:
            self.get_logger().error(f"MQTT连接失败: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.get_logger().info("MQTT连接成功")
        else:
            self.get_logger().error(f"MQTT连接失败，返回码: {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        self.get_logger().warning(f"MQTT连接断开，返回码: {rc}")

    def create_subscribers(self):
        """创建ROS2订阅者"""
        
        # 图像订阅者
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # 压缩图像订阅者
        self.compressed_image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.compressed_image_callback,
            10
        )
        
        # 深度图订阅者
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # 点云订阅者
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            10
        )

    def create_test_publishers(self):
        """创建测试数据发布器"""
        
        # 创建定时器发布测试图像
        self.test_image_timer = self.create_timer(1.0, self.publish_test_image)
        self.test_depth_timer = self.create_timer(2.0, self.publish_test_depth)
        self.test_pointcloud_timer = self.create_timer(3.0, self.publish_test_pointcloud)

    def image_callback(self, msg):
        """处理图像消息"""
        if not self.should_publish('/camera/image_raw'):
            return
            
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 编码为JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
            
            if result:
                # Base64编码
                img_base64 = base64.b64encode(encimg).decode('utf-8')
                
                # 创建MQTT消息
                mqtt_data = {
                    'type': 'image',
                    'encoding': 'bgr8',
                    'width': msg.width,
                    'height': msg.height,
                    'image_data': img_base64,
                    'timestamp': time.time(),
                    'frame_id': msg.header.frame_id,
                    'seq': msg.header.stamp.sec
                }
                
                # 发布到MQTT
                topic = f"{self.topic_prefix}/image"
                self.publish_to_mqtt(topic, mqtt_data)
                
                self.stats['images_processed'] += 1
                self.stats['bytes_sent'] += len(img_base64)
                
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {e}")

    def compressed_image_callback(self, msg):
        """处理压缩图像消息"""
        if not self.should_publish('/camera/image_raw/compressed'):
            return
            
        try:
            # 直接使用压缩数据
            img_base64 = base64.b64encode(msg.data).decode('utf-8')
            
            mqtt_data = {
                'type': 'compressed_image',
                'format': msg.format,
                'image_data': img_base64,
                'timestamp': time.time(),
                'frame_id': msg.header.frame_id,
                'size': len(msg.data)
            }
            
            topic = f"{self.topic_prefix}/compressed"
            self.publish_to_mqtt(topic, mqtt_data)
            
        except Exception as e:
            self.get_logger().error(f"压缩图像处理错误: {e}")

    def depth_callback(self, msg):
        """处理深度图消息"""
        if not self.should_publish('/camera/depth/image_raw'):
            return
            
        try:
            # 转换深度图
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 归一化深度值到0-255范围
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            
            # 编码为PNG（保持深度信息）
            result, encimg = cv2.imencode('.png', depth_normalized)
            
            if result:
                img_base64 = base64.b64encode(encimg).decode('utf-8')
                
                mqtt_data = {
                    'type': 'depth',
                    'encoding': msg.encoding,
                    'width': msg.width,
                    'height': msg.height,
                    'image_data': img_base64,
                    'timestamp': time.time(),
                    'frame_id': msg.header.frame_id,
                    'min_depth': float(np.min(depth_image)),
                    'max_depth': float(np.max(depth_image))
                }
                
                topic = f"{self.topic_prefix}/depth"
                self.publish_to_mqtt(topic, mqtt_data)
                
        except Exception as e:
            self.get_logger().error(f"深度图处理错误: {e}")

    def pointcloud_callback(self, msg):
        """处理点云消息"""
        if not self.should_publish('/lidar/points'):
            return
            
        try:
            # 简化点云数据（仅提取前1000个点）
            import struct
            
            points = []
            point_step = msg.point_step
            row_step = msg.row_step
            
            # 解析点云数据（假设为XYZRGB格式）
            for i in range(0, min(len(msg.data), 1000 * point_step), point_step):
                x = struct.unpack('f', msg.data[i:i+4])[0]
                y = struct.unpack('f', msg.data[i+4:i+8])[0]
                z = struct.unpack('f', msg.data[i+8:i+12])[0]
                points.extend([x, y, z])
            
            mqtt_data = {
                'type': 'pointcloud',
                'width': msg.width,
                'height': msg.height,
                'points': points,
                'point_count': len(points) // 3,
                'timestamp': time.time(),
                'frame_id': msg.header.frame_id
            }
            
            topic = f"{self.topic_prefix}/pointcloud"
            self.publish_to_mqtt(topic, mqtt_data)
            
        except Exception as e:
            self.get_logger().error(f"点云处理错误: {e}")

    def publish_test_image(self):
        """发布测试图像"""
        try:
            # 创建测试图像
            height, width = 480, 640
            
            # 创建彩色渐变图像
            img = np.zeros((height, width, 3), dtype=np.uint8)
            
            # 时间相关的颜色变化
            t = time.time()
            for y in range(height):
                for x in range(width):
                    r = int(128 + 127 * np.sin(t + x/100.0))
                    g = int(128 + 127 * np.sin(t + y/100.0))
                    b = int(128 + 127 * np.sin(t + (x+y)/200.0))
                    img[y, x] = [b, g, r]  # BGR格式
            
            # 添加时间文本
            current_time = datetime.now().strftime("%H:%M:%S")
            cv2.putText(img, f"Test Image - {current_time}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # 编码为JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
            result, encimg = cv2.imencode('.jpg', img, encode_param)
            
            if result:
                img_base64 = base64.b64encode(encimg).decode('utf-8')
                
                mqtt_data = {
                    'type': 'image',
                    'encoding': 'bgr8',
                    'width': width,
                    'height': height,
                    'image_data': img_base64,
                    'timestamp': time.time(),
                    'frame_id': 'test_camera',
                    'source': 'test_generator'
                }
                
                topic = f"{self.topic_prefix}/test_image"
                self.publish_to_mqtt(topic, mqtt_data)
                
        except Exception as e:
            self.get_logger().error(f"测试图像生成错误: {e}")

    def publish_test_depth(self):
        """发布测试深度图"""
        try:
            height, width = 480, 640
            
            # 创建测试深度图（模拟波纹效果）
            t = time.time()
            depth_img = np.zeros((height, width), dtype=np.float32)
            
            center_x, center_y = width // 2, height // 2
            for y in range(height):
                for x in range(width):
                    dist = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                    depth_img[y, x] = 128 + 64 * np.sin(t + dist/20.0)
            
            # 归一化并转换为uint8
            depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            
            # 编码为PNG
            result, encimg = cv2.imencode('.png', depth_normalized)
            
            if result:
                img_base64 = base64.b64encode(encimg).decode('utf-8')
                
                mqtt_data = {
                    'type': 'depth',
                    'encoding': 'mono16',
                    'width': width,
                    'height': height,
                    'image_data': img_base64,
                    'timestamp': time.time(),
                    'frame_id': 'test_depth',
                    'min_depth': float(np.min(depth_img)),
                    'max_depth': float(np.max(depth_img)),
                    'source': 'test_generator'
                }
                
                topic = f"{self.topic_prefix}/test_depth"
                self.publish_to_mqtt(topic, mqtt_data)
                
        except Exception as e:
            self.get_logger().error(f"测试深度图生成错误: {e}")

    def publish_test_pointcloud(self):
        """发布测试点云"""
        try:
            # 生成测试点云（螺旋形状）
            t = time.time()
            points = []
            
            for i in range(500):
                angle = i * 0.1 + t
                radius = i * 0.01
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                z = np.sin(angle * 2) * 0.5
                points.extend([x, y, z])
            
            mqtt_data = {
                'type': 'pointcloud',
                'width': 500,
                'height': 1,
                'points': points,
                'point_count': len(points) // 3,
                'timestamp': time.time(),
                'frame_id': 'test_lidar',
                'source': 'test_generator'
            }
            
            topic = f"{self.topic_prefix}/test_pointcloud"
            self.publish_to_mqtt(topic, mqtt_data)
            
        except Exception as e:
            self.get_logger().error(f"测试点云生成错误: {e}")

    def should_publish(self, topic):
        """检查是否应该发布消息（帧率控制）"""
        current_time = time.time()
        
        if topic not in self.last_publish_time:
            self.last_publish_time[topic] = current_time
            return True
        
        if current_time - self.last_publish_time[topic] >= self.min_interval:
            self.last_publish_time[topic] = current_time
            return True
        
        return False

    def publish_to_mqtt(self, topic, data):
        """发布数据到MQTT"""
        try:
            json_data = json.dumps(data)
            result = self.mqtt_client.publish(topic, json_data)
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warning(f"MQTT发布失败: {topic}")
                
        except Exception as e:
            self.get_logger().error(f"MQTT发布错误: {e}")

    def publish_statistics(self):
        """发布统计信息"""
        uptime = time.time() - self.stats['start_time']
        
        stats_data = {
            'type': 'statistics',
            'uptime': uptime,
            'images_processed': self.stats['images_processed'],
            'bytes_sent': self.stats['bytes_sent'],
            'avg_fps': self.stats['images_processed'] / uptime if uptime > 0 else 0,
            'avg_throughput': self.stats['bytes_sent'] / uptime if uptime > 0 else 0,
            'timestamp': time.time()
        }
        
        topic = f"{self.topic_prefix}/statistics"
        self.publish_to_mqtt(topic, stats_data)

    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = MediaMQTTBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
