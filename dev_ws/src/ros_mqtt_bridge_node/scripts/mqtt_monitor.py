#!/usr/bin/env python3
"""
MQTT 话题监控工具
用于查看和分析 MQTT 话题上的数据
"""

import paho.mqtt.client as mqtt
import json
import argparse
import sys
from datetime import datetime


class MQTTMonitor:
    def __init__(self, broker_host='localhost', broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = None
        self.message_count = 0
        
    def setup_mqtt(self):
        """设置MQTT客户端"""
        try:
            self.client = mqtt.Client(
                client_id='mqtt_monitor',
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message
            
            print(f"正在连接到 MQTT 服务器 {self.broker_host}:{self.broker_port}...")
            self.client.connect(self.broker_host, self.broker_port, 60)
            return True
            
        except Exception as e:
            print(f"MQTT连接失败: {str(e)}")
            return False
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            print(f"✓ 已连接到MQTT服务器")
        else:
            print(f"✗ MQTT连接失败，错误码: {rc}")
    
    def on_message(self, client, userdata, msg):
        """MQTT消息回调"""
        self.message_count += 1
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        print(f"\n[{self.message_count}] {timestamp}")
        print(f"话题: {msg.topic}")
        print(f"QoS: {msg.qos}")
        print(f"保留: {msg.retain}")
        print(f"数据长度: {len(msg.payload)} 字节")
        
        # 尝试解析数据
        try:
            # 尝试作为UTF-8字符串解码
            text_data = msg.payload.decode('utf-8')
            
            # 尝试解析为JSON
            try:
                json_data = json.loads(text_data)
                print("数据类型: JSON")
                print("内容预览:")
                print(json.dumps(json_data, indent=2, ensure_ascii=False)[:500])
                if len(json.dumps(json_data)) > 500:
                    print("... (内容过长，已截断)")
            except json.JSONDecodeError:
                print("数据类型: 文本")
                print(f"内容: {text_data[:200]}")
                if len(text_data) > 200:
                    print("... (内容过长，已截断)")
                    
        except UnicodeDecodeError:
            print("数据类型: 二进制")
            print(f"前32字节: {msg.payload[:32].hex()}")
        
        print("-" * 60)
    
    def subscribe_topic(self, topic):
        """订阅话题"""
        if self.client:
            print(f"正在订阅话题: {topic}")
            self.client.subscribe(topic)
    
    def start_monitoring(self, topics):
        """开始监控"""
        if not self.setup_mqtt():
            return False
        
        # 订阅话题
        for topic in topics:
            self.subscribe_topic(topic)
        
        print(f"开始监控 {len(topics)} 个话题...")
        print("按 Ctrl+C 退出")
        
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n收到中断信号，正在退出...")
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
        
        return True


def main():
    parser = argparse.ArgumentParser(description='MQTT话题监控工具')
    parser.add_argument('--host', default='localhost', help='MQTT服务器地址')
    parser.add_argument('--port', type=int, default=1883, help='MQTT服务器端口')
    parser.add_argument('--topics', nargs='+', 
                       default=['ros2/+/+'], 
                       help='要监控的话题列表（支持通配符）')
    
    args = parser.parse_args()
    
    monitor = MQTTMonitor(args.host, args.port)
    monitor.start_monitoring(args.topics)


if __name__ == '__main__':
    main()
