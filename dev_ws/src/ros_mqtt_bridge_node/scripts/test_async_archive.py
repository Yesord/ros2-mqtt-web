#!/usr/bin/env python3
"""
测试异步压缩功能的脚本
模拟连续接收图像数据的场景，验证是否会掉帧
"""

import os
import sys
import time
import json
import base64
import random
from datetime import datetime
import paho.mqtt.client as mqtt
import threading

def create_dummy_image_data(image_id):
    """创建模拟的图像数据"""
    # 生成一些随机的"图像"数据
    dummy_image_bytes = bytes([random.randint(0, 255) for _ in range(10000)])  # 10KB模拟图像
    base64_data = base64.b64encode(dummy_image_bytes).decode('utf-8')
    
    return {
        "data": {
            "encoding": "base64",
            "data_type": "uint8_array", 
            "original_size": len(dummy_image_bytes),
            "data": base64_data,
            "image_id": image_id,
            "timestamp": datetime.now().isoformat()
        }
    }

def mqtt_publisher_worker(host="localhost", port=1883, topic="test/camera/image", interval=0.1, total_images=50):
    """MQTT发布者工作线程"""
    try:
        # 创建MQTT客户端
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        
        print(f"🔌 连接到MQTT服务器 {host}:{port}")
        client.connect(host, port, 60)
        client.loop_start()
        
        print(f"📤 开始发送 {total_images} 张模拟图像，间隔: {interval}s")
        
        for i in range(1, total_images + 1):
            # 创建模拟图像数据
            image_data = create_dummy_image_data(i)
            message = json.dumps(image_data)
            
            # 发布消息
            result = client.publish(topic, message)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"📷 [{i:02d}/{total_images}] 发送图像 ID: {i} (大小: {len(message)} 字节)")
            else:
                print(f"❌ [{i:02d}/{total_images}] 发送失败: {result.rc}")
            
            time.sleep(interval)
        
        print("✅ 所有图像发送完成")
        client.loop_stop()
        client.disconnect()
        
    except Exception as e:
        print(f"❌ 发布者出错: {e}")

def main():
    print("🧪 异步压缩功能测试")
    print("=" * 50)
    
    if len(sys.argv) < 2:
        print("用法: python3 test_async_archive.py <test_mode>")
        print("测试模式:")
        print("  publisher  - 作为MQTT发布者，发送模拟图像数据")
        print("  receiver   - 作为接收者，启动mqtt_image_receiver.py")
        return
    
    test_mode = sys.argv[1].lower()
    
    if test_mode == "publisher":
        print("🚀 启动发布者模式")
        
        # 参数配置
        host = "localhost"
        port = 1883
        topic = "test/camera/image"
        interval = 0.2  # 每200ms发送一张图像
        total_images = 25  # 总共发送25张图像
        
        print(f"📋 测试配置:")
        print(f"   MQTT服务器: {host}:{port}")
        print(f"   发布话题: {topic}")
        print(f"   发送间隔: {interval}s")
        print(f"   图像总数: {total_images}")
        print(f"   预期压缩包: {(total_images + 9) // 10} 个 (每10张图片一个)")
        
        print("\n⏳ 5秒后开始发送...")
        time.sleep(5)
        
        # 启动发布者线程
        mqtt_publisher_worker(host, port, topic, interval, total_images)
        
    elif test_mode == "receiver":
        print("🎯 启动接收者模式")
        
        # 启动接收者
        receiver_cmd = [
            "python3", "mqtt_image_receiver.py",
            "--host", "localhost",
            "--port", "1883", 
            "--save-dir", "test_images",
            "--enable-archive",
            "--archive-batch-size", "10",
            "--archive-dir", "test_archives"
        ]
        
        print(f"🏃 执行命令: {' '.join(receiver_cmd)}")
        os.execvp("python3", receiver_cmd)
        
    else:
        print(f"❌ 未知测试模式: {test_mode}")
        print("支持的模式: publisher, receiver")

if __name__ == "__main__":
    main()
