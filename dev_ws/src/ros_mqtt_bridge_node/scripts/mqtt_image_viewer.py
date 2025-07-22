#!/usr/bin/env python3
"""
MQTT 图像订阅和显示工具
支持 Base64 和二进制图像数据的解码和显示
"""

import paho.mqtt.client as mqtt
import json
import base64
import cv2
import numpy as np
import argparse
import sys
import signal
from datetime import datetime


class MQTTImageViewer:
    def __init__(self, broker_host='localhost', broker_port=1883, topic='ros2/camera/image'):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.topic = topic
        self.client = None
        self.window_name = 'MQTT Image Viewer'
        self.save_images = False
        self.save_count = 0
        
    def setup_mqtt(self):
        """设置MQTT客户端"""
        try:
            self.client = mqtt.Client(
                client_id='image_viewer',
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message
            self.client.on_disconnect = self.on_disconnect
            
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
            print(f"正在订阅话题: {self.topic}")
            client.subscribe(self.topic)
        else:
            print(f"✗ MQTT连接失败，错误码: {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT断开回调"""
        print("MQTT连接已断开")
    
    def decode_compressed_image(self, data):
        """解码压缩图像数据（直接JPEG/PNG等格式）"""
        try:
            # 解析JSON（如果data是字符串）或直接使用（如果已经是字典）
            if isinstance(data, str):
                message = json.loads(data)
            elif isinstance(data, dict):
                message = data
            else:
                print(f"错误: 不支持的数据类型: {type(data)}")
                return None
            
            if 'data' not in message:
                print("错误: 消息中没有图像数据")
                return None
            
            # 处理压缩图像数据
            if message.get('encoding') == 'compressed_base64':
                # Base64编码的压缩数据
                img_data_str = message['data']
                if not isinstance(img_data_str, str):
                    print(f"错误: 图像数据应该是字符串，但得到: {type(img_data_str)}")
                    return None
                
                # 解码Base64
                img_data = base64.b64decode(img_data_str)
            else:
                # 直接的压缩数据
                img_data = message['data']
                if isinstance(img_data, str):
                    img_data = img_data.encode('utf-8')
            
            # 转换为numpy数组
            nparr = np.frombuffer(img_data, np.uint8)
            
            # 解码图像
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is not None:
                # 显示图像信息
                height, width = image.shape[:2]
                timestamp = message.get('timestamp', 'unknown')
                format_info = message.get('format', 'unknown')
                data_size = message.get('data_size', len(img_data))
                
                print(f"收到压缩图像: {width}x{height}, 格式: {format_info}, 大小: {data_size} 字节, 时间: {timestamp}")
                
            return image
            
        except json.JSONDecodeError:
            print("错误: 无法解析JSON数据")
            return None
        except Exception as e:
            print(f"解码压缩图像失败: {str(e)}")
            return None
        """解码Base64图像数据"""
        try:
            # 解析JSON（如果data是字符串）或直接使用（如果已经是字典）
            if isinstance(data, str):
                message = json.loads(data)
            elif isinstance(data, dict):
                message = data
            else:
                print(f"错误: 不支持的数据类型: {type(data)}")
                return None
            
            if 'data' not in message:
                print("错误: 消息中没有图像数据")
                return None
            
            # 确保data字段是字符串
            img_data_str = message['data']
            if not isinstance(img_data_str, str):
                print(f"错误: 图像数据应该是字符串，但得到: {type(img_data_str)}")
                return None
                
            # 解码Base64
            img_data = base64.b64decode(img_data_str)
            
            # 转换为numpy数组
            nparr = np.frombuffer(img_data, np.uint8)
            
            # 解码图像
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is not None:
                # 显示图像信息
                height, width = image.shape[:2]
                timestamp = message.get('timestamp', 'unknown')
                encoding = message.get('encoding', 'unknown')
                quality = message.get('quality', 'unknown')
                
                print(f"收到图像: {width}x{height}, 编码: {encoding}, 质量: {quality}, 时间: {timestamp}")
                
            return image
            
        except json.JSONDecodeError:
            print("错误: 无法解析JSON数据")
            return None
        except Exception as e:
            print(f"解码Base64图像失败: {str(e)}")
            return None
    
    def decode_binary_image(self, data):
        """解码二进制图像数据"""
        try:
            # 读取头部长度
            if len(data) < 4:
                print("错误: 数据太短")
                return None
                
            header_length = int.from_bytes(data[:4], byteorder='big')
            
            if len(data) < 4 + header_length:
                print("错误: 头部数据不完整")
                return None
            
            # 解析头部
            header_data = data[4:4+header_length]
            header = json.loads(header_data.decode('utf-8'))
            
            # 获取图像数据
            image_data = data[4+header_length:]
            
            # 转换为numpy数组并解码
            nparr = np.frombuffer(image_data, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is not None:
                # 显示图像信息
                timestamp = header.get('timestamp', 'unknown')
                encoding = header.get('encoding', 'unknown')
                width = header.get('width', 'unknown')
                height = header.get('height', 'unknown')
                
                print(f"收到图像: {width}x{height}, 编码: {encoding}, 时间: {timestamp}")
                
            return image
            
        except Exception as e:
            print(f"解码二进制图像失败: {str(e)}")
            return None
    
    def on_message(self, client, userdata, msg):
        """MQTT消息回调"""
        try:
            print(f"收到消息，话题: {msg.topic}, 数据长度: {len(msg.payload)} 字节")
            
            # 尝试解码图像
            image = None
            
            # 首先尝试作为Base64 JSON解码
            try:
                decoded_text = msg.payload.decode('utf-8')
                print("消息类型: UTF-8 文本")
                
                # 检查是否是JSON格式
                try:
                    json_data = json.loads(decoded_text)
                    print("消息格式: JSON")
                    
                    # 检查是否包含图像数据
                    if 'data' in json_data:
                        encoding_type = json_data.get('encoding', '')
                        
                        if 'compressed' in encoding_type:
                            print("检测到压缩图像数据")
                            image = self.decode_compressed_image(json_data)
                        elif 'base64' in encoding_type:
                            print("检测到Base64图像数据")
                            image = self.decode_base64_image(json_data)
                        else:
                            print("尝试通用图像解码")
                            # 尝试两种方法
                            image = self.decode_compressed_image(json_data)
                            if image is None:
                                image = self.decode_base64_image(json_data)
                    else:
                        print("JSON数据不包含图像，可能是元数据")
                        print(f"数据内容: {json.dumps(json_data, indent=2, ensure_ascii=False)[:300]}...")
                        
                except json.JSONDecodeError:
                    print("消息格式: 纯文本")
                    # 如果不是JSON，可能是直接的base64字符串
                    try:
                        img_data = base64.b64decode(decoded_text)
                        nparr = np.frombuffer(img_data, np.uint8)
                        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        if image is not None:
                            print("成功解码直接的Base64图像数据")
                    except Exception as e:
                        print(f"直接Base64解码失败: {str(e)}")
                        
            except UnicodeDecodeError:
                print("消息类型: 二进制数据")
                # 如果解码失败，尝试作为二进制数据
                image = self.decode_binary_image(msg.payload)
            
            if image is not None:
                print(f"成功解码图像，尺寸: {image.shape}")
                # 显示图像
                cv2.imshow(self.window_name, image)
                
                # 保存图像（如果启用）
                if self.save_images:
                    filename = f"mqtt_image_{self.save_count:04d}.jpg"
                    cv2.imwrite(filename, image)
                    print(f"图像已保存: {filename}")
                    self.save_count += 1
                
                # 等待按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("用户退出")
                    self.stop()
                elif key == ord('s'):
                    # 手动保存当前图像
                    filename = f"manual_save_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(filename, image)
                    print(f"手动保存图像: {filename}")
            else:
                print("无法解码图像数据")
                # 显示原始数据的前100个字符用于调试
                try:
                    preview = msg.payload.decode('utf-8')[:100]
                    print(f"数据预览: {preview}...")
                except:
                    preview = msg.payload[:100].hex()
                    print(f"二进制数据预览: {preview}...")
                
        except Exception as e:
            print(f"处理消息时出错: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def start(self, save_images=False):
        """启动图像查看器"""
        self.save_images = save_images
        
        if not self.setup_mqtt():
            return False
        
        # 创建OpenCV窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        print("图像查看器已启动")
        print("按键说明:")
        print("  'q' - 退出")
        print("  's' - 手动保存当前图像")
        print("等待图像数据...")
        
        try:
            # 启动MQTT循环
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n收到中断信号，正在退出...")
            self.stop()
        
        return True
    
    def stop(self):
        """停止查看器"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        cv2.destroyAllWindows()
        sys.exit(0)


def signal_handler(sig, frame):
    """信号处理器"""
    print('\n收到中断信号，正在退出...')
    cv2.destroyAllWindows()
    sys.exit(0)


def main():
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='MQTT图像查看器')
    parser.add_argument('--host', default='localhost', help='MQTT服务器地址')
    parser.add_argument('--port', type=int, default=1883, help='MQTT服务器端口')
    parser.add_argument('--topic', default='ros2/camera/image', help='MQTT图像话题')
    parser.add_argument('--save', action='store_true', help='自动保存接收到的图像')
    parser.add_argument('--list-topics', action='store_true', help='列出当前可用的MQTT话题')
    
    args = parser.parse_args()
    
    if args.list_topics:
        # 列出可用话题（需要另外实现）
        print("请使用 mosquitto_sub -h localhost -t '+' -v 查看所有话题")
        return
    
    # 创建并启动图像查看器
    viewer = MQTTImageViewer(args.host, args.port, args.topic)
    viewer.start(save_images=args.save)


if __name__ == '__main__':
    main()
