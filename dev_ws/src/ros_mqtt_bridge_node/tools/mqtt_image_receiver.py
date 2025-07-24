#!/usr/bin/env python3
"""
MQTT 图像数据读取和保存工具
支持从MQTT接收Base64编码的图像数据并保存为图片文件
"""

import paho.mqtt.client as mqtt
import json
import base64
import cv2
import numpy as np
import argparse
import os
import sys
from datetime import datetime
import time
import zipfile
import shutil
import threading
from concurrent.futures import ThreadPoolExecutor
import queue


class MQTTImageReceiver:
    """MQTT图像接收器"""
    
    def __init__(self, broker_host='localhost', broker_port=1883, save_dir='mqtt_images', 
                 archive_enabled=False, archive_batch_size=10, archive_dir='mqtt_archives'):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.save_dir = save_dir
        self.client = None
        self.image_count = 0
        
        # 压缩打包相关参数
        self.archive_enabled = archive_enabled
        self.archive_batch_size = archive_batch_size
        self.archive_dir = archive_dir
        self.archive_count = 0
        self.pending_files = []  # 待打包的文件列表
        
        # 异步压缩相关
        self.archive_lock = threading.Lock()  # 保护 pending_files 列表
        self.archive_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="archive")
        self.archive_futures = []  # 跟踪压缩任务
        
        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)
        print(f"图像保存目录: {os.path.abspath(self.save_dir)}")
        
        # 如果启用压缩打包，创建归档目录
        if self.archive_enabled:
            os.makedirs(self.archive_dir, exist_ok=True)
            print(f"压缩包保存目录: {os.path.abspath(self.archive_dir)}")
            print(f"压缩批次大小: {self.archive_batch_size} 张图片/包")
        
    def setup_mqtt(self):
        """设置MQTT客户端"""
        try:
            self.client = mqtt.Client(
                client_id='image_receiver',
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
            print("✓ 已连接到MQTT服务器")
            # 订阅所有可能的图像话题
            topics = [
                'ros2/image_compressed/data',
                'ros2/image_raw/data', 
                'ros2/camera/image',
                #'ros2/+/+',  # 通配符，接收所有ROS2相关话题
            ]
            
            for topic in topics:
                client.subscribe(topic)
                print(f"已订阅话题: {topic}")
        else:
            print(f"✗ MQTT连接失败，错误码: {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT断开回调"""
        print("MQTT连接已断开")
    
    def add_file_to_archive_queue(self, filepath):
        """添加文件到压缩打包队列（线程安全）"""
        if not self.archive_enabled:
            return
        
        with self.archive_lock:
            self.pending_files.append(filepath)
            pending_count = len(self.pending_files)
            
        print(f"  添加到打包队列: {os.path.basename(filepath)} (队列中 {pending_count}/{self.archive_batch_size})")
        
        # 检查是否达到批次大小（不阻塞）
        if pending_count >= self.archive_batch_size:
            self.schedule_archive_creation()
    
    def schedule_archive_creation(self):
        """安排异步创建压缩包"""
        with self.archive_lock:
            if len(self.pending_files) >= self.archive_batch_size:
                # 创建当前批次的文件列表副本
                files_to_archive = self.pending_files[:self.archive_batch_size]
                # 从待打包列表中移除这些文件
                self.pending_files = self.pending_files[self.archive_batch_size:]
                
                # 提交异步压缩任务
                future = self.archive_executor.submit(self._create_archive_async, files_to_archive)
                self.archive_futures.append(future)
                
                print(f"  🔄 已安排压缩任务，剩余队列: {len(self.pending_files)} 个文件")
    
    def _create_archive_async(self, files_to_archive):
        """异步创建压缩包（在后台线程中执行）"""
        try:
            self.archive_count += 1
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            archive_name = f"mqtt_images_batch_{self.archive_count:04d}_{timestamp}.zip"
            archive_path = os.path.join(self.archive_dir, archive_name)
            
            print(f"\n📦 后台创建压缩包: {archive_name}")
            print(f"   包含 {len(files_to_archive)} 个文件")
            
            with zipfile.ZipFile(archive_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as zipf:
                for file_path in files_to_archive:
                    if os.path.exists(file_path):
                        # 使用相对路径作为归档内的文件名
                        arcname = os.path.basename(file_path)
                        zipf.write(file_path, arcname)
                        print(f"   ✓ 已添加: {arcname}")
                    else:
                        print(f"   ✗ 文件不存在: {file_path}")
            
            # 获取压缩包大小信息
            archive_size = os.path.getsize(archive_path)
            original_size = sum(os.path.getsize(f) for f in files_to_archive if os.path.exists(f))
            compression_ratio = (1 - archive_size / original_size) * 100 if original_size > 0 else 0
            
            print(f"   📊 压缩完成:")
            print(f"      原始大小: {original_size:,} 字节")
            print(f"      压缩后: {archive_size:,} 字节")
            print(f"      压缩率: {compression_ratio:.1f}%")
            print(f"   💾 保存到: {archive_path}")
            
            # 可选：删除原始文件
            self._cleanup_files_async(files_to_archive)
            
            return archive_path
            
        except Exception as e:
            print(f"✗ 后台创建压缩包失败: {str(e)}")
            return None
    
    def _cleanup_files_async(self, files_to_cleanup):
        """异步清理文件"""
        print(f"   🗑️  清理原始文件:")
        for file_path in files_to_cleanup:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
                    print(f"      ✓ 已删除: {os.path.basename(file_path)}")
            except Exception as e:
                print(f"      ✗ 删除失败: {os.path.basename(file_path)} - {str(e)}")
    
    def create_archive(self):
        """立即创建压缩包（同步版本，保留用于手动调用）"""
        if not self.pending_files:
            return
            
        with self.archive_lock:
            files_to_archive = self.pending_files.copy()
            self.pending_files.clear()
        
        return self._create_archive_async(files_to_archive)
    
    def cleanup_archive_tasks(self):
        """清理已完成的异步压缩任务"""
        if not self.archive_futures:
            return
            
        completed_futures = []
        for future in self.archive_futures:
            if future.done():
                completed_futures.append(future)
                try:
                    result = future.result()  # 获取结果，如果有异常会抛出
                    if result:
                        print(f"  ✅ 压缩任务完成: {os.path.basename(result)}")
                except Exception as e:
                    print(f"  ❌ 压缩任务失败: {e}")
        
        # 从列表中移除已完成的任务
        for future in completed_futures:
            self.archive_futures.remove(future)
    
    def finalize_remaining_files(self):
        """处理剩余未打包的文件"""
        if self.archive_enabled and self.pending_files:
            print(f"\n📦 处理剩余的 {len(self.pending_files)} 个文件...")
            self.create_archive()
        
        # 等待所有异步压缩任务完成
        if hasattr(self, 'archive_executor') and self.archive_executor:
            print("⏳ 等待压缩任务完成...")
            self.archive_executor.shutdown(wait=True)
            print("✅ 所有压缩任务已完成")
    
    def decode_and_save_image(self, image_data, topic, timestamp=None):
        """解码并保存图像数据"""
        try:
            if timestamp is None:
                timestamp = datetime.now()
            
            print(f"  开始解码图像数据，类型: {type(image_data)}")
            
            # 解码Base64数据
            if isinstance(image_data, dict):
                # 结构化数据
                encoding = image_data.get('encoding', 'unknown')
                data_type = image_data.get('data_type', 'unknown')
                base64_data = image_data.get('data', '')
                
                print(f"  解码参数 - 编码: {encoding}, 类型: {data_type}, Base64长度: {len(base64_data)}")
                print(f"  原始大小: {image_data.get('original_size', 'unknown')} 字节")
                
                if encoding == 'base64' and base64_data:
                    print(f"  开始Base64解码...")
                    try:
                        # 解码Base64数据
                        img_bytes = base64.b64decode(base64_data)
                        print(f"  Base64解码成功，得到 {len(img_bytes)} 字节")
                        
                        # 生成文件名
                        safe_topic = topic.replace('/', '_').replace('+', 'all')
                        timestamp_str = timestamp.strftime('%Y%m%d_%H%M%S_%f')[:-3]  # 毫秒精度
                        filename = f"{safe_topic}_{timestamp_str}_{self.image_count:04d}.jpg"
                        filepath = os.path.join(self.save_dir, filename)
                        
                        print(f"  准备保存到: {filepath}")
                        
                        if data_type == 'uint8_array':
                            # 对于压缩图像数据，直接保存
                            print(f"  保存为压缩图像文件...")
                            with open(filepath, 'wb') as f:
                                f.write(img_bytes)
                            print(f"✓ 压缩图像已保存: {filename} ({len(img_bytes)} 字节)")
                            
                            # 添加到压缩打包队列
                            self.add_file_to_archive_queue(filepath)
                            
                        elif data_type in ['bytes', 'numpy_array']:
                            # 尝试使用OpenCV解码
                            print(f"  尝试OpenCV解码...")
                            nparr = np.frombuffer(img_bytes, np.uint8)
                            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            
                            if img is not None:
                                cv2.imwrite(filepath, img)
                                print(f"✓ 图像已保存: {filename} (尺寸: {img.shape})")
                                # 添加到压缩打包队列
                                self.add_file_to_archive_queue(filepath)
                            else:
                                # 如果OpenCV解码失败，直接保存二进制数据
                                print(f"  OpenCV解码失败，保存为二进制文件...")
                                bin_filepath = filepath.replace('.jpg', '.bin')
                                with open(bin_filepath, 'wb') as f:
                                    f.write(img_bytes)
                                print(f"✓ 二进制数据已保存: {filename.replace('.jpg', '.bin')} ({len(img_bytes)} 字节)")
                                # 添加到压缩打包队列
                                self.add_file_to_archive_queue(bin_filepath)
                        else:
                            # 未知类型，尝试直接保存
                            print(f"  未知数据类型，尝试直接保存...")
                            with open(filepath, 'wb') as f:
                                f.write(img_bytes)
                            print(f"✓ 数据已保存: {filename} ({len(img_bytes)} 字节)")
                            # 添加到压缩打包队列
                            self.add_file_to_archive_queue(filepath)
                        
                        self.image_count += 1
                        return True
                        
                    except Exception as decode_error:
                        print(f"  ✗ Base64解码失败: {str(decode_error)}")
                        return False
                else:
                    print(f"  ✗ 不是有效的Base64编码数据 (encoding={encoding}, data_len={len(base64_data)})")
                    return False
                    
            elif isinstance(image_data, str):
                # 直接的Base64字符串
                try:
                    img_bytes = base64.b64decode(image_data)
                    
                    # 生成文件名
                    safe_topic = topic.replace('/', '_').replace('+', 'all')
                    timestamp_str = timestamp.strftime('%Y%m%d_%H%M%S_%f')[:-3]
                    filename = f"{safe_topic}_{timestamp_str}_{self.image_count:04d}.jpg"
                    filepath = os.path.join(self.save_dir, filename)
                    
                    # 尝试OpenCV解码
                    nparr = np.frombuffer(img_bytes, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if img is not None:
                        cv2.imwrite(filepath, img)
                        print(f"✓ 图像已保存: {filename} (尺寸: {img.shape})")
                        # 添加到压缩打包队列
                        self.add_file_to_archive_queue(filepath)
                    else:
                        # 直接保存二进制数据
                        bin_filepath = filepath.replace('.jpg', '.bin')
                        with open(bin_filepath, 'wb') as f:
                            f.write(img_bytes)
                        print(f"✓ 二进制数据已保存: {filename.replace('.jpg', '.bin')}")
                        # 添加到压缩打包队列
                        self.add_file_to_archive_queue(bin_filepath)
                    
                    self.image_count += 1
                    return True
                    
                except Exception as e:
                    print(f"解码直接Base64数据失败: {str(e)}")
                    
            return False
            
        except Exception as e:
            print(f"解码和保存图像失败: {str(e)}")
            return False
    
    def on_message(self, client, userdata, msg):
        """MQTT消息回调"""
        try:
            # 定期清理已完成的异步压缩任务
            self.cleanup_archive_tasks()
            
            topic = msg.topic
            timestamp = datetime.now()
            
            print(f"\n[{timestamp.strftime('%H:%M:%S')}] 收到消息:")
            print(f"  话题: {topic}")
            print(f"  数据长度: {len(msg.payload)} 字节")
            
            # 尝试解析消息
            try:
                # 首先尝试解析为JSON
                message_text = msg.payload.decode('utf-8')
                message_data = json.loads(message_text)
                
                print(f"  消息类型: JSON")
                
                # 检查是否包含图像数据
                if isinstance(message_data, dict):
                    # 检查是否有直接的图像数据字段
                    if 'data' in message_data:
                        data_field = message_data['data']
                        
                        if isinstance(data_field, dict):
                            # 检查不同的编码格式
                            print(f"  数据字段是字典，键: {list(data_field.keys())}")
                            
                            if data_field.get('encoding') == 'base64':
                                # 新格式：包含encoding字段
                                print(f"  发现Base64编码图像数据 (新格式)")
                                success = self.decode_and_save_image(data_field, topic, timestamp)
                                if success:
                                    print(f"  ✓ 图像处理成功")
                                else:
                                    print(f"  ✗ 图像处理失败")
                            elif data_field.get('type') == 'uint8_array' and 'data' in data_field:
                                # MQTT接口的自定义序列化格式
                                print(f"  发现uint8_array图像数据 (MQTT格式)")
                                print(f"  数据大小: {data_field.get('size', 0)} 字节")
                                print(f"  Base64数据长度: {len(data_field.get('data', ''))} 字符")
                                
                                # 转换为统一格式
                                unified_data = {
                                    'encoding': 'base64',
                                    'data_type': 'uint8_array',
                                    'original_size': data_field.get('size', 0),
                                    'data': data_field.get('data', ''),
                                    'timestamp': datetime.now().isoformat()
                                }
                                print(f"  转换后的数据格式: {list(unified_data.keys())}")
                                success = self.decode_and_save_image(unified_data, topic, timestamp)
                                if success:
                                    print(f"  ✓ 图像处理成功")
                                else:
                                    print(f"  ✗ 图像处理失败")
                            else:
                                print(f"  未识别的数据字段格式")
                                print(f"  数据字段结构: {list(data_field.keys())}")
                                print(f"  数据字段预览: {str(data_field)[:200]}...")
                        elif isinstance(data_field, str) and len(data_field) > 100:
                            # 可能是直接的Base64字符串
                            print(f"  发现可能的Base64字符串 (长度: {len(data_field)})")
                            success = self.decode_and_save_image(data_field, topic, timestamp)
                            if success:
                                print(f"  ✓ 图像处理成功")
                            else:
                                print(f"  ✗ 图像处理失败")
                        else:
                            print(f"  数据字段类型: {type(data_field)}, 内容预览: {str(data_field)[:100]}...")
                    else:
                        print(f"  消息结构: {list(message_data.keys())}")
                        # 显示消息内容预览
                        preview = json.dumps(message_data, indent=2, ensure_ascii=False)[:300]
                        print(f"  内容预览: {preview}...")
                else:
                    print(f"  消息不是字典格式: {type(message_data)}")
                    
            except UnicodeDecodeError:
                print(f"  消息类型: 二进制数据")
                print(f"  前32字节: {msg.payload[:32].hex()}")
                
            except json.JSONDecodeError:
                print(f"  消息类型: 非JSON文本")
                try:
                    text_content = msg.payload.decode('utf-8')
                    print(f"  文本内容预览: {text_content[:100]}...")
                    
                    # 检查是否是直接的Base64数据
                    if len(text_content) > 100 and text_content.replace('\n', '').replace(' ', '').isalnum():
                        print(f"  尝试作为Base64数据处理...")
                        success = self.decode_and_save_image(text_content, topic, timestamp)
                        if success:
                            print(f"  ✓ 图像处理成功")
                        else:
                            print(f"  ✗ 图像处理失败")
                except:
                    print(f"  无法解码为文本")
            
            print("-" * 60)
            
        except Exception as e:
            print(f"处理消息时出错: {str(e)}")
    
    def start_receiving(self):
        """开始接收图像"""
        if not self.setup_mqtt():
            return False
        
        print("\n图像接收器已启动")
        print("等待图像数据...")
        print("按 Ctrl+C 退出")
        print("=" * 60)
        
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n收到中断信号，正在退出...")
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
        
        # 处理剩余未打包的文件
        self.finalize_remaining_files()
        
        print(f"\n接收会话结束，共保存 {self.image_count} 张图像")
        if self.archive_enabled:
            print(f"共创建 {self.archive_count} 个压缩包")
            print(f"压缩包保存在: {os.path.abspath(self.archive_dir)}")
        return True


def main():
    parser = argparse.ArgumentParser(description='MQTT图像接收和保存工具')
    parser.add_argument('--host', default='localhost', help='MQTT服务器地址')
    parser.add_argument('--port', type=int, default=1883, help='MQTT服务器端口')
    parser.add_argument('--save-dir', default='mqtt_images', help='图像保存目录')
    parser.add_argument('--list-topics', action='store_true', help='仅列出活跃的MQTT话题')
    
    # 压缩打包相关参数
    parser.add_argument('--enable-archive', action='store_true', 
                       help='启用自动压缩打包功能')
    parser.add_argument('--archive-batch-size', type=int, default=100,
                       help='每个压缩包包含的图片数量 (默认: 100)')
    parser.add_argument('--archive-dir', default='mqtt_archives',
                       help='压缩包保存目录 (默认: mqtt_archives)')
    
    args = parser.parse_args()
    
    if args.list_topics:
        # 简单的话题列表功能
        print("使用以下命令查看活跃的MQTT话题:")
        print(f"mosquitto_sub -h {args.host} -p {args.port} -t '+' -v")
        return
    
    # 显示配置信息
    print("="*60)
    print("MQTT图像接收器配置:")
    print(f"  MQTT服务器: {args.host}:{args.port}")
    print(f"  图像保存目录: {args.save_dir}")
    if args.enable_archive:
        print(f"  压缩打包: 启用")
        print(f"  批次大小: {args.archive_batch_size} 张图片/包")
        print(f"  压缩包目录: {args.archive_dir}")
    else:
        print(f"  压缩打包: 禁用 (使用 --enable-archive 启用)")
    print("="*60)
    
    # 创建并启动图像接收器
    receiver = MQTTImageReceiver(
        broker_host=args.host, 
        broker_port=args.port, 
        save_dir=args.save_dir,
        archive_enabled=args.enable_archive,
        archive_batch_size=args.archive_batch_size,
        archive_dir=args.archive_dir
    )
    receiver.start_receiving()


if __name__ == '__main__':
    main()
