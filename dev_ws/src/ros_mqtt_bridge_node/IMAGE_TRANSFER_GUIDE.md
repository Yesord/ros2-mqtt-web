# MQTT 图像数据传输使用指南

## 概述

本系统已经修改为支持 uint8[] 图像数据的Base64编码传输，可以通过MQTT传输ROS2图像数据并在客户端保存为图片文件。

## 功能特点

- ✅ 自动检测图像数据字段（sensor_msgs/Image 和 sensor_msgs/CompressedImage）
- ✅ 智能Base64编码处理 uint8[] 数组
- ✅ 支持 numpy 数组、列表和字节数据
- ✅ 完整的图像接收和保存脚本
- ✅ 详细的调试信息和错误处理

## 使用方法

### 1. 启动图像数据桥接

```bash
# 进入工作目录
cd /home/seeed/Project/ros2-mqtt-web-main/test_ws

# 加载环境
source install/setup.bash

# 启动多桥接管理器（包含图像数据桥接）
ros2 run ros_mqtt_bridge_node multi_bridge_manager
```

### 2. 接收和保存图像

在另一个终端中启动图像接收器：

```bash
# 基本使用
python3 /home/seeed/Project/ros2-mqtt-web-main/test_ws/src/ros_mqtt_bridge_node/scripts/mqtt_image_receiver.py

# 指定保存目录
python3 /home/seeed/Project/ros2-mqtt-web-main/test_ws/src/ros_mqtt_bridge_node/scripts/mqtt_image_receiver.py --save-dir ./my_images

# 指定MQTT服务器
python3 /home/seeed/Project/ros2-mqtt-web-main/test_ws/src/ros_mqtt_bridge_node/scripts/mqtt_image_receiver.py --host 192.168.1.100 --port 1883
```

### 3. 查看活跃的MQTT话题

```bash
# 使用脚本查看
python3 /home/seeed/Project/ros2-mqtt-web-main/test_ws/src/ros_mqtt_bridge_node/scripts/mqtt_image_receiver.py --list-topics

# 或直接使用mosquitto命令
mosquitto_sub -h localhost -t '+' -v
```

## 配置说明

当前配置的图像桥接器：

### 图像元数据桥接 (Image_bridge)
- **话题**: `/image_raw` 
- **数据**: 高度、宽度、编码格式
- **MQTT话题**: `ros2/image_raw/metadata`

### 压缩图像数据桥接 (Compressed_Image_Data_bridge)
- **话题**: `/image_raw/compressed`
- **数据**: 完整的压缩图像数据（Base64编码）
- **MQTT话题**: `ros2/image_compressed/data`

## 数据格式

### Base64编码的图像数据格式：

```json
{
  "timestamp": "2025-07-22T10:30:45.123456",
  "source_node": "ros_bridge",
  "source_topic": "/image_raw/compressed",
  "data": {
    "encoding": "base64",
    "data_type": "uint8_array",
    "original_size": 25836,
    "data": "iVBORw0KGgoAAAANSUhEUgAA...",
    "timestamp": "2025-07-22T10:30:45.123456"
  },
  "message_id": 1234,
  "frame_id": "camera_frame",
  "bridge_name": "Compressed_Image_Data_bridge"
}
```

## 故障排除

### 1. 图像数据无法传输
- 检查是否有图像数据在发布：`ros2 topic list | grep image`
- 查看话题数据：`ros2 topic echo /image_raw/compressed --once`
- 检查桥接器状态：查看multi_bridge_manager的日志

### 2. 图像保存失败
- 确认保存目录有写权限
- 检查磁盘空间
- 查看接收器的错误日志

### 3. MQTT连接问题
- 确认Mosquitto服务运行：`sudo systemctl status mosquitto`
- 测试MQTT连接：`mosquitto_pub -h localhost -t test -m "hello"`

## 文件说明

### 修改的核心文件：
1. **mqtt_interface.py** - 添加了自定义JSON序列化器支持Base64编码
2. **topic_bridge.py** - 添加了图像数据检测和Base64编码处理
3. **multi_bridge_config.yaml** - 启用了压缩图像数据桥接

### 新增的工具脚本：
1. **mqtt_image_receiver.py** - 完整的图像接收和保存工具

## 性能考虑

- Base64编码会增加约33%的数据量
- 建议对图像进行压缩处理
- 大图像可能影响MQTT传输性能
- 可以通过调整QoS和队列大小优化性能

## 扩展功能

如需添加更多图像处理功能，可以修改：
- `_process_field_data` 方法添加其他数据类型支持
- `mqtt_image_receiver.py` 添加图像预处理功能
- 配置文件添加新的图像话题桥接
