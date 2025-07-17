# Hello World MQTT Bridge Node

这个ROS2节点用于将`hello_world_node`发布的消息转发到MQTT服务器。该节点采用了解耦合的架构设计，分离了MQTT接口和ROS数据读取功能。

## 架构特性

### 模块化设计
- **MQTTInterface**: 通用MQTT接口类，处理所有MQTT相关操作
- **ROSNodeDataReader**: ROS节点数据读取器，负责从ROS话题读取数据
- **HelloWorldNodeDataReader**: 专门用于Hello World节点的数据读取器
- **HelloWorldMQTTBridge**: 主要的桥接节点，协调各个模块

### 低耦合设计
- MQTT功能与ROS功能完全分离
- 可以轻松扩展支持其他类型的ROS节点
- 消息格式标准化，便于维护
- 配置参数化，支持运行时调整

## 功能特性

- 订阅`hello_world`话题
- 将接收到的消息转换为JSON格式
- 发布到MQTT服务器的`ros2/hello_world`话题
- 支持MQTT认证
- 提供连接状态监控和自动重连
- 消息统计和历史记录功能
- 定期发布节点状态和统计信息

## 依赖项

- `rclpy`: ROS2 Python客户端库
- `std_msgs`: 标准消息类型
- `paho-mqtt`: MQTT客户端库

## 安装和构建

1. 确保已安装paho-mqtt:
   ```bash
   pip install paho-mqtt
   ```

2. 构建ROS2包:
   ```bash
   cd /home/tp/Project/WebCom/dev_ws
   ./build_mqtt_bridge.sh
   ```

3. 源环境设置:
   ```bash
   source install/setup.bash
   ```

## 使用方法

### 方法1: 使用便捷脚本

```bash
./run_hello_world_mqtt_bridge.sh
```

### 方法2: 直接运行节点

```bash
ros2 run ros_mqtt_bridge_node hello_world_mqtt_bridge
```

### 方法3: 使用参数运行

```bash
ros2 run ros_mqtt_bridge_node hello_world_mqtt_bridge --ros-args \
  -p mqtt_broker_host:="your_mqtt_broker" \
  -p mqtt_broker_port:=1883 \
  -p mqtt_topic_prefix:="ros2" \
  -p mqtt_client_id:="hello_world_bridge" \
  -p mqtt_username:="your_username" \
  -p mqtt_password:="your_password" \
  -p mqtt_keepalive:=60 \
  -p mqtt_qos:=1
```

### 方法4: 使用启动文件

```bash
ros2 launch ros_mqtt_bridge_node hello_world_mqtt_bridge.launch.py
```

### 方法5: 使用conda环境运行

```bash
./run_with_conda.sh
```

## 配置参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `mqtt_broker_host` | `localhost` | MQTT服务器地址 |
| `mqtt_broker_port` | `1883` | MQTT服务器端口 |
| `mqtt_topic_prefix` | `ros2` | MQTT主题前缀 |
| `mqtt_client_id` | `hello_world_bridge` | MQTT客户端ID |
| `mqtt_username` | `` | MQTT用户名(可选) |
| `mqtt_password` | `` | MQTT密码(可选) |

## MQTT消息格式

节点会将ROS2消息转换为以下JSON格式发布到MQTT:

```json
{
  "timestamp": "2025-07-17T10:30:00.123456",
  "source_node": "hello_world_publisher",
  "source_topic": "hello_world",
  "message_id": 1,
  "data": "Hello World! 计数: 0",
  "frame_id": "hello_world_frame"
}
```

## 完整运行示例

1. 启动hello_world_publisher节点:
   ```bash
   ros2 run hello_world_node hello_world_publisher
   ```

2. 启动MQTT bridge节点:
   ```bash
   ros2 run ros_mqtt_bridge_node hello_world_mqtt_bridge
   ```

3. 使用MQTT客户端订阅消息:
   ```bash
   mosquitto_sub -h localhost -t "ros2/hello_world"
   ```

## 故障排除

1. **MQTT连接失败**: 检查MQTT服务器地址和端口是否正确
2. **认证失败**: 确认用户名和密码是否正确
3. **消息未发布**: 检查hello_world_node是否正在运行
4. **依赖项缺失**: 确保已安装paho-mqtt库

## 日志输出

节点会输出以下类型的日志:
- 连接状态信息
- 消息发布成功/失败
- 错误和警告信息
- 统计信息
