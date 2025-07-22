# ROS2 MQTT Bridge Node

这个ROS2节点用于将多个ROS话题的消息转发到MQTT服务器。该节点采用了现代化的多桥接器架构设计，支持通过YAML配置文件灵活管理多个话题桥接。

## 架构特性

### 现代化多桥接器设计
- **MultiBridgeManager**: 统一管理多个话题桥接的主节点
- **TopicBridge**: 通用话题桥接器，支持动态消息类型
- **MQTTInterface**: 通用MQTT接口类，处理所有MQTT相关操作
- **ConfigLoader**: YAML配置文件加载器，支持配置驱动的桥接管理

### 高效统一架构
- 一个节点管理多个话题桥接
- 配置驱动的桥接管理，无需修改代码即可添加新话题
- 消息格式标准化，便于维护
- 支持运行时参数调整

## 功能特性

- 同时桥接多个ROS话题到MQTT
- 支持不同的ROS消息类型 (String, Twist, 等)
- 智能数据字段提取
- 将ROS消息转换为标准JSON格式
- 支持MQTT认证和QoS配置
- 提供连接状态监控和自动重连
- 消息统计和历史记录功能
- 定期发布节点状态和统计信息

## 依赖项

- `rclpy`: ROS2 Python客户端库
- `std_msgs`: 标准消息类型
- `geometry_msgs`: 几何消息类型
- `paho-mqtt`: MQTT客户端库
- `PyYAML`: YAML配置文件解析

## 安装和构建

1. 确保已安装依赖:
   ```bash
   pip install paho-mqtt PyYAML
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

### 方法1: 直接运行多桥接管理器

```bash
ros2 run ros_mqtt_bridge_node multi_bridge_manager
```

### 方法2: 使用参数运行

```bash
ros2 run ros_mqtt_bridge_node multi_bridge_manager --ros-args \
  -p mqtt_broker_host:="your_mqtt_broker" \
  -p mqtt_broker_port:=1883 \
  -p mqtt_topic_prefix:="ros2" \
  -p mqtt_username:="your_username" \
  -p mqtt_password:="your_password"
```

### 方法3: 使用启动文件

```bash
ros2 launch ros_mqtt_bridge_node multi_bridge_manager.launch.py
```

### 方法4: 使用conda环境运行

```bash
./run_with_conda.sh
```

## 配置文件

### 多桥接器配置 (multi_bridge_config.yaml)

系统使用YAML配置文件定义桥接器，默认配置包括：

- **hello_world_bridge**: 桥接 `/hello_world` 话题到 `ros2/hello_world/data`
- **turtle_cmd_vel_bridge**: 桥接 `/turtle1/cmd_vel` 话题到 `ros2/turtle/cmd_vel`

#### 配置示例:
```yaml
bridges:
  - name: "my_custom_bridge"
    description: "自定义话题桥接"
    enabled: true
    ros_config:
      topic: "/my_topic"
      message_type: "sensor_msgs/LaserScan"
      data_field: "ranges"
      queue_size: 10
    mqtt_config:
      topic_name: "laser"
      topic_suffix: "scan"
      qos: 1
      retain: false
```

## 配置参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `mqtt_broker_host` | `localhost` | MQTT服务器地址 |
| `mqtt_broker_port` | `1883` | MQTT服务器端口 |
| `mqtt_topic_prefix` | `ros2` | MQTT主题前缀 |
| `enable_statistics` | `true` | 启用统计信息 |
| `enable_heartbeat` | `true` | 启用心跳检测 |

## 消息格式

MQTT消息采用标准JSON格式：

```json
{
  "timestamp": "2025-07-18T16:35:24.031792",
  "source_node": "hello_world_publisher",
  "source_topic": "/hello_world", 
  "data": "Hello World! 计数: 2236",
  "message_id": 2236,
  "frame_id": "hello_world_frame",
  "bridge_name": "hello_world_bridge"
}
```

## 测试和验证

### 检查MQTT消息

```bash
# 订阅hello_world消息
mosquitto_sub -h localhost -t 'ros2/hello_world/data'

# 订阅turtle命令消息  
mosquitto_sub -h localhost -t 'ros2/turtle/cmd_vel'

# 订阅所有ros2消息
mosquitto_sub -h localhost -t 'ros2/+/+'
```

### 检查节点状态

```bash
# 查看节点信息
ros2 node info /multi_bridge_manager

# 查看话题列表
ros2 topic list

# 查看参数
ros2 param list /multi_bridge_manager
```

## 扩展新话题

要添加新的话题桥接，只需在 `multi_bridge_config.yaml` 中添加新的桥接器配置，无需修改任何代码：

1. 编辑配置文件添加新桥接器
2. 重启节点
3. 新话题自动开始桥接

这种配置驱动的方式大大简化了系统扩展和维护。
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
