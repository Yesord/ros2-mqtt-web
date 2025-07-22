# MQTT客户端命令行工具使用指南

## 🛠️ Mosquitto客户端工具

### 基本订阅命令

#### 1. 订阅所有ROS2话题（推荐）
```bash
# 订阅所有ros2相关话题，显示话题名和内容
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -v

# 订阅时显示时间戳
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -v --pretty
```

#### 2. 订阅特定话题
```bash
# 订阅hello_world数据
mosquitto_sub -h localhost -p 1883 -t "ros2/hello_world/data" -v

# 订阅管理器状态
mosquitto_sub -h localhost -p 1883 -t "ros2/multi_bridge/manager_status" -v

# 订阅统计信息
mosquitto_sub -h localhost -p 1883 -t "ros2/multi_bridge/statistics" -v

# 订阅心跳信息
mosquitto_sub -h localhost -p 1883 -t "ros2/multi_bridge/heartbeat" -v
```

#### 3. 订阅多个话题
```bash
# 同时订阅多个特定话题
mosquitto_sub -h localhost -p 1883 -t "ros2/hello_world/data" -t "ros2/multi_bridge/statistics" -v
```

#### 4. 高级订阅选项
```bash
# 设置QoS等级
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -q 1 -v

# 清理会话
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -c -v

# 设置客户端ID
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -i "mqtt_monitor_client" -v

# 保持连接时间
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -k 60 -v
```

### 发布测试消息

#### 1. 发送控制命令
```bash
# 获取统计信息
mosquitto_pub -h localhost -p 1883 -t "ros2/multi_bridge/control" -m '{"command": "get_stats"}'

# 获取桥接器列表
mosquitto_pub -h localhost -p 1883 -t "ros2/multi_bridge/control" -m '{"command": "get_bridge_list"}'

# 重新加载配置
mosquitto_pub -h localhost -p 1883 -t "ros2/multi_bridge/control" -m '{"command": "reload_config"}'

# 重置统计信息
mosquitto_pub -h localhost -p 1883 -t "ros2/multi_bridge/control" -m '{"command": "reset_stats"}'
```

#### 2. 发送测试数据
```bash
# 发送Hello World测试消息
mosquitto_pub -h localhost -p 1883 -t "ros2/hello_world/data" -m "Test message from MQTT client"
```

## 🔧 其他MQTT客户端工具

### 1. mosquitto_rr (请求-响应)
```bash
# 发送命令并等待响应
mosquitto_rr -h localhost -p 1883 -t "ros2/multi_bridge/control" -e "ros2/multi_bridge/response" -m '{"command": "get_stats"}'
```

### 2. MQTT Explorer (图形化工具)
如果需要图形化界面，可以安装MQTT Explorer：
```bash
# 安装MQTT Explorer (需要Node.js)
npm install -g mqtt-explorer

# 或者使用Snap安装
sudo snap install mqtt-explorer
```

## 📊 实用的监控脚本

### 1. 实时监控脚本
```bash
#!/bin/bash
# mqtt_monitor.sh - 实时监控MQTT话题

echo "🔍 开始监控ROS2 MQTT话题..."
echo "按 Ctrl+C 停止监控"
echo "========================================"

mosquitto_sub -h localhost -p 1883 -t "ros2/#" -v | while read line; do
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] $line"
done
```

### 2. 话题统计脚本
```bash
#!/bin/bash
# mqtt_stats.sh - 统计话题消息数量

declare -A topic_count
total_messages=0

echo "📈 统计MQTT话题消息 (60秒)..."

timeout 60 mosquitto_sub -h localhost -p 1883 -t "ros2/#" -v | while read line; do
    topic=$(echo $line | cut -d' ' -f1)
    ((topic_count[$topic]++))
    ((total_messages++))
    
    if [ $((total_messages % 10)) -eq 0 ]; then
        echo "已接收 $total_messages 条消息..."
    fi
done

echo "统计完成！"
for topic in "${!topic_count[@]}"; do
    echo "$topic: ${topic_count[$topic]} 条消息"
done
```

### 3. JSON格式化显示
```bash
# 使用jq格式化JSON消息
mosquitto_sub -h localhost -p 1883 -t "ros2/multi_bridge/statistics" -v | while read topic message; do
    echo "话题: $topic"
    echo "$message" | jq .
    echo "---"
done
```

## 🚨 常见问题解决

### 1. 连接失败
```bash
# 检查MQTT服务器状态
sudo systemctl status mosquitto

# 检查端口是否开放
netstat -an | grep 1883

# 测试连接
mosquitto_sub -h localhost -p 1883 -t '$SYS/broker/version' -C 1
```

### 2. 权限问题
```bash
# 如果需要认证
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -u username -P password -v
```

### 3. 性能优化
```bash
# 限制消息数量
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -C 100 -v

# 只显示最新消息
mosquitto_sub -h localhost -p 1883 -t "ros2/#" -v | tail -f
```

## 📝 快速参考

### 常用参数说明
- `-h`: MQTT服务器地址
- `-p`: MQTT服务器端口
- `-t`: 订阅的话题
- `-v`: 显示话题名称（verbose模式）
- `-q`: QoS等级 (0, 1, 2)
- `-c`: 清理会话
- `-i`: 客户端ID
- `-k`: 保持连接时间
- `-u`: 用户名
- `-P`: 密码
- `-C`: 消息数量限制
- `--pretty`: 格式化输出

### 话题通配符
- `#`: 多级通配符（订阅所有子话题）
- `+`: 单级通配符（订阅单层级话题）

例如：
- `ros2/#`: 订阅所有ros2开头的话题
- `ros2/+/data`: 订阅ros2/*/data格式的话题
- `ros2/multi_bridge/+`: 订阅multi_bridge下的所有直接子话题
