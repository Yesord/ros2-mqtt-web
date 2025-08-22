# 项目介绍
本项目完成搭建 ROS2 桥接 MQTT 服务器的节点，拥有简单的 WEB 监控界面可以读取ROS2 发送的数据。


# 环境要求
- ROS2 Foxy或更高版本
- Python 3.8+
- Mosquitto MQTT Broker
- 现代Web浏览器


# 快速开始

0.**依赖安装**
```bash
# 安装 MQTT 服务器（server机）
sudo apt update && sudo apt install -y mosquitto 

# 安装 MQTT 客户端(client机)
sudo apt update && sudo apt install -y mosquitto-clients

# 安装 Python MQTT 客户端库（兼容版本）
pip3 install paho-mqtt==1.6.1
```

1.**构建项目**
```bash
cd dev_ws
colcon build
source install/setup.bash
```

2. **启动MQTT服务器**
```bash
sudo systemctl start mosquitto
```

3. **运行ROS节点**
```bash
# 终端1: 启动hello world发布者
ros2 run hello_world_node hello_world_publisher
# 终端2: 启动MQTT桥接节点
ros2 run ros_mqtt_bridge_node multi_bridge_manager
# 目前测试使用 ros_domain_id 一致的另一个ROS2上跑bag
# export ROS_DOMAIN_ID=66

```

```bash

# 一键启动 dev_ws目录下
chmod +x ./start_ros_mqtt_node.sh
./start_ros_mqtt_node.sh

```

4. **启动Web监控**
```bash
cd /src/web
chmod +x start_monitoring_web.sh
./start_monitoring_web.sh 
```

5. **访问监控界面**
```bash
打开浏览器访问 [localhost可以替换成你的服务器地址]
http://localhost:8080/index.html
http://localhost:8080/media_monitor_bento.html
http://localhost:8080/ros2_mqtt_monitor.html
http://localhost:8080/hello_world_monitor.html
```

# 参数配置
## MQTT桥接节点配置
参考[./dev_ws/src/ros_mqtt_bridge_node/README.md](./dev_ws/src/ros_mqtt_bridge_node/README.md)
更改[yaml文件](./dev_ws/src/ros_mqtt_bridge_node/config/multi_bridge_config.yaml)
