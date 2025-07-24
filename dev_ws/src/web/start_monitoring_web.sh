#!/bin/bash

# ROS2 MQTT Web监控面板启动脚本

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}🚀 ROS2 MQTT Web监控面板启动脚本${NC}"
echo "======================================"

# 检查Python是否安装
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 未安装，请先安装Python3${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Python3 已安装${NC}"

# 检查Python依赖
echo -e "${BLUE}🔍 检查Python依赖...${NC}"
python3 -c "import websockets, asyncio, paho.mqtt.client" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Python WebSocket和MQTT依赖已安装${NC}"
else
    echo -e "${YELLOW}⚠️  缺少Python依赖，GPS监控功能可能受影响${NC}"
    echo -e "${BLUE}💡 安装依赖: pip3 install websockets paho-mqtt${NC}"
fi

# 检查端口是否被占用
PORT=8080
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠️  端口 $PORT 已被占用，尝试使用端口 8081${NC}"
    PORT=8081
    if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo -e "${RED}❌ 端口 $PORT 也被占用，请手动指定端口${NC}"
        echo "使用方法: $0 [端口号]"
        exit 1
    fi
fi

# 如果提供了端口参数
if [ ! -z "$1" ]; then
    PORT=$1
    echo -e "${BLUE}🔧 使用指定端口: $PORT${NC}"
fi

# 检查MQTT服务器状态（可选）
echo -e "${BLUE}🔍 检查MQTT服务器状态...${NC}"
if command -v mosquitto_pub &> /dev/null; then
    if mosquitto_pub -h localhost -p 1883 -t "test/connection" -m "test" -W 2 >/dev/null 2>&1; then
        echo -e "${GREEN}✓ MQTT服务器 (localhost:1883) 可达${NC}"
    else
        echo -e "${YELLOW}⚠️  MQTT服务器 (localhost:1883) 不可达，请确保MQTT服务器运行中${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  mosquitto-clients 未安装，跳过MQTT服务器检查${NC}"
fi

# 切换到web目录
cd "$SCRIPT_DIR" || exit 1

echo -e "${BLUE}📂 Web目录: $SCRIPT_DIR${NC}"
echo -e "${BLUE}🌐 启动Web服务器在端口 $PORT...${NC}"
echo ""
echo -e "${GREEN}📱 可用的监控面板:${NC}"
echo -e "   🗺️  GPS实时位置监控:  ${YELLOW}http://localhost:$PORT/gps_monitor.html${NC} ${GREEN}[新增]${NC}"
echo -e "   🎨 Bento风格媒体监控: ${YELLOW}http://localhost:$PORT/media_monitor_bento.html${NC} ${GREEN}[推荐]${NC}"
echo -e "   📊 多话题监控面板:   ${YELLOW}http://localhost:$PORT/ros2_mqtt_monitor.html${NC}"
echo -e "   🏠 监控中心面板:     ${YELLOW}http://localhost:$PORT/index.html${NC}"
echo ""
echo -e "${BLUE}💡 使用说明:${NC}"
echo "   1. 确保MQTT服务器运行在 localhost:1883"
echo "   2. 确保MQTT服务器启用WebSocket支持 (端口9001)"
echo "   3. 确保ROS2 MQTT桥接节点正在运行"
echo ""
echo -e "${BLUE}📋 各监控面板功能:${NC}"
echo -e "   ${YELLOW}🗺️  GPS实时位置监控${NC}: 高德地图实时GPS位置追踪，轨迹显示，地址解析"
echo -e "   ${YELLOW}🎨 Bento风格媒体监控${NC}: Apple风格设计，专注Base64图像解码和视频流显示"
echo -e "   ${YELLOW}📊 多话题监控面板${NC}: 支持多个ROS2话题同时监控，适用于复杂系统"
echo -e "   ${YELLOW}🏠 原始监控面板${NC}: 基础的MQTT消息监控界面"
echo ""
echo -e "${BLUE}🚀 快速测试:${NC}"
echo "   启动GPS MQTT桥接: ros2 run ros_mqtt_bridge_node gps_mqtt_bridge"
echo "   启动测试数据发布器: ros2 run ros_mqtt_bridge_node media_mqtt_bridge"
echo "   查看GPS MQTT消息: mosquitto_sub -h localhost -p 1883 -t 'ros2/gps/#' -v"
echo "   查看媒体MQTT消息: mosquitto_sub -h localhost -p 1883 -t 'ros2/camera/#' -v"
echo ""
echo -e "${BLUE}🔧 检查MQTT WebSocket服务...${NC}"

# 检查MQTT WebSocket端口是否可用
if nc -z localhost 9001 2>/dev/null; then
    echo -e "${GREEN}✓ MQTT WebSocket服务可用 (端口9001)${NC}"
else
    echo -e "${YELLOW}⚠️  MQTT WebSocket服务不可用，GPS监控可能无法正常工作${NC}"
    echo -e "${BLUE}💡 请确保MQTT服务器启用WebSocket支持在端口9001${NC}"
fi

echo ""
echo -e "${RED}按 Ctrl+C 停止Web服务器${NC}"
echo "======================================"

# 启动Python HTTP服务器
python3 -m http.server $PORT
