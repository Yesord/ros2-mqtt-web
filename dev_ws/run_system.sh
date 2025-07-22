#!/bin/bash

# ROS2 MQTT Bridge 系统启动脚本
# 启动完整的ROS2-MQTT-Web系统

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

print_info "ROS2 MQTT Bridge 系统启动器"
print_info "=========================================="

# 检查是否已构建
if [ ! -d "install" ]; then
    print_warning "项目尚未构建，正在执行构建..."
    ./build.sh
fi

# 设置环境
print_info "设置ROS2环境..."
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 检查MQTT服务器
print_info "检查MQTT服务器状态..."
if ! systemctl is-active --quiet mosquitto; then
    print_warning "MQTT服务器未运行，尝试启动..."
    sudo systemctl start mosquitto || {
        print_error "无法启动MQTT服务器，请手动检查"
        exit 1
    }
fi
print_success "MQTT服务器运行正常"

# 显示启动选项
echo ""
print_info "请选择启动模式："
echo "1) 完整系统 (Hello World + MQTT Bridge + Web服务器)"
echo "2) 仅MQTT桥接器"
echo "3) 多话题桥接管理器"
echo "4) 仅Web服务器"
echo "5) 测试模式 (逐步启动)"
echo "q) 退出"
echo ""

read -p "请输入选择 [1-5/q]: " choice

case $choice in
    1)
        print_info "启动完整系统..."
        
        # 在后台启动Hello World发布者
        print_info "启动Hello World发布者..."
        gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run hello_world_node hello_world_publisher; exec bash" &
        sleep 2
        
        # 在后台启动MQTT桥接器
        print_info "启动MQTT桥接器..."
        gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run ros_mqtt_bridge_node mqtt_bridge; exec bash" &
        sleep 2
        
        # 启动Web服务器
        print_info "启动Web服务器..."
        gnome-terminal -- bash -c "cd src/web && python3 webserver.py; exec bash" &
        sleep 2
        
        print_success "完整系统已启动！"
        print_info "Web界面: http://localhost:8080"
        print_info "MQTT监控: mosquitto_sub -h localhost -p 1883 -t 'ros2/#' -v"
        ;;
        
    2)
        print_info "启动MQTT桥接器..."
        ros2 run ros_mqtt_bridge_node mqtt_bridge
        ;;
        
    3)
        print_info "启动多话题桥接管理器..."
        ros2 run ros_mqtt_bridge_node multi_bridge_manager
        ;;
        
    4)
        print_info "启动Web服务器..."
        cd src/web
        python3 webserver.py
        ;;
        
    5)
        print_info "测试模式 - 逐步启动组件"
        echo ""
        
        # 步骤1: Hello World发布者
        read -p "启动Hello World发布者? [y/N]: " start_publisher
        if [[ $start_publisher =~ ^[Yy]$ ]]; then
            print_info "启动Hello World发布者..."
            gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run hello_world_node hello_world_publisher; exec bash" &
            sleep 2
        fi
        
        # 步骤2: MQTT桥接器
        read -p "启动MQTT桥接器? [y/N]: " start_bridge
        if [[ $start_bridge =~ ^[Yy]$ ]]; then
            print_info "启动MQTT桥接器..."
            gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run ros_mqtt_bridge_node mqtt_bridge; exec bash" &
            sleep 2
        fi
        
        # 步骤3: Web服务器
        read -p "启动Web服务器? [y/N]: " start_web
        if [[ $start_web =~ ^[Yy]$ ]]; then
            print_info "启动Web服务器..."
            gnome-terminal -- bash -c "cd src/web && python3 webserver.py; exec bash" &
            sleep 2
        fi
        
        # 步骤4: MQTT监控
        read -p "启动MQTT监控终端? [y/N]: " start_monitor
        if [[ $start_monitor =~ ^[Yy]$ ]]; then
            print_info "启动MQTT监控..."
            gnome-terminal -- bash -c "mosquitto_sub -h localhost -p 1883 -t 'ros2/#' -v; exec bash" &
        fi
        
        print_success "测试模式启动完成！"
        ;;
        
    q|Q)
        print_info "退出启动器"
        exit 0
        ;;
        
    *)
        print_error "无效选择"
        exit 1
        ;;
esac

echo ""
print_info "系统启动完成！"
print_info "=========================================="
print_info "有用的命令："
echo "  - 查看ROS2话题: ros2 topic list"
echo "  - 查看话题数据: ros2 topic echo /hello_world"
echo "  - MQTT监控: mosquitto_sub -h localhost -p 1883 -t 'ros2/#' -v"
echo "  - Web界面: http://localhost:8080"
echo ""
print_info "按Ctrl+C停止此脚本"

# 保持脚本运行
while true; do
    sleep 1
done
