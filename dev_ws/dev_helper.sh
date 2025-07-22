#!/bin/bash

# ROS2 MQTT Bridge 开发助手脚本
# 提供常用的开发和调试命令

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

print_info "ROS2 MQTT Bridge 开发助手"
print_info "=========================================="

# 显示菜单
show_menu() {
    echo ""
    print_info "请选择操作："
    echo "1) 构建项目 (完整构建)"
    echo "2) 清理构建 (删除build/install/log)"
    echo "3) 快速构建 (仅修改的包)"
    echo "4) 运行测试"
    echo "5) 启动开发环境"
    echo "6) 查看系统状态"
    echo "7) MQTT调试工具"
    echo "8) 查看日志"
    echo "9) 环境诊断"
    echo "0) 一键演示"
    echo "q) 退出"
    echo ""
}

# 构建项目
build_project() {
    print_info "开始构建项目..."
    ./build.sh
}

# 清理构建
clean_build() {
    print_warning "将删除所有构建文件..."
    read -p "确认删除? [y/N]: " confirm
    if [[ $confirm =~ ^[Yy]$ ]]; then
        rm -rf build/ install/ log/
        print_success "构建文件已清理"
    fi
}

# 快速构建
quick_build() {
    print_info "执行快速构建..."
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install --packages-up-to ros_mqtt_bridge_node hello_world_node
}

# 运行测试
run_tests() {
    print_info "运行项目测试..."
    source /opt/ros/foxy/setup.bash
    if [ -d "install" ]; then
        source install/setup.bash
    fi
    
    # 测试配置文件
    if [ -f "src/ros_mqtt_bridge_node/ros_mqtt_bridge_node/test_multi_bridge.py" ]; then
        python3 src/ros_mqtt_bridge_node/ros_mqtt_bridge_node/test_multi_bridge.py
    else
        print_warning "测试文件未找到"
    fi
}

# 启动开发环境
start_dev_env() {
    print_info "启动开发环境..."
    
    # 在新终端中启动各个组件
    gnome-terminal --tab --title="ROS2 Core" -- bash -c "
        source /opt/ros/foxy/setup.bash
        echo 'ROS2 Core终端已就绪'
        echo '可用命令:'
        echo '  ros2 topic list'
        echo '  ros2 node list'
        echo '  ros2 run hello_world_node hello_world_publisher'
        exec bash
    " &
    
    gnome-terminal --tab --title="MQTT Monitor" -- bash -c "
        echo 'MQTT监控终端'
        echo '使用以下命令监控MQTT消息:'
        echo '  mosquitto_sub -h localhost -p 1883 -t \"ros2/#\" -v'
        exec bash
    " &
    
    gnome-terminal --tab --title="Web Server" -- bash -c "
        cd src/web
        echo 'Web服务器目录'
        echo '启动命令: python3 webserver.py'
        exec bash
    " &
    
    print_success "开发环境已启动"
}

# 查看系统状态
check_status() {
    print_info "检查系统状态..."
    
    echo ""
    print_info "ROS2进程:"
    ps aux | grep -E "(ros2|colcon)" | grep -v grep || echo "无ROS2进程运行"
    
    echo ""
    print_info "MQTT服务器状态:"
    systemctl is-active mosquitto && print_success "MQTT服务器运行中" || print_warning "MQTT服务器未运行"
    
    echo ""
    print_info "网络端口状态:"
    netstat -tlnp 2>/dev/null | grep -E "(1883|8080)" || echo "相关端口未监听"
    
    echo ""
    print_info "Python包状态:"
    python3 -c "import paho.mqtt.client; print('paho-mqtt: OK')" 2>/dev/null || print_warning "paho-mqtt未安装"
    python3 -c "import yaml; print('PyYAML: OK')" 2>/dev/null || print_warning "PyYAML未安装"
}

# MQTT调试工具
mqtt_debug() {
    print_info "MQTT调试工具"
    echo ""
    echo "1) 订阅所有ROS2话题"
    echo "2) 发送测试消息"
    echo "3) 查看MQTT服务器状态"
    echo "4) 重启MQTT服务器"
    echo "b) 返回主菜单"
    echo ""
    
    read -p "请选择: " mqtt_choice
    
    case $mqtt_choice in
        1)
            print_info "订阅所有ROS2话题 (按Ctrl+C停止)..."
            mosquitto_sub -h localhost -p 1883 -t "ros2/#" -v
            ;;
        2)
            print_info "发送测试消息到 ros2/test/message..."
            mosquitto_pub -h localhost -p 1883 -t "ros2/test/message" -m "Test message from debug tool"
            print_success "测试消息已发送"
            ;;
        3)
            systemctl status mosquitto
            ;;
        4)
            print_warning "重启MQTT服务器..."
            sudo systemctl restart mosquitto
            print_success "MQTT服务器已重启"
            ;;
        b)
            return
            ;;
        *)
            print_error "无效选择"
            ;;
    esac
}

# 查看日志
view_logs() {
    print_info "查看构建和运行日志"
    echo ""
    echo "1) 查看最新构建日志"
    echo "2) 查看colcon日志"
    echo "3) 查看MQTT服务器日志"
    echo "4) 查看系统日志"
    echo "b) 返回主菜单"
    echo ""
    
    read -p "请选择: " log_choice
    
    case $log_choice in
        1)
            if [ -d "log/latest_build" ]; then
                find log/latest_build -name "*.log" -exec echo "=== {} ===" \; -exec cat {} \;
            else
                print_warning "未找到构建日志"
            fi
            ;;
        2)
            if [ -d "log" ]; then
                ls -la log/
                echo ""
                read -p "输入要查看的日志目录: " log_dir
                if [ -d "log/$log_dir" ]; then
                    find "log/$log_dir" -name "*.log" -exec echo "=== {} ===" \; -exec cat {} \;
                fi
            else
                print_warning "未找到colcon日志目录"
            fi
            ;;
        3)
            sudo journalctl -u mosquitto -f
            ;;
        4)
            journalctl --since "1 hour ago" | grep -E "(ros|mqtt)"
            ;;
        b)
            return
            ;;
        *)
            print_error "无效选择"
            ;;
    esac
}

# 环境诊断
diagnose_env() {
    print_info "环境诊断"
    echo ""
    
    print_info "ROS2环境:"
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        print_success "ROS2 Foxy已安装"
        source /opt/ros/foxy/setup.bash
        echo "ROS_DISTRO: $ROS_DISTRO"
    else
        print_error "ROS2 Foxy未找到"
    fi
    
    echo ""
    print_info "Python环境:"
    python3 --version
    python3 -c "import sys; print(f'Python路径: {sys.executable}')"
    
    echo ""
    print_info "工作空间检查:"
    [ -d "src" ] && print_success "src目录存在" || print_error "src目录不存在"
    [ -d "src/ros_mqtt_bridge_node" ] && print_success "ros_mqtt_bridge_node包存在" || print_error "ros_mqtt_bridge_node包不存在"
    [ -d "src/hello_world_node" ] && print_success "hello_world_node包存在" || print_error "hello_world_node包不存在"
    [ -d "src/web" ] && print_success "web目录存在" || print_error "web目录不存在"
    
    echo ""
    print_info "构建工具:"
    command -v colcon >/dev/null && print_success "colcon已安装" || print_error "colcon未安装"
    
    echo ""
    print_info "MQTT工具:"
    command -v mosquitto_pub >/dev/null && print_success "mosquitto客户端已安装" || print_error "mosquitto客户端未安装"
}

# 一键演示
demo_run() {
    print_info "一键演示 - 启动完整系统"
    
    # 构建项目
    print_info "1/5 构建项目..."
    ./build.sh
    
    # 设置环境
    print_info "2/5 设置环境..."
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    
    # 启动MQTT服务器
    print_info "3/5 启动MQTT服务器..."
    sudo systemctl start mosquitto
    
    # 启动组件
    print_info "4/5 启动系统组件..."
    
    # Hello World发布者
    gnome-terminal --tab --title="Hello World Publisher" -- bash -c "
        source /opt/ros/foxy/setup.bash
        source install/setup.bash
        sleep 3
        ros2 run hello_world_node hello_world_publisher
        exec bash
    " &
    
    # MQTT桥接器
    gnome-terminal --tab --title="MQTT Bridge" -- bash -c "
        source /opt/ros/foxy/setup.bash
        source install/setup.bash
        sleep 5
        ros2 run ros_mqtt_bridge_node mqtt_bridge
        exec bash
    " &
    
    # Web服务器
    gnome-terminal --tab --title="Web Server" -- bash -c "
        cd src/web
        sleep 7
        python3 webserver.py
        exec bash
    " &
    
    # MQTT监控
    gnome-terminal --tab --title="MQTT Monitor" -- bash -c "
        sleep 10
        mosquitto_sub -h localhost -p 1883 -t 'ros2/#' -v
        exec bash
    " &
    
    print_info "5/5 系统启动完成!"
    
    sleep 12
    print_success "演示系统已启动!"
    print_info "Web界面: http://localhost:8080"
    print_info "查看各个终端标签了解系统运行状态"
}

# 主循环
while true; do
    show_menu
    read -p "请输入选择: " choice
    
    case $choice in
        1) build_project ;;
        2) clean_build ;;
        3) quick_build ;;
        4) run_tests ;;
        5) start_dev_env ;;
        6) check_status ;;
        7) mqtt_debug ;;
        8) view_logs ;;
        9) diagnose_env ;;
        0) demo_run ;;
        q|Q) 
            print_info "退出开发助手"
            exit 0
            ;;
        *)
            print_error "无效选择，请重新输入"
            ;;
    esac
    
    echo ""
    read -p "按Enter继续..."
done
