#!/bin/bash

# ROS2 MQTT Bridge 项目完整构建脚本
# 适用于 ros2-mqtt-web-main 项目

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
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

print_info "开始构建 ROS2 MQTT Bridge 完整项目..."
print_info "项目路径: /home/seeed/Project/ros2-mqtt-web-main"

# 检查当前目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DEV_WS_PATH="$PROJECT_ROOT/dev_ws"

print_info "脚本目录: $SCRIPT_DIR"
print_info "项目根目录: $PROJECT_ROOT"
print_info "工作空间路径: $DEV_WS_PATH"

# 确保在正确的工作空间目录
if [ ! -d "$DEV_WS_PATH" ]; then
    print_error "工作空间目录不存在: $DEV_WS_PATH"
    exit 1
fi

cd "$DEV_WS_PATH"
print_info "已切换到工作空间: $(pwd)"

# 检查Python环境并处理conda冲突
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    print_warning "检测到conda环境: $CONDA_DEFAULT_ENV"
    print_info "临时禁用conda环境以避免Python版本冲突..."
    conda deactivate || true
fi

# 检查并安装Python依赖
print_info "检查Python依赖..."

# 检查paho-mqtt
python3 -c "import paho.mqtt.client" 2>/dev/null || {
    print_warning "paho-mqtt未安装，正在安装..."
    pip3 install paho-mqtt || {
        print_warning "系统级安装失败，尝试用户目录安装..."
        pip3 install --user paho-mqtt
    }
    print_success "paho-mqtt安装完成"
}

# 检查PyYAML
python3 -c "import yaml" 2>/dev/null || {
    print_warning "PyYAML未安装，正在安装..."
    pip3 install PyYAML || {
        print_warning "系统级安装失败，尝试用户目录安装..."
        pip3 install --user PyYAML
    }
    print_success "PyYAML安装完成"
}

# 设置ROS2环境
print_info "设置ROS2环境..."
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    print_success "ROS2 Foxy环境已加载"
else
    print_error "ROS2 Foxy未找到，请确保已正确安装ROS2"
    exit 1
fi

# 检查colcon工具
if ! command -v colcon &> /dev/null; then
    print_error "colcon构建工具未找到，请安装: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# 清理之前的构建（可选）
if [ "$1" = "--clean" ] || [ "$1" = "-c" ]; then
    print_warning "清理之前的构建文件..."
    rm -rf build/ install/ log/
    print_success "构建文件已清理"
fi

# 检查源码包
print_info "检查源码包..."
if [ ! -d "src/ros_mqtt_bridge_node" ]; then
    print_error "ros_mqtt_bridge_node包未找到"
    exit 1
fi

if [ ! -d "src/hello_world_node" ]; then
    print_error "hello_world_node包未找到"
    exit 1
fi

print_success "所有源码包检查完成"

# 执行构建
print_info "开始构建所有ROS2包..."
print_info "使用系统Python环境进行构建..."

# 构建所有包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查构建结果
BUILD_RESULT=$?
if [ $BUILD_RESULT -eq 0 ]; then
    print_success "所有包构建成功！"
    
    # 创建便捷的环境设置脚本
    cat > setup_env.sh << 'EOF'
#!/bin/bash
# ROS2 MQTT Bridge 环境设置脚本

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 设置工作空间环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"

echo "ROS2 MQTT Bridge 环境已设置完成！"
echo ""
echo "可用的命令："
echo "  ros2 run hello_world_node hello_world_publisher"
echo "  ros2 run hello_world_node hello_world_subscriber"
echo "  ros2 run ros_mqtt_bridge_node mqtt_bridge"
echo "  ros2 run ros_mqtt_bridge_node multi_bridge_manager"
echo ""
echo "可用的启动文件："
echo "  ros2 launch ros_mqtt_bridge_node hello_world_mqtt_bridge.launch.py"
echo "  ros2 launch ros_mqtt_bridge_node multi_bridge_manager.launch.py"
EOF
    chmod +x setup_env.sh
    
    print_success "环境设置脚本已创建: setup_env.sh"
    
    echo ""
    print_info "=========================================="
    print_success "构建完成！"
    print_info "=========================================="
    echo ""
    print_info "下一步操作："
    echo ""
    print_info "1. 设置环境："
    echo "   source setup_env.sh"
    echo ""
    print_info "2. 启动Hello World发布者："
    echo "   ros2 run hello_world_node hello_world_publisher"
    echo ""
    print_info "3. 启动MQTT桥接器："
    echo "   ros2 run ros_mqtt_bridge_node mqtt_bridge"
    echo ""
    print_info "4. 或启动多话题桥接管理器："
    echo "   ros2 run ros_mqtt_bridge_node multi_bridge_manager"
    echo ""
    print_info "5. 启动Web监控界面："
    echo "   cd src/web && python3 webserver.py"
    echo ""
    print_info "6. 使用MQTT客户端监控："
    echo "   mosquitto_sub -h localhost -p 1883 -t 'ros2/#' -v"
    echo ""
    print_info "Web界面地址："
    echo "   - ROS2 MQTT监控: http://localhost:8080/ros2_mqtt_monitor.html"
    echo "   - Hello World监控: http://localhost:8080/hello_world_monitor.html"
    echo ""
    
else
    print_error "构建失败！返回代码: $BUILD_RESULT"
    print_info "请检查上面的错误信息，常见问题："
    print_info "1. 检查ROS2环境是否正确设置"
    print_info "2. 检查Python依赖是否已安装"
    print_info "3. 检查包的package.xml和setup.py文件"
    print_info "4. 尝试清理构建: ./build.sh --clean"
    exit 1
fi
