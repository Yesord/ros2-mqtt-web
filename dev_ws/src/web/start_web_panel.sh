#!/bin/bash

# ROS2 MQTT监控面板启动脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$SCRIPT_DIR"

# 默认配置
DEFAULT_PORT=8080
DEFAULT_MQTT_PORT=9001
DEFAULT_MQTT_HOST="localhost"

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

# 显示帮助信息
show_help() {
    echo "ROS2 MQTT监控面板启动脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -p, --port PORT           Web服务器端口 (默认: $DEFAULT_PORT)"
    echo "  -m, --mqtt-host HOST      MQTT服务器地址 (默认: $DEFAULT_MQTT_HOST)"
    echo "  -mp, --mqtt-port PORT     MQTT服务器端口 (默认: $DEFAULT_MQTT_PORT)"
    echo "  -d, --directory DIR       Web文件目录 (默认: 当前目录)"
    echo "  --no-browser              不自动打开浏览器"
    echo "  --check-deps              检查依赖项"
    echo "  --setup-mosquitto         安装并配置Mosquitto MQTT服务器"
    echo "  -h, --help                显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                        # 使用默认配置启动"
    echo "  $0 -p 8081                # 在8081端口启动Web服务器"
    echo "  $0 -m 192.168.1.100       # 连接到指定MQTT服务器"
    echo "  $0 --setup-mosquitto      # 安装并配置Mosquitto"
    echo ""
}

# 检查依赖项
check_dependencies() {
    print_info "检查依赖项..."
    
    # 检查Python
    if ! command -v python3 &> /dev/null; then
        print_error "Python3 未安装"
        exit 1
    fi
    
    # 检查文件
    if [ ! -f "$WEB_DIR/index.html" ]; then
        print_error "index.html 文件不存在"
        exit 1
    fi
    
    if [ ! -f "$WEB_DIR/webserver.py" ]; then
        print_error "webserver.py 文件不存在"
        exit 1
    fi
    
    print_success "所有依赖项检查通过"
}

# 检查端口是否被占用
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        print_error "端口 $port 已被占用"
        return 1
    fi
    return 0
}

# 安装并配置Mosquitto
setup_mosquitto() {
    print_info "安装并配置Mosquitto MQTT服务器..."
    
    # 检查是否已安装
    if command -v mosquitto &> /dev/null; then
        print_info "Mosquitto 已安装"
    else
        print_info "正在安装Mosquitto..."
        
        # 检测系统类型
        if [ -f /etc/debian_version ]; then
            # Debian/Ubuntu
            sudo apt-get update
            sudo apt-get install -y mosquitto mosquitto-clients
        elif [ -f /etc/redhat-release ]; then
            # CentOS/RHEL
            sudo yum install -y mosquitto
        else
            print_error "不支持的操作系统，请手动安装Mosquitto"
            exit 1
        fi
    fi
    
    # 创建配置文件
    MOSQUITTO_CONFIG="/etc/mosquitto/mosquitto.conf"
    if [ ! -f "$MOSQUITTO_CONFIG" ]; then
        print_info "创建Mosquitto配置文件..."
        sudo tee "$MOSQUITTO_CONFIG" > /dev/null <<EOF
# Mosquitto MQTT 服务器配置
# 用于ROS2 MQTT监控面板

# 基本设置
pid_file /var/run/mosquitto.pid
persistence true
persistence_location /var/lib/mosquitto/
log_dest file /var/log/mosquitto/mosquitto.log

# 监听端口
listener 1883
listener 8083
protocol websockets

# 允许匿名连接（生产环境中应禁用）
allow_anonymous true

# 日志级别
log_type error
log_type warning
log_type notice
log_type information
log_type debug

# 连接限制
max_connections 100
max_inflight_messages 100
max_queued_messages 100
EOF
    fi
    
    # 启动Mosquitto服务
    print_info "启动Mosquitto服务..."
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    
    # 检查服务状态
    if sudo systemctl is-active --quiet mosquitto; then
        print_success "Mosquitto服务已启动"
        print_info "MQTT服务器地址: localhost:1883"
        print_info "WebSocket地址: localhost:8083"
    else
        print_error "Mosquitto服务启动失败"
        exit 1
    fi
}

# 解析命令行参数
PORT=$DEFAULT_PORT
MQTT_HOST=$DEFAULT_MQTT_HOST
MQTT_PORT=$DEFAULT_MQTT_PORT
DIRECTORY=""
NO_BROWSER=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        -m|--mqtt-host)
            MQTT_HOST="$2"
            shift 2
            ;;
        -mp|--mqtt-port)
            MQTT_PORT="$2"
            shift 2
            ;;
        -d|--directory)
            DIRECTORY="$2"
            shift 2
            ;;
        --no-browser)
            NO_BROWSER=true
            shift
            ;;
        --check-deps)
            check_dependencies
            exit 0
            ;;
        --setup-mosquitto)
            setup_mosquitto
            exit 0
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# 主函数
main() {
    print_info "启动 ROS2 MQTT 监控面板..."
    echo ""
    print_info "配置信息:"
    print_info "  Web服务器端口: $PORT"
    print_info "  MQTT服务器: $MQTT_HOST:$MQTT_PORT"
    print_info "  工作目录: $WEB_DIR"
    print_info "  自动打开浏览器: $([ "$NO_BROWSER" = true ] && echo "否" || echo "是")"
    echo ""
    
    # 检查依赖项
    check_dependencies
    
    # 检查端口
    if ! check_port $PORT; then
        print_error "请使用其他端口或停止占用端口 $PORT 的进程"
        exit 1
    fi
    
    # 检查MQTT服务器
    print_info "检查MQTT服务器连接..."
    if ! nc -z "$MQTT_HOST" "$MQTT_PORT" 2>/dev/null; then
        print_warning "无法连接到MQTT服务器 $MQTT_HOST:$MQTT_PORT"
        print_warning "请确保MQTT服务器正在运行，或使用 --setup-mosquitto 安装本地服务器"
    else
        print_success "MQTT服务器连接正常"
    fi
    
    # 设置工作目录
    cd "$WEB_DIR"
    
    # 构建启动参数
    PYTHON_ARGS="webserver.py --port $PORT"
    if [ -n "$DIRECTORY" ]; then
        PYTHON_ARGS="$PYTHON_ARGS --directory $DIRECTORY"
    fi
    if [ "$NO_BROWSER" = true ]; then
        PYTHON_ARGS="$PYTHON_ARGS --no-browser"
    fi
    
    # 显示启动信息
    echo ""
    print_success "准备启动Web服务器..."
    print_info "访问地址: http://localhost:$PORT"
    print_info "在Web界面中配置MQTT服务器为: $MQTT_HOST:$MQTT_PORT"
    echo ""
    print_info "按 Ctrl+C 停止服务器"
    echo ""
    
    # 启动Python Web服务器
    python3 $PYTHON_ARGS
}

# 信号处理
trap 'print_info "正在停止服务器..."; exit 0' SIGINT SIGTERM

# 运行主函数
main
