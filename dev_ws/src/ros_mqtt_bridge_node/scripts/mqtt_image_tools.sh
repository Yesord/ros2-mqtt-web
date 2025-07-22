#!/bin/bash
# MQTT 图像查看工具启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== MQTT 图像查看工具 ==="
echo "1. 查看图像数据（可视化显示）"
echo "2. 监控 MQTT 话题数据"
echo "3. 列出当前活跃的 MQTT 话题"
echo "4. 直接查看 Base64 图像数据"
echo ""

read -p "请选择功能 (1-4): " choice

case $choice in
    1)
        echo "启动图像查看器..."
        read -p "输入 MQTT 话题 (默认: ros2/camera/image): " topic
        topic=${topic:-"ros2/camera/image"}
        
        read -p "是否自动保存图像? (y/n, 默认: n): " save_images
        save_flag=""
        if [[ "$save_images" == "y" || "$save_images" == "Y" ]]; then
            save_flag="--save"
        fi
        
        python3 "$SCRIPT_DIR/mqtt_image_viewer.py" --topic "$topic" $save_flag
        ;;
    2)
        echo "启动 MQTT 监控器..."
        read -p "输入要监控的话题 (默认: ros2/+/+): " topics
        topics=${topics:-"ros2/+/+"}
        
        python3 "$SCRIPT_DIR/mqtt_monitor.py" --topics $topics
        ;;
    3)
        echo "查看当前活跃的 MQTT 话题..."
        timeout 5 mosquitto_sub -h localhost -t '+' -v | head -20
        ;;
    4)
        echo "查看 Base64 图像数据..."
        read -p "输入图像话题 (默认: ros2/camera/image): " topic
        topic=${topic:-"ros2/camera/image"}
        
        echo "订阅话题: $topic"
        echo "按 Ctrl+C 退出"
        mosquitto_sub -h localhost -t "$topic" -v
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
