#!/usr/bin/env python3

import http.server
import socketserver
import os
import webbrowser
import argparse
import threading
import time
from pathlib import Path

class WebServer:
    def __init__(self, port=8080, directory=None):
        self.port = port
        self.directory = directory or os.path.dirname(os.path.abspath(__file__))
        self.httpd = None
        
    def start(self, open_browser=True):
        """启动Web服务器"""
        os.chdir(self.directory)
        
        # 创建处理器
        handler = http.server.SimpleHTTPRequestHandler
        
        # 添加CORS支持
        class CORSRequestHandler(handler):
            def end_headers(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                super().end_headers()
                
            def do_OPTIONS(self):
                self.send_response(200)
                self.end_headers()
        
        # 启动服务器
        try:
            self.httpd = socketserver.TCPServer(("", self.port), CORSRequestHandler)
            print(f"🌐 Web服务器已启动")
            print(f"📁 服务目录: {self.directory}")
            print(f"🔗 访问地址: http://localhost:{self.port}")
            print(f"📊 监控面板: http://localhost:{self.port}/index.html")
            print("-" * 60)
            
            if open_browser:
                # 延迟打开浏览器
                threading.Timer(1.0, lambda: webbrowser.open(f'http://localhost:{self.port}')).start()
            
            print("按 Ctrl+C 停止服务器")
            self.httpd.serve_forever()
            
        except OSError as e:
            if e.errno == 48:  # Address already in use
                print(f"❌ 端口 {self.port} 已被占用，请尝试其他端口")
                print(f"   使用 --port 参数指定其他端口，例如: python webserver.py --port 8081")
            else:
                print(f"❌ 启动服务器失败: {e}")
        except KeyboardInterrupt:
            print(f"\n⏹️  服务器已停止")
            self.stop()
            
    def stop(self):
        """停止Web服务器"""
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()

def main():
    parser = argparse.ArgumentParser(description='ROS2 MQTT监控面板 Web服务器')
    parser.add_argument('--port', '-p', type=int, default=8080, help='服务器端口 (默认: 8080)')
    parser.add_argument('--directory', '-d', type=str, help='服务目录 (默认: 当前目录)')
    parser.add_argument('--no-browser', action='store_true', help='不自动打开浏览器')
    
    args = parser.parse_args()
    
    # 确定服务目录
    if args.directory:
        directory = os.path.abspath(args.directory)
    else:
        directory = os.path.dirname(os.path.abspath(__file__))
    
    # 检查目录是否存在
    if not os.path.exists(directory):
        print(f"❌ 目录不存在: {directory}")
        return
    
    # 检查是否有index.html文件
    index_file = os.path.join(directory, 'index.html')
    if not os.path.exists(index_file):
        print(f"❌ 在 {directory} 中找不到 index.html 文件")
        return
    
    print("🚀 正在启动 ROS2 MQTT 监控面板...")
    print(f"📋 使用配置:")
    print(f"   端口: {args.port}")
    print(f"   目录: {directory}")
    print(f"   自动打开浏览器: {'否' if args.no_browser else '是'}")
    print()
    
    # 启动服务器
    server = WebServer(port=args.port, directory=directory)
    server.start(open_browser=not args.no_browser)

if __name__ == '__main__':
    main()
