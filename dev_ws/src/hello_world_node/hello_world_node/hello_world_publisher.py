#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        
        # 创建发布者，发布到 'hello_world' 话题
        self.publisher = self.create_publisher(String, 'hello_world', 10)
        
        # 创建定时器，每2秒调用一次回调函数
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.counter = 0
        self.get_logger().info('Hello World Publisher 节点已启动')

    def timer_callback(self):
        """定时器回调函数，每2秒执行一次"""
        msg = String()
        msg.data = f'Hello World! 计数: {self.counter}'
        
        # 发布消息
        self.publisher.publish(msg)
        
        # 打印日志
        self.get_logger().info(f'发布消息: "{msg.data}"')
        
        self.counter += 1


def main(args=None):
    # 初始化 ROS 2
    rclpy.init(args=args)
    
    # 创建节点
    hello_world_publisher = HelloWorldPublisher()
    
    try:
        # 保持节点运行
        rclpy.spin(hello_world_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        hello_world_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
