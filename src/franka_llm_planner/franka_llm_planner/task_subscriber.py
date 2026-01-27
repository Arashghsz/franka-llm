#!/usr/bin/env python3
"""Task Subscriber - Receives and executes task commands."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class TaskSubscriber(Node):
    
    def __init__(self):
        super().__init__('task_subscriber')
        
        self.subscription = self.create_subscription(
            String, '/task_commands', self.task_callback, 10)
        
        self.get_logger().info('Task Subscriber started')
        self.get_logger().info('Listening on: /task_commands')
    
    def task_callback(self, msg: String):
        try:
            task = json.loads(msg.data)
            action = task.get('action', 'unknown')
            obj = task.get('object', 'unknown')
            location = task.get('location', 'unknown')
            self.get_logger().info(f'Received: {action} {obj} at {location}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
