#!/usr/bin/env python3
"""Task Publisher - Publishes task commands to /task_commands topic."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class TaskPublisher(Node):
    
    def __init__(self):
        super().__init__('task_publisher')
        
        self.publisher = self.create_publisher(String, '/task_commands', 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        
        self.tasks = [
            {"action": "pick", "object": "red_cube", "location": "table"},
            {"action": "place", "object": "red_cube", "location": "bin"},
        ]
        self.task_index = 0
        
        self.get_logger().info('Task Publisher started')
        self.get_logger().info('Publishing to: /task_commands')
    
    def timer_callback(self):
        msg = String()
        task = self.tasks[self.task_index % len(self.tasks)]
        msg.data = json.dumps(task)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.task_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
