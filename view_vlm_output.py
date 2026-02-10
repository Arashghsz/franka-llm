#!/usr/bin/env python3
"""Simple script to view full VLM output without truncation"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VLMViewer(Node):
    def __init__(self):
        super().__init__('vlm_viewer')
        self.subscription = self.create_subscription(
            String,
            '/vlm/explanation',
            self.listener_callback,
            10
        )
        print("=" * 80)
        print("VLM OUTPUT VIEWER - Listening to /vlm/explanation")
        print("=" * 80)
        print()
    
    def listener_callback(self, msg):
        print("\n" + "=" * 80)
        print("NEW VLM ANALYSIS:")
        print("=" * 80)
        print(msg.data)
        print("=" * 80)
        print()

def main(args=None):
    rclpy.init(args=args)
    viewer = VLMViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
