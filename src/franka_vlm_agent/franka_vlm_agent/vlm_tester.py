#!/usr/bin/env python3
"""
Quick test script to use VLM image analyzer
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class VLMTester(Node):
    """Simple node to test VLM analyzer"""

    def __init__(self):
        super().__init__('vlm_tester')
        
        self.analysis_sub = self.create_subscription(
            String,
            '/vlm/image_analysis',
            self.analysis_callback,
            10
        )
        
        self.get_logger().info('VLM Tester initialized. Waiting for VLM analysis...')
    
    def analysis_callback(self, msg: String):
        """Print analysis results"""
        self.get_logger().info(f'\n{"="*80}')
        self.get_logger().info('VLM ANALYSIS RESULT:')
        self.get_logger().info(f'{"="*80}')
        self.get_logger().info(msg.data)
        self.get_logger().info(f'{"="*80}\n')


def main(args=None):
    rclpy.init(args=args)
    node = VLMTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shutdown


if __name__ == '__main__':
    main()
