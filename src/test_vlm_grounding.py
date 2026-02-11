#!/usr/bin/env python3
"""
Test script for VLM grounding functionality
Sends a grounding request and listens for the response
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class VLMGroundingTester(Node):
    def __init__(self):
        super().__init__('vlm_grounding_tester')
        
        # Publisher for VLM grounding requests
        self.vlm_request_pub = self.create_publisher(
            String, '/vlm_request', 10
        )
        
        # Subscriber for VLM grounding results
        self.vlm_grounding_sub = self.create_subscription(
            String, '/vlm_grounding', self.grounding_callback, 10
        )
        
        # Subscriber for target 3D position
        from geometry_msgs.msg import PoseStamped
        self.target_position_sub = self.create_subscription(
            PoseStamped, '/target_position', self.position_callback, 10
        )
        
        self.get_logger().info('VLM Grounding Tester ready')
        self.get_logger().info('Waiting 3 seconds for nodes to connect...')
        time.sleep(3)
    
    def grounding_callback(self, msg: String):
        """Handle VLM grounding response"""
        try:
            grounding = json.loads(msg.data)
            self.get_logger().info('=' * 60)
            self.get_logger().info('VLM Grounding Result:')
            self.get_logger().info(f'  Target: {grounding.get("target", "N/A")}')
            self.get_logger().info(f'  Bbox: {grounding.get("bbox", "N/A")}')
            self.get_logger().info(f'  Confidence: {grounding.get("confidence", "N/A")}')
            self.get_logger().info(f'  Rationale: {grounding.get("rationale", "N/A")}')
            self.get_logger().info('=' * 60)
        except Exception as e:
            self.get_logger().error(f'Error parsing grounding result: {e}')
    
    def position_callback(self, msg):
        """Handle 3D position result"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('3D Target Position:')
        self.get_logger().info(f'  Frame: {msg.header.frame_id}')
        self.get_logger().info(f'  X: {msg.pose.position.x:.3f} m')
        self.get_logger().info(f'  Y: {msg.pose.position.y:.3f} m')
        self.get_logger().info(f'  Z: {msg.pose.position.z:.3f} m')
        self.get_logger().info('=' * 60)
    
    def send_grounding_request(self, target_object):
        """Send a grounding request to VLM"""
        request = {
            'target': target_object,
            'task': 'pick'
        }
        
        msg = String()
        msg.data = json.dumps(request)
        
        self.get_logger().info(f'Sending grounding request for: "{target_object}"')
        self.vlm_request_pub.publish(msg)


def main():
    rclpy.init()
    tester = VLMGroundingTester()
    
    # Send test request
    target = input('Enter target object to locate (e.g., "red cup", "apple"): ')
    if target.strip():
        tester.send_grounding_request(target.strip())
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
