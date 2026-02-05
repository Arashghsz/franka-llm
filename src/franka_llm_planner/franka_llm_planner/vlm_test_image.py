#!/usr/bin/env python3
"""
Test VLM with image from URL or file
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import requests
from urllib.parse import urlparse
import time


class ImagePublisher(Node):
    """Publish image from URL or file"""

    def __init__(self):
        super().__init__('image_publisher')
        
        self.declare_parameter('image_url', 'https://upload.wikimedia.org/wikipedia/commons/thumb/d/d7/Desktop_computer_clipart_-_Yellow_theme.svg/800px-Desktop_computer_clipart_-_Yellow_theme.svg.png')
        self.declare_parameter('image_path', '')
        
        self.image_url = self.get_parameter('image_url').value
        self.image_path = self.get_parameter('image_path').value
        
        self.bridge = CvBridge()
        
        self.image_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        
        self.analysis_sub = self.create_subscription(
            String,
            '/vlm/image_analysis',
            self.analysis_callback,
            10
        )
        
        self.get_logger().info(f'Image Publisher initialized')
        
        # Publish image after startup
        self.create_timer(2.0, self.publish_image_once)
        self.image_sent = False
        self.analysis_received = False
        self.timeout_timer = self.create_timer(30.0, self.timeout_callback)
    
    def timeout_callback(self):
        """Exit if no analysis received within timeout"""
        if not self.analysis_received:
            self.get_logger().warn('No analysis received within 30 seconds, exiting...')
            raise KeyboardInterrupt()
    
    def publish_image_once(self):
        """Publish image once"""
        if self.image_sent:
            return
        
        self.image_sent = True
        
        try:
            if self.image_path:
                self.get_logger().info(f'Loading image from file: {self.image_path}')
                image = cv2.imread(self.image_path)
            else:
                self.get_logger().info(f'Downloading image from URL: {self.image_url}')
                response = requests.get(self.image_url, timeout=10)
                image_array = np.frombuffer(response.content, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().error('Failed to load image')
                raise KeyboardInterrupt()
            
            self.get_logger().info(f'Image shape: {image.shape}')
            
            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.image_pub.publish(ros_image)
            
            self.get_logger().info('Image published! Waiting for VLM analysis...')
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            raise KeyboardInterrupt()
    
    def analysis_callback(self, msg: String):
        """Print VLM analysis and exit"""
        self.analysis_received = True
        self.get_logger().info(f'\n{"="*80}')
        self.get_logger().info('VLM ANALYSIS:')
        self.get_logger().info(f'{"="*80}')
        self.get_logger().info(msg.data)
        self.get_logger().info(f'{"="*80}\n')
        
        # Exit after receiving analysis
        self.get_logger().info('Analysis complete. Exiting...')
        raise KeyboardInterrupt()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    
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
