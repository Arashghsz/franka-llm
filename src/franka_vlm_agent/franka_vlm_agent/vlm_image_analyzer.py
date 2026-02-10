#!/usr/bin/env python3
"""
VLM Image Analyzer Node
Subscribes to camera image and provides explanations using a Vision Language Model (LLaVA/VILA)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import requests
import base64
import json
from pathlib import Path


class VLMImageAnalyzer(Node):
    """ROS2 Node for VLM-based image analysis"""

    def __init__(self):
        super().__init__('vlm_image_analyzer')
        
        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('vlm_endpoint', 'http://localhost:11434/api/generate')
        self.declare_parameter('vlm_model', 'llava:7b')
        self.declare_parameter('save_images', False)
        
        self.image_topic = self.get_parameter('image_topic').value
        self.vlm_endpoint = self.get_parameter('vlm_endpoint').value
        self.vlm_model = self.get_parameter('vlm_model').value
        self.save_images = self.get_parameter('save_images').value
        
        # Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        self.analysis_pub = self.create_publisher(
            String,
            '/vlm/image_analysis',
            10
        )
        
        self.get_logger().info(
            f'VLM Image Analyzer initialized\n'
            f'  Image topic: {self.image_topic}\n'
            f'  VLM endpoint: {self.vlm_endpoint}\n'
            f'  VLM model: {self.vlm_model}'
        )
        
        self.last_analysis_time = self.get_clock().now().nanoseconds
        self.min_interval_ns = 2 * 1_000_000_000  # 2 seconds minimum between analyses
        
    def image_callback(self, msg: Image):
        """Process incoming image"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Rate limit to avoid overwhelming the VLM
            current_time = self.get_clock().now().nanoseconds
            if current_time - self.last_analysis_time < self.min_interval_ns:
                return
            
            self.last_analysis_time = current_time
            
            # Save image for debugging if enabled
            if self.save_images:
                Path('/tmp/vlm_images').mkdir(exist_ok=True)
                cv2.imwrite(f'/tmp/vlm_images/{current_time}.jpg', cv_image)
            
            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Call VLM
            self.get_logger().info('Sending image to VLM for analysis...')
            analysis = self.query_vlm(image_base64)
            
            if analysis:
                # Publish analysis
                msg_out = String()
                msg_out.data = analysis
                self.analysis_pub.publish(msg_out)
                self.get_logger().info(f'Analysis published: {analysis[:100]}...')
            else:
                self.get_logger().warn('Failed to get analysis from VLM')
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
    
    def query_vlm(self, image_base64: str) -> str:
        """Query the VLM with an image"""
        try:
            payload = {
                'model': self.vlm_model,
                'prompt': 'Explain what you see in this image. Be concise and practical.',
                'images': [image_base64],
                'stream': False,
                'temperature': 0.7,
            }
            
            response = requests.post(
                self.vlm_endpoint,
                json=payload,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                return result.get('response', 'No response from VLM')
            else:
                self.get_logger().error(f'VLM API error: {response.status_code}')
                return None
                
        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                f'Cannot connect to VLM endpoint: {self.vlm_endpoint}\n'
                f'Make sure Ollama is running: ollama serve'
            )
            return None
        except Exception as e:
            self.get_logger().error(f'Error querying VLM: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = VLMImageAnalyzer()
    
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
