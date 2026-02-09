#!/usr/bin/env python3
"""
RealSense Camera Hub - Detects and reports connected cameras
"""

import rclpy
from rclpy.node import Node
import subprocess


class CameraHub(Node):
    """Detects and monitors RealSense cameras"""
    
    def __init__(self):
        super().__init__('camera_hub')
        self.get_logger().info("RealSense Camera Hub initialized")
        self.detect_cameras()
    
    def detect_cameras(self):
        """Detect connected RealSense cameras"""
        try:
            result = subprocess.run(
                ['rs-enumerate-devices', '-s'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                self.get_logger().info("Connected RealSense Cameras:\n" + result.stdout)
            else:
                self.get_logger().warn("Could not enumerate cameras")
        except Exception as e:
            self.get_logger().error(f"Camera detection error: {e}")


def main(args=None):
    rclpy.init(args=args)
    hub = CameraHub()
    
    try:
        rclpy.spin(hub)
    except KeyboardInterrupt:
        pass
    finally:
        hub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
