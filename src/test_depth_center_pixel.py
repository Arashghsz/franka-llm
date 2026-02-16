#!/usr/bin/env python3
"""
Test script for sampling depth at center pixel
Tests getting 3D coordinates from the center of the camera image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np


class DepthCenterPixelTester(Node):
    def __init__(self):
        super().__init__('depth_center_pixel_tester')
        
        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.camera_intrinsics = None
        
        # Subscriber for depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/cameras/ee/ee_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        # Subscriber for camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/cameras/ee/ee_camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher for 3D position
        self.position_pub = self.create_publisher(
            PoseStamped,
            '/center_pixel_position',
            10
        )
        
        # Timer to sample and publish center pixel depth
        self.timer = self.create_timer(1.0, self.sample_center_depth)
        
        self.get_logger().info('Depth Center Pixel Tester initialized')
        self.get_logger().info('Subscribing to: /cameras/ee/ee_camera/aligned_depth_to_color/image_raw')
        self.get_logger().info('Publishing to: /center_pixel_position')
    
    def depth_callback(self, msg: Image):
        """Store latest depth image"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert from mm to meters if needed
            if self.latest_depth_image.dtype == np.uint16:
                self.latest_depth_image = self.latest_depth_image.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(f'Camera intrinsics loaded:')
            self.get_logger().info(f'  Resolution: {self.camera_intrinsics["width"]}x{self.camera_intrinsics["height"]}')
            self.get_logger().info(f'  fx: {self.camera_intrinsics["fx"]:.2f}, fy: {self.camera_intrinsics["fy"]:.2f}')
            self.get_logger().info(f'  ppx: {self.camera_intrinsics["ppx"]:.2f}, ppy: {self.camera_intrinsics["ppy"]:.2f}')
    
    def sample_center_depth(self):
        """Sample depth at center pixel and compute 3D position"""
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            if self.latest_depth_image is None:
                self.get_logger().warn('Waiting for depth image...')
            if self.camera_intrinsics is None:
                self.get_logger().warn('Waiting for camera intrinsics...')
            return
        
        try:
            height, width = self.latest_depth_image.shape
            
            # Get center pixel
            center_x = width // 2
            center_y = height // 2
            
            # Get depth at center pixel
            depth_z = float(self.latest_depth_image[center_y, center_x])
            
            if depth_z <= 0:
                self.get_logger().warn(f'Invalid depth at center ({center_x}, {center_y}): {depth_z}')
                return
            
            # Deproject center pixel to 3D coordinates
            fx = self.camera_intrinsics['fx']
            fy = self.camera_intrinsics['fy']
            ppx = self.camera_intrinsics['ppx']
            ppy = self.camera_intrinsics['ppy']
            
            x = (center_x - ppx) * depth_z / fx
            y = (center_y - ppy) * depth_z / fy
            z = depth_z
            
            # Log the result
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Center pixel: ({center_x}, {center_y})')
            self.get_logger().info(f'Depth at center: {depth_z:.3f} m')
            self.get_logger().info(f'3D Position in camera frame:')
            self.get_logger().info(f'  X: {x:>7.3f} m')
            self.get_logger().info(f'  Y: {y:>7.3f} m')
            self.get_logger().info(f'  Z: {z:>7.3f} m')
            self.get_logger().info('=' * 60)
            
            # Publish as PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'ee_d435i_color_optical_frame'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.w = 1.0
            
            self.position_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error sampling center depth: {e}')


def main(args=None):
    rclpy.init(args=args)
    tester = DepthCenterPixelTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
