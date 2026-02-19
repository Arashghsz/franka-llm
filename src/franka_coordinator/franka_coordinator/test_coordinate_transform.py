#!/usr/bin/env python3
"""
Test Coordinate Transformation Pipeline
Verifies pixel -> camera frame -> robot frame transformations

Usage:
    ros2 run franka_coordinator test_coordinate_transform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
from cv_bridge import CvBridge
import json
import sys


class CoordinateTransformTester(Node):
    """Tests coordinate transformation with real camera data"""
    
    def __init__(self):
        super().__init__('coordinate_transform_tester')
        
        self.bridge = CvBridge()
        self.latest_depth = None
        self.camera_info = None
        
        # QoS profile for RealSense (uses BEST_EFFORT)
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/cameras/ee/ee_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            camera_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/cameras/ee/ee_camera/color/camera_info',
            self.camera_info_callback,
            camera_qos
        )
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/target_position',
            self.target_pose_callback,
            10
        )
        
        # Publisher for test VLM grounding
        self.vlm_grounding_pub = self.create_publisher(
            String,
            '/vlm_grounding',
            10
        )
        
        self.get_logger().info('🧪 Coordinate Transform Tester initialized')
        self.get_logger().info('Waiting for camera data...')
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_test)
        self.test_count = 0
    
    def depth_callback(self, msg: Image):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if self.latest_depth.dtype == np.uint16:
                self.latest_depth = self.latest_depth.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(
                f'✅ Camera info received: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}'
            )
    
    def target_pose_callback(self, msg: PoseStamped):
        """Display received target pose"""
        self.get_logger().info(
            f'🎯 TARGET POSE received in frame: {msg.header.frame_id}\\n'
            f'   Position: X={msg.pose.position.x:.3f}m, '
            f'Y={msg.pose.position.y:.3f}m, '
            f'Z={msg.pose.position.z:.3f}m'
        )
    
    def run_test(self):
        """Run a test transformation"""
        if self.latest_depth is None or self.camera_info is None:
            self.get_logger().warn('Still waiting for camera data...')
            return
        
        self.test_count += 1
        
        # Stop after 3 tests
        if self.test_count > 3:
            self.test_timer.cancel()
            self.get_logger().info('✅ Testing complete!')
            return
        
        # Test different pixel locations
        test_cases = [
            # Center of image
            {'name': 'Image Center', 'bbox': [280, 200, 360, 280]},
            # Your red dice position (from docs)
            {'name': 'Red Dice Position', 'bbox': [850, 250, 890, 290]},
            # Random test point
            {'name': 'Random Point', 'bbox': [400, 300, 450, 350]},
        ]
        
        test = test_cases[(self.test_count - 1) % len(test_cases)]
        
        self.get_logger().info(f'\\n========== TEST {self.test_count}: {test["name"]} ==========')
        
        # Simulate VLM grounding message
        grounding_msg = {
            'target': test['name'],
            'bbox': test['bbox'],
            'confidence': 0.95
        }
        
        msg = String()
        msg.data = json.dumps(grounding_msg)
        self.vlm_grounding_pub.publish(msg)
        
        self.get_logger().info(f'📤 Published VLM grounding: {grounding_msg}')
        self.get_logger().info('⏳ Waiting for coordinator to process...')


def main(args=None):
    rclpy.init(args=args)
    
    print('\\n' + '='*60)
    print('  COORDINATE TRANSFORMATION TEST')
    print('='*60)
    print('\\nThis test will:')
    print('  1. Subscribe to camera topics')
    print('  2. Publish fake VLM grounding messages')
    print('  3. Verify coordinator converts to robot frame')
    print('  4. Display target poses on /target_position')
    print('\\nMake sure the following are running:')
    print('  ✓ Camera: ros2 launch realsense_cameras ee_camera.launch.py')
    print('  ✓ Camera TF: ros2 launch realsense_cameras camera_tf.launch.py')
    print('  ✓ Coordinator: ros2 run franka_coordinator coordinator')
    print('\\nStarting test in 3 seconds...')
    print('='*60 + '\\n')
    
    tester = CoordinateTransformTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print('\\n\\n⚠️  Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
