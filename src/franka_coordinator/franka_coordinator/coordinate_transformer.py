#!/usr/bin/env python3
"""
Coordinate Transformation Utility for Franka LLM Pipeline
Based on friend's coordinate_computer.py implementation
Handles pixel → 3D camera frame → robot base frame transformations
"""

import numpy as np
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class CoordinateTransformer:
    """
    Transforms coordinates between pixel space, camera frame, and robot frame.
    
    Usage:
        transformer = CoordinateTransformer(node, robot_frame='panda_link0', camera_frame='camera_color_optical_frame')
        point_robot = transformer.pixel_to_robot_frame(pixel_x=320, pixel_y=240, depth_m=0.5)
    """
    
    def __init__(self, 
                 node: Node,
                 robot_frame: str = 'fr3_link0',
                 camera_frame: str = 'ee_d435i_color_optical_frame'):
        """
        Initialize coordinate transformer.
        
        Args:
            node: ROS 2 node for logging and TF
            robot_frame: Robot base frame name (default: fr3_link0 for Franka FR3)
            camera_frame: Camera optical frame name (default: ee_d435i_color_optical_frame)
        """
        self.node = node
        self.robot_frame = robot_frame
        self.camera_frame = camera_frame
        
        # Camera intrinsics (will be set from CameraInfo)
        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None
        self.width = None
        self.height = None
        
        # TF2 for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        self.node.get_logger().info(
            f'Coordinate Transformer initialized: {camera_frame} → {robot_frame}'
        )
    
    def update_camera_intrinsics(self, camera_info: CameraInfo):
        """
        Update camera intrinsics from CameraInfo message.
        
        Args:
            camera_info: sensor_msgs/CameraInfo message
        """
        self.fx = camera_info.k[0]
        self.fy = camera_info.k[4]
        self.ppx = camera_info.k[2]
        self.ppy = camera_info.k[5]
        self.width = camera_info.width
        self.height = camera_info.height
        
        self.node.get_logger().info(
            f'Camera intrinsics updated: fx={self.fx:.1f}, fy={self.fy:.1f}, '
            f'ppx={self.ppx:.1f}, ppy={self.ppy:.1f}, size={self.width}x{self.height}'
        )
    
    def pixel_to_camera_frame(self, 
                              pixel_u: int, 
                              pixel_v: int, 
                              depth_m: float) -> Optional[np.ndarray]:
        """
        Convert pixel coordinates + depth to 3D point in camera frame.
        Uses pinhole camera model (same as pyrealsense2's rs2_deproject_pixel_to_point).
        
        Args:
            pixel_u: Horizontal pixel coordinate (x, column)
            pixel_v: Vertical pixel coordinate (y, row)
            depth_m: Depth in meters
            
        Returns:
            3D point [x, y, z] in camera frame (meters), or None if invalid
        """
        if None in [self.fx, self.fy, self.ppx, self.ppy]:
            self.node.get_logger().error('Camera intrinsics not set! Call update_camera_intrinsics() first')
            return None
        
        if depth_m <= 0 or np.isnan(depth_m) or np.isinf(depth_m):
            self.node.get_logger().warn(f'Invalid depth value: {depth_m}')
            return None
        
        # Clamp pixel coordinates to image bounds
        pixel_u = max(0, min(pixel_u, self.width - 1))
        pixel_v = max(0, min(pixel_v, self.height - 1))
        
        # Pinhole camera model (Brown-Conrady for RealSense)
        # This matches rs2.rs2_deproject_pixel_to_point() behavior
        x_cam = (pixel_u - self.ppx) * depth_m / self.fx
        y_cam = (pixel_v - self.ppy) * depth_m / self.fy
        z_cam = depth_m
        
        point_cam = np.array([x_cam, y_cam, z_cam], dtype=np.float64)
        
        self.node.get_logger().debug(
            f'Pixel ({pixel_u}, {pixel_v}) @ {depth_m:.3f}m → '
            f'Camera frame: [{x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}]'
        )
        
        return point_cam
    
    def camera_frame_to_robot_frame(self, 
                                     point_camera: np.ndarray,
                                     timeout_sec: float = 1.0) -> Optional[np.ndarray]:
        """
        Transform 3D point from camera frame to robot base frame using TF2.
        
        Args:
            point_camera: 3D point [x, y, z] in camera frame (meters)
            timeout_sec: TF lookup timeout (default: 1.0 second)
            
        Returns:
            3D point [x, y, z] in robot frame (meters), or None if transform unavailable
        """
        if point_camera is None or len(point_camera) != 3:
            return None
        
        try:
            # Create PointStamped in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = self.node.get_clock().now().to_msg()
            point_stamped.point.x = float(point_camera[0])
            point_stamped.point.y = float(point_camera[1])
            point_stamped.point.z = float(point_camera[2])
            
            # Lookup transform (camera → robot)
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.camera_frame,
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )
            
            # Apply transform
            point_robot_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            point_robot = np.array([
                point_robot_stamped.point.x,
                point_robot_stamped.point.y,
                point_robot_stamped.point.z
            ], dtype=np.float64)
            
            self.node.get_logger().debug(
                f'Camera [{point_camera[0]:.3f}, {point_camera[1]:.3f}, {point_camera[2]:.3f}] → '
                f'Robot [{point_robot[0]:.3f}, {point_robot[1]:.3f}, {point_robot[2]:.3f}]'
            )
            
            return point_robot
            
        except Exception as e:
            self.node.get_logger().error(f'TF lookup failed: {e}')
            self.node.get_logger().error(
                f'Make sure camera TF is published! Check: ros2 run tf2_ros tf2_echo {self.robot_frame} {self.camera_frame}'
            )
            return None
    
    def pixel_to_robot_frame(self,
                             pixel_u: int,
                             pixel_v: int,
                             depth_m: float,
                             timeout_sec: float = 1.0) -> Optional[np.ndarray]:
        """
        One-step conversion: pixel + depth → robot frame coordinates.
        
        Args:
            pixel_u: Horizontal pixel coordinate
            pixel_v: Vertical pixel coordinate
            depth_m: Depth in meters
            timeout_sec: TF lookup timeout
            
        Returns:
            3D point [x, y, z] in robot frame, or None if conversion fails
        """
        # Step 1: Pixel → Camera frame
        point_camera = self.pixel_to_camera_frame(pixel_u, pixel_v, depth_m)
        if point_camera is None:
            return None
        
        # Step 2: Camera frame → Robot frame
        point_robot = self.camera_frame_to_robot_frame(point_camera, timeout_sec)
        
        if point_robot is not None:
            self.node.get_logger().info(
                f'✅ Pixel ({pixel_u}, {pixel_v}) @ {depth_m:.3f}m → '
                f'Robot frame: X={point_robot[0]:.3f}m, Y={point_robot[1]:.3f}m, Z={point_robot[2]:.3f}m'
            )
        
        return point_robot
    
    def bbox_to_robot_frame(self,
                            bbox: Tuple[int, int, int, int],
                            depth_image: np.ndarray,
                            method: str = 'median') -> Optional[np.ndarray]:
        """
        Convert bounding box + depth image to 3D point in robot frame.
        Uses bbox centroid and robust depth estimation.
        
        Args:
            bbox: Bounding box [x1, y1, x2, y2] in pixels
            depth_image: Depth image array (float32, meters)
            method: Depth estimation method ('median', 'min', 'center')
            
        Returns:
            3D point [x, y, z] in robot frame, or None if invalid
        """
        x1, y1, x2, y2 = bbox
        
        # Get bbox centroid
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        
        # Clamp bbox to image bounds
        height, width = depth_image.shape
        cy = max(0, min(cy, height - 1))
        cx = max(0, min(cx, width - 1))
        y1 = max(0, min(int(y1), height - 1))
        y2 = max(0, min(int(y2), height - 1))
        x1 = max(0, min(int(x1), width - 1))
        x2 = max(0, min(int(x2), width - 1))
        
        # Extract depth from ROI
        roi = depth_image[y1:y2, x1:x2]
        valid_depths = roi[roi > 0]  # Filter invalid (zero) depths
        
        if len(valid_depths) == 0:
            self.node.get_logger().warn('No valid depth values in bounding box ROI')
            return None
        
        # Robust depth estimation
        if method == 'median':
            depth_m = float(np.median(valid_depths))
        elif method == 'min':
            depth_m = float(np.min(valid_depths))
        elif method == 'center':
            depth_m = float(depth_image[cy, cx])
            if depth_m <= 0:
                depth_m = float(np.median(valid_depths))  # Fallback
        else:
            depth_m = float(np.median(valid_depths))
        
        self.node.get_logger().info(
            f'Bbox [{x1}, {y1}, {x2}, {y2}] → Centroid ({cx}, {cy}), '
            f'Depth: {depth_m:.3f}m ({method})'
        )
        
        # Convert to robot frame
        return self.pixel_to_robot_frame(cx, cy, depth_m)
    
    def robot_frame_to_camera_frame(self,
                                     point_robot: np.ndarray,
                                     timeout_sec: float = 1.0) -> Optional[np.ndarray]:
        """
        Inverse transform: robot frame → camera frame.
        
        Args:
            point_robot: 3D point in robot frame
            timeout_sec: TF lookup timeout
            
        Returns:
            3D point in camera frame, or None if unavailable
        """
        if point_robot is None or len(point_robot) != 3:
            return None
        
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.robot_frame
            point_stamped.header.stamp = self.node.get_clock().now().to_msg()
            point_stamped.point.x = float(point_robot[0])
            point_stamped.point.y = float(point_robot[1])
            point_stamped.point.z = float(point_robot[2])
            
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )
            
            point_camera_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            return np.array([
                point_camera_stamped.point.x,
                point_camera_stamped.point.y,
                point_camera_stamped.point.z
            ], dtype=np.float64)
            
        except Exception as e:
            self.node.get_logger().error(f'Inverse TF lookup failed: {e}')
            return None
