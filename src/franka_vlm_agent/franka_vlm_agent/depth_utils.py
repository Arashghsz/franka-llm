#!/usr/bin/env python3
"""
Depth utilities for 3D position calculation
Handles depth image processing and deprojection
"""

import numpy as np
from typing import Optional, Dict


class DepthProcessor:
    """Processes depth images and calculates 3D positions"""
    
    def __init__(self):
        self.camera_intrinsics = None
        self.latest_depth_image = None
    
    def update_intrinsics(self, camera_info_msg):
        """
        Update camera intrinsics from CameraInfo message
        
        Args:
            camera_info_msg: sensor_msgs/CameraInfo message
        """
        self.camera_intrinsics = {
            'fx': camera_info_msg.k[0],
            'fy': camera_info_msg.k[4],
            'ppx': camera_info_msg.k[2],
            'ppy': camera_info_msg.k[5],
            'width': camera_info_msg.width,
            'height': camera_info_msg.height
        }
    
    def update_depth_image(self, depth_array: np.ndarray):
        """
        Update latest depth image
        
        Args:
            depth_array: Depth image array (float32, in meters)
        """
        self.latest_depth_image = depth_array
    
    def get_3d_position(self, pixel_x: int, pixel_y: int) -> Optional[Dict]:
        """
        Get 3D position from pixel coordinates using depth
        
        Args:
            pixel_x: X pixel coordinate
            pixel_y: Y pixel coordinate
            
        Returns:
            {
                'position': [x, y, z],  # in meters
                'pixel': [pixel_x, pixel_y],
                'depth': depth_value
            } or None if unavailable
        """
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            return None
        
        try:
            # Ensure pixel coordinates are within bounds
            height, width = self.latest_depth_image.shape
            pixel_x = max(0, min(pixel_x, width - 1))
            pixel_y = max(0, min(pixel_y, height - 1))
            
            # Get depth at pixel
            depth_z = float(self.latest_depth_image[pixel_y, pixel_x])
            
            if depth_z <= 0 or np.isnan(depth_z) or np.isinf(depth_z):
                return None
            
            # Deproject to 3D using pinhole camera model
            fx = self.camera_intrinsics['fx']
            fy = self.camera_intrinsics['fy']
            ppx = self.camera_intrinsics['ppx']
            ppy = self.camera_intrinsics['ppy']
            
            x = (pixel_x - ppx) * depth_z / fx
            y = (pixel_y - ppy) * depth_z / fy
            z = depth_z
            
            return {
                'position': [x, y, z],
                'pixel': [pixel_x, pixel_y],
                'depth': depth_z
            }
        except Exception:
            return None
    
    def get_center_3d_position(self) -> Optional[Dict]:
        """
        Get 3D position of image center pixel
        
        Returns:
            3D position dict or None
        """
        if self.latest_depth_image is None:
            return None
        
        height, width = self.latest_depth_image.shape
        center_x = width // 2
        center_y = height // 2
        
        return self.get_3d_position(center_x, center_y)
