#!/usr/bin/env python3
"""
High-level manipulation functions for Franka FR3
Simple functions for common tasks: move, gripper control
"""

from .robot import FrankaHelperReal, create_pose
import math


class FrankaManipulation:
    """Simple manipulation task functions"""
    
    def __init__(self, helper: FrankaHelperReal):
        """
        Args:
            helper: FrankaHelperReal instance
        """
        self.helper = helper
        self.logger = helper.logger
    
    # ============================================================================
    # MOVEMENT FUNCTIONS
    # ============================================================================
    
    def move_to_position(self, x: float, y: float, z: float,
                        rx: float = 0.0, ry: float = math.pi, rz: float = 2.45,
                        velocity_scaling: float = 0.5) -> bool:
        """
        Move end-effector to specific position (Cartesian)
        
        DEFAULT ORIENTATION: Tuned for grasping (rx=0, ry=π, rz=2.45)
        These defaults position the gripper optimally for picking objects.
        
        Args:
            x, y, z: Position in meters (relative to robot base)
            rx, ry, rz: Rotation in radians - DEFAULT: optimal grasp orientation
            velocity_scaling: Speed 0.0-1.0 (0.1=slow, 0.5=medium, 1.0=fast)
        
        Returns:
            True if successful
        
        Example:
            # Move to grasp position (uses optimal defaults)
            move_to_position(0.5, 0.0, 0.15, velocity_scaling=0.1)
        """
        self.logger.info(f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f} at {velocity_scaling*100:.0f}% speed")
        
        pose = create_pose(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
        return self.helper.move_to_pose(pose, velocity_scaling=velocity_scaling)
    
    def move_to_joints(self, angles: list, velocity_scaling: float = 0.5) -> bool:
        """
        Move to joint configuration
        
        Args:
            angles: List of 7 joint angles in radians
            velocity_scaling: Speed 0.0-1.0 (0.1=slow, 0.5=medium, 1.0=fast)
        
        Returns:
            True if successful
        
        Example:
            move_to_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], velocity_scaling=0.2)
        """
        self.logger.info(f"Moving to joint configuration at {velocity_scaling*100:.0f}% speed")
        return self.helper.move_to_joints(angles, velocity_scaling=velocity_scaling)
    
    def move_home(self) -> bool:
        """Move to home position (safe, neutral)"""
        self.logger.info("Moving to HOME")
        return self.helper.move_to_named_state("home")
    
    # ============================================================================
    # GRIPPER FUNCTIONS
    # ============================================================================
    
    def open_gripper(self, width: float = 0.08) -> bool:
        """
        Open gripper
        
        Args:
            width: Opening width in meters (0.0=closed, 0.08=fully open)
        
        Returns:
            True if successful
        
        Example:
            open_gripper()           # Open fully (80mm)
            open_gripper(0.05)       # Open to 50mm
        """
        self.logger.info(f"Opening gripper to {width*1000:.0f}mm")
        return self.helper.open_gripper(width=width)
    
    def close_gripper(self) -> bool:
        """
        Close gripper (full closure)
        
        Returns:
            True if successful
        """
        self.logger.info("Closing gripper")
        return self.helper.close_gripper()

