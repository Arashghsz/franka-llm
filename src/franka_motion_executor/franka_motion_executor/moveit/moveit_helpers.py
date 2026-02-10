#!/usr/bin/env python3
"""
MoveIt2 Helper Functions for Franka FR3
Provides convenient wrappers for common robot operations:
- Move to pose/joints
- Move through waypoints
- Gripper control (open/close/grasp)
- Get current state
- Plan and execute motions
"""

import numpy as np
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_commander.conversions import pose_to_list
from builtin_interfaces.msg import Duration
from franka_msgs.action import Move, Grasp, Homing
from rclpy.action import ActionClient
import math


class FrankaHelper:
    """Helper class for Franka FR3 robot control via MoveIt2"""
    
    def __init__(self, node: Node, group_name: str = "panda_arm", gripper_ns: str = "franka_gripper"):
        """
        Initialize Franka Helper
        
        Args:
            node: ROS2 node
            group_name: MoveIt2 planning group name (default: "panda_arm")
            gripper_ns: Namespace for gripper actions (default: "franka_gripper")
        """
        self.node = node
        self.group_name = group_name
        self.gripper_ns = gripper_ns
        
        # Initialize MoveIt2
        self.move_group = MoveGroupCommander(group_name, node=node)
        self.scene = PlanningSceneInterface(node=node)
        
        # Set planning parameters
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_goal_position_tolerance(0.01)  # 1cm
        self.move_group.set_goal_orientation_tolerance(0.01)  # ~0.5 degrees
        
        # Initialize gripper action clients
        self._init_gripper_actions()
        
    def _init_gripper_actions(self):
        """Initialize gripper action clients"""
        self.gripper_move_client = ActionClient(
            self.node,
            Move,
            f"{self.gripper_ns}/move"
        )
        self.gripper_grasp_client = ActionClient(
            self.node,
            Grasp,
            f"{self.gripper_ns}/grasp"
        )
        self.gripper_homing_client = ActionClient(
            self.node,
            Homing,
            f"{self.gripper_ns}/homing"
        )
        
    def move_to_pose(self, pose: Pose, frame_id: str = "panda_link0") -> bool:
        """
        Move end-effector to a target pose
        
        Args:
            pose: Target pose (geometry_msgs/Pose)
            frame_id: Reference frame (default: "panda_link0")
            
        Returns:
            True if motion executed successfully, False otherwise
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = pose
        
        self.move_group.set_pose_target(pose)
        
        try:
            plan = self.move_group.plan()
            if plan[0]:  # plan[0] is success flag
                success = self.move_group.execute(plan[1])
                return success
            else:
                self.node.get_logger().warn("Failed to plan motion to pose")
                return False
        except Exception as e:
            self.node.get_logger().error(f"Error moving to pose: {e}")
            return False
        finally:
            self.move_group.clear_pose_targets()
    
    def move_to_joints(self, joint_values: List[float]) -> bool:
        """
        Move to target joint configuration
        
        Args:
            joint_values: Target joint positions [j1, j2, j3, j4, j5, j6, j7]
            
        Returns:
            True if motion executed successfully, False otherwise
        """
        if len(joint_values) != 7:
            self.node.get_logger().error("Expected 7 joint values for FR3")
            return False
        
        self.move_group.set_joint_value_target(joint_values)
        
        try:
            plan = self.move_group.plan()
            if plan[0]:
                success = self.move_group.execute(plan[1])
                return success
            else:
                self.node.get_logger().warn("Failed to plan motion to joints")
                return False
        except Exception as e:
            self.node.get_logger().error(f"Error moving to joints: {e}")
            return False
        finally:
            self.move_group.clear_pose_targets()
    
    def move_through_waypoints(
        self,
        waypoints: List[Pose],
        eef_step: float = 0.01,
        jump_threshold: float = 0.0
    ) -> Tuple[bool, float]:
        """
        Move end-effector through a sequence of waypoints (Cartesian path)
        
        Args:
            waypoints: List of target poses
            eef_step: Maximum end-effector step size (meters)
            jump_threshold: Threshold for joint angle jumps (0.0 = disabled)
            
        Returns:
            Tuple of (success: bool, fraction_planned: float)
            fraction_planned indicates % of path successfully planned (0.0-1.0)
        """
        try:
            plan, fraction = self.move_group.compute_cartesian_path(
                waypoints,
                eef_step=eef_step,
                jump_threshold=jump_threshold
            )
            
            if fraction > 0.5:  # Execute if >50% planned
                self.move_group.execute(plan)
                return True, fraction
            else:
                self.node.get_logger().warn(
                    f"Only {fraction*100:.1f}% of Cartesian path planned"
                )
                return False, fraction
                
        except Exception as e:
            self.node.get_logger().error(f"Error in Cartesian motion: {e}")
            return False, 0.0
        finally:
            self.move_group.clear_pose_targets()
    
    def open_gripper(self, width: float = 0.04, speed: float = 0.1) -> bool:
        """
        Open gripper to specified width
        
        Args:
            width: Target width in meters (0-0.04m for FR3)
            speed: Motion speed in m/s
            
        Returns:
            True if action succeeded, False otherwise
        """
        return self._send_gripper_move(width, speed)
    
    def close_gripper(self, speed: float = 0.1) -> bool:
        """
        Close gripper
        
        Args:
            speed: Motion speed in m/s
            
        Returns:
            True if action succeeded, False otherwise
        """
        return self._send_gripper_move(0.0, speed)
    
    def grasp_object(
        self,
        width: float = 0.01,
        force: float = 50.0,
        speed: float = 0.1,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005
    ) -> bool:
        """
        Grasp object with force control
        
        Args:
            width: Target grasp width in meters
            force: Maximum grasping force in Newtons
            speed: Motion speed in m/s
            epsilon_inner: Inner force tolerance in Newtons
            epsilon_outer: Outer force tolerance in Newtons
            
        Returns:
            True if grasp succeeded, False otherwise
        """
        try:
            # Wait for gripper action server
            if not self.gripper_grasp_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().error("Gripper grasp action server not available")
                return False
            
            goal = Grasp.Goal()
            goal.width = width
            goal.speed = speed
            goal.force = force
            goal.epsilon.inner = epsilon_inner
            goal.epsilon.outer = epsilon_outer
            
            future = self.gripper_grasp_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future)
            
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            
            result = result_future.result()
            return result.success
            
        except Exception as e:
            self.node.get_logger().error(f"Error grasping object: {e}")
            return False
    
    def _send_gripper_move(self, width: float, speed: float) -> bool:
        """
        Internal function to send gripper move action
        
        Args:
            width: Target width in meters
            speed: Motion speed in m/s
            
        Returns:
            True if action succeeded, False otherwise
        """
        try:
            # Wait for gripper action server
            if not self.gripper_move_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().error("Gripper move action server not available")
                return False
            
            goal = Move.Goal()
            goal.width = width
            goal.speed = speed
            
            future = self.gripper_move_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future)
            
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            
            result = result_future.result()
            return result.success
            
        except Exception as e:
            self.node.get_logger().error(f"Error moving gripper: {e}")
            return False
    
    def home_gripper(self) -> bool:
        """
        Home the gripper
        
        Returns:
            True if homing succeeded, False otherwise
        """
        try:
            if not self.gripper_homing_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().error("Gripper homing action server not available")
                return False
            
            goal = Homing.Goal()
            
            future = self.gripper_homing_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future)
            
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            
            result = result_future.result()
            return result.success
            
        except Exception as e:
            self.node.get_logger().error(f"Error homing gripper: {e}")
            return False
    
    def get_current_pose(self) -> Optional[Pose]:
        """
        Get current end-effector pose
        
        Returns:
            Current pose (geometry_msgs/Pose) or None if unavailable
        """
        try:
            return self.move_group.get_current_pose().pose
        except Exception as e:
            self.node.get_logger().error(f"Error getting current pose: {e}")
            return None
    
    def get_current_joints(self) -> Optional[List[float]]:
        """
        Get current joint positions
        
        Returns:
            List of 7 joint values or None if unavailable
        """
        try:
            return self.move_group.get_current_joint_values()
        except Exception as e:
            self.node.get_logger().error(f"Error getting current joints: {e}")
            return None
    
    def set_collision_object(
        self,
        object_id: str,
        pose: Pose,
        dimensions: Tuple[float, float, float],
        frame_id: str = "panda_link0"
    ):
        """
        Add collision object to planning scene
        
        Args:
            object_id: Unique identifier for the object
            pose: Object pose
            dimensions: [length, width, height] in meters
            frame_id: Reference frame
        """
        try:
            collision_object = CollisionObject()
            collision_object.id = object_id
            collision_object.header.frame_id = frame_id
            collision_object.operation = CollisionObject.ADD
            
            from shape_msgs.msg import SolidPrimitive
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.BOX
            shape.dimensions = list(dimensions)
            
            collision_object.primitives.append(shape)
            collision_object.primitive_poses.append(pose)
            
            self.scene.add_object(collision_object)
            
        except Exception as e:
            self.node.get_logger().error(f"Error adding collision object: {e}")
    
    def remove_collision_object(self, object_id: str):
        """
        Remove collision object from planning scene
        
        Args:
            object_id: Unique identifier of the object to remove
        """
        try:
            self.scene.remove_world_object(object_id)
        except Exception as e:
            self.node.get_logger().error(f"Error removing collision object: {e}")
    
    def set_velocity_scaling(self, scale: float):
        """
        Scale velocity (0.0-1.0)
        
        Args:
            scale: Velocity scaling factor (0.0-1.0)
        """
        if 0.0 <= scale <= 1.0:
            self.move_group.set_max_velocity_scaling_factor(scale)
        else:
            self.node.get_logger().warn("Velocity scale must be between 0.0 and 1.0")
    
    def set_acceleration_scaling(self, scale: float):
        """
        Scale acceleration (0.0-1.0)
        
        Args:
            scale: Acceleration scaling factor (0.0-1.0)
        """
        if 0.0 <= scale <= 1.0:
            self.move_group.set_max_acceleration_scaling_factor(scale)
        else:
            self.node.get_logger().warn("Acceleration scale must be between 0.0 and 1.0")
    
    def move_to_named_state(self, state_name: str) -> bool:
        """
        Move to a predefined named state (from SRDF)
        
        Args:
            state_name: Name of the predefined state (e.g., "home", "ready")
            
        Returns:
            True if motion executed successfully, False otherwise
        """
        try:
            self.move_group.set_named_target(state_name)
            plan = self.move_group.plan()
            if plan[0]:
                success = self.move_group.execute(plan[1])
                return success
            return False
        except Exception as e:
            self.node.get_logger().error(f"Error moving to named state '{state_name}': {e}")
            return False
        finally:
            self.move_group.clear_pose_targets()


# Utility functions for common operations

def create_pose(
    x: float,
    y: float,
    z: float,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 0.0
) -> Pose:
    """
    Create a Pose from xyz and Euler angles (radians)
    
    Args:
        x, y, z: Position in meters
        rx, ry, rz: Rotation in radians (roll, pitch, yaw)
        
    Returns:
        geometry_msgs/Pose object
    """
    from scipy.spatial.transform import Rotation
    
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    
    quat = Rotation.from_euler('xyz', [rx, ry, rz]).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    
    return pose


def create_pose_from_quat(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float
) -> Pose:
    """
    Create a Pose from xyz and quaternion
    
    Args:
        x, y, z: Position in meters
        qx, qy, qz, qw: Quaternion components
        
    Returns:
        geometry_msgs/Pose object
    """
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    
    return pose
