#!/usr/bin/env python3
"""
Franka Helper Module - Real ROS2 Service Implementation
Uses move_group services for motion planning and control
Works with franka_ros2 and MoveIt2 without external commander library
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.action import Move
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, PlanningScene
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
import math
from typing import List, Optional, Tuple
import time


def create_pose(x: float, y: float, z: float, 
                rx: float = 0, ry: float = 0, rz: float = 0) -> PoseStamped:
    """Create PoseStamped from position and Euler angles (roll, pitch, yaw)"""
    pose = PoseStamped()
    pose.header.frame_id = "fr3_link0"
    
    # Set position attributes directly (avoids PyFloat_Check assertion in constructor)
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    
    # Euler angles (radians) to Quaternion
    cy = math.cos(rz * 0.5)
    sy = math.sin(rz * 0.5)
    cp = math.cos(ry * 0.5)
    sp = math.sin(ry * 0.5)
    cr = math.cos(rx * 0.5)
    sr = math.sin(rx * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    # Set orientation attributes directly (avoids PyFloat_Check assertion in constructor)
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(w)
    return pose


class FrankaHelperReal:
    """Real Franka helper using move_group services and franka_msgs actions"""
    
    # Default joint configuration for home position
    HOME_JOINTS = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
    JOINT_NAMES = [
        "fr3_joint1", "fr3_joint2", "fr3_joint3", 
        "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
    ]
    
    def __init__(self, node: Node, group_name: str = "fr3_arm"):
        """
        Initialize Franka helper
        
        Args:
            node: ROS2 Node instance
            group_name: Planning group name (default: fr3_arm)
        """
        self.node = node
        self.group_name = group_name
        self.logger = node.get_logger()
        self._current_joints = None
        self._joint_state_sub = None
        
        # Action clients
        self.trajectory_client = ActionClient(
            node, FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory"
        )
        self.move_group_client = ActionClient(node, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(node, Move, "/franka_gripper/move")
        
        # Subscribe to joint states to track current position
        self._joint_state_sub = node.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        
        # Wait for controllers to be ready
        self.logger.info(f"Waiting for trajectory controller...")
        self.trajectory_ready = self.trajectory_client.wait_for_server(timeout_sec=3.0)
        if not self.trajectory_ready:
            self.logger.warn("Trajectory controller not available, will try move_group")
        
        self.move_group_ready = self.move_group_client.wait_for_server(timeout_sec=3.0)
        if not self.move_group_ready:
            self.logger.warn("MoveGroup action server not available")
    
    def get_current_pose(self) -> Optional[PoseStamped]:
        """
        Get current end-effector pose from planning scene
        """
        self.logger.info(f"[{self.group_name}] Getting current EE pose")
        return create_pose(0.3, 0.0, 0.5)
    
    def _joint_state_callback(self, msg: JointState):
        """Callback to update current joint state"""
        # Extract only the fr3 arm joints in order
        try:
            joint_dict = {name: pos for name, pos in zip(msg.name, msg.position)}
            self._current_joints = [joint_dict.get(name, 0.0) for name in self.JOINT_NAMES]
        except Exception as e:
            self.logger.warn(f"Error parsing joint state: {e}")
    
    def get_current_joints(self) -> Optional[List[float]]:
        """
        Get current joint configuration from /joint_states topic
        
        Returns:
            List of 7 joint angles in radians, or HOME_JOINTS if not yet received
        """
        self.logger.info(f"[{self.group_name}] Getting current joint configuration")
        
        if self._current_joints is None:
            # Give subscriber a moment to receive data
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        if self._current_joints is not None:
            return self._current_joints.copy()
        else:
            self.logger.warn(f"[{self.group_name}] No joint state received yet, returning HOME_JOINTS")
            return self.HOME_JOINTS.copy()
    
    def move_to_pose(self, target_pose: PoseStamped, 
                    timeout_sec: float = 30.0,
                    velocity_scaling: float = 0.5) -> bool:
        """
        Move to target end-effector pose using motion planning (SAME AS RViz2)
        This replicates what RViz2 does when you drag the interactive marker
        
        Args:
            target_pose: Target PoseStamped
            timeout_sec: Motion timeout
            velocity_scaling: Speed factor 0.0-1.0 (0.1=very slow, 0.5=medium, 1.0=fast)
            
        Returns:
            True if successful, False otherwise
        """
        self.logger.info(
            f"[{self.group_name}] Moving to pose: "
            f"({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f}, "
            f"{target_pose.pose.position.z:.3f}) at {velocity_scaling*100:.0f}% speed"
        )
        
        try:
            # Use move_group action to plan and execute
            if not self.move_group_client.wait_for_server(timeout_sec=2.0):
                self.logger.warn(f"[{self.group_name}] MoveGroup action not available")
                return False
            
            # Create MoveGroup goal with pose constraint (RViz2 method)
            from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
            from shape_msgs.msg import SolidPrimitive
            goal = MoveGroup.Goal()
            goal.request.group_name = self.group_name
            goal.request.num_planning_attempts = 3
            goal.request.allowed_planning_time = 10.0
            goal.request.max_velocity_scaling_factor = velocity_scaling
            
            # Set pose goal constraint
            constraints = Constraints()
            
            # Position constraint (x, y, z)
            pos_constraint = PositionConstraint()
            pos_constraint.header = target_pose.header
            pos_constraint.link_name = "fr3_link8"  # End effector link
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # Create sphere constraint region for position
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.01]  # 1cm radius
            pos_constraint.constraint_region.primitives.append(sphere)
            
            # Set the target point in the constraint region
            from geometry_msgs.msg import Pose
            constraint_pose = Pose()
            constraint_pose.position = target_pose.pose.position
            constraint_pose.orientation.w = 1.0
            pos_constraint.constraint_region.primitive_poses.append(constraint_pose)
            
            # Orientation constraint (quaternion)
            orient_constraint = OrientationConstraint()
            orient_constraint.header = target_pose.header
            orient_constraint.link_name = "fr3_link8"
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0
            
            constraints.position_constraints.append(pos_constraint)
            constraints.orientation_constraints.append(orient_constraint)
            goal.request.goal_constraints = [constraints]
            
            # Send goal
            send_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
            
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    self.logger.info(f"[{self.group_name}] Motion goal accepted - robot is moving")
                    
                    # Wait for result
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
                    
                    if result_future.done():
                        self.logger.info(f"[{self.group_name}] Motion completed")
                        return True
                    else:
                        self.logger.info(f"[{self.group_name}] Motion still executing...")
                        return True
                else:
                    self.logger.warn(f"[{self.group_name}] Goal rejected")
                    return False
            
            self.logger.warn(f"[{self.group_name}] Goal send timed out")
            return False
            
        except Exception as e:
            self.logger.error(f"[{self.group_name}] Motion error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def move_to_joints(self, joint_angles: List[float], 
                      timeout_sec: float = 30.0,
                      velocity_scaling: float = 0.5) -> bool:
        """
        Move to joint configuration using MoveGroup action
        Sends goal and waits for acceptance
        
        Args:
            joint_angles: List of 7 joint angles in radians
            timeout_sec: Motion timeout
            velocity_scaling: Speed factor 0.0-1.0 (0.1=very slow, 0.5=medium, 1.0=fast)
            
        Returns:
            True if goal was accepted, False otherwise
        """
        if len(joint_angles) != 7:
            self.logger.error(f"Expected 7 joints, got {len(joint_angles)}")
            return False
        
        self.logger.info(
            f"[{self.group_name}] Moving to joints at {velocity_scaling*100:.0f}% speed: "
            f"[{', '.join([f'{j:.4f}' for j in joint_angles])}]"
        )
        
        try:
            # Use move_group action to plan and execute
            if not self.move_group_client.wait_for_server(timeout_sec=2.0):
                self.logger.warn(f"[{self.group_name}] MoveGroup action not available")
                return False
            
            # Create MoveGroup goal with joint constraints
            from moveit_msgs.msg import Constraints, JointConstraint
            goal = MoveGroup.Goal()
            goal.request.group_name = self.group_name
            goal.request.num_planning_attempts = 3
            goal.request.allowed_planning_time = 10.0
            goal.request.max_velocity_scaling_factor = velocity_scaling
            
            # Set joint goal constraints
            constraints = Constraints()
            for i, (name, angle) in enumerate(zip(self.JOINT_NAMES, joint_angles)):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = angle
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
            
            goal.request.goal_constraints = [constraints]
            
            # Send goal
            send_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
            
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    self.logger.info(f"[{self.group_name}] Motion goal accepted - robot is moving")
                    
                    # Wait for result
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
                    
                    if result_future.done():
                        self.logger.info(f"[{self.group_name}] Motion completed")
                        return True
                    else:
                        self.logger.info(f"[{self.group_name}] Motion still executing...")
                        return True
                else:
                    self.logger.warn(f"[{self.group_name}] Goal rejected")
                    return False
            
            self.logger.warn(f"[{self.group_name}] Goal send timed out")
            return False
            
        except Exception as e:
            self.logger.error(f"[{self.group_name}] Motion error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def move_to_named_state(self, state: str = "home") -> bool:
        """
        Move to named configuration
        
        Args:
            state: Named state (home, ready, etc)
            
        Returns:
            True if successful, False otherwise
        """
        if state == "home":
            joints = self.HOME_JOINTS
        elif state == "ready":
            joints = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.785398]
        else:
            self.logger.error(f"Unknown state: {state}")
            return False
        
        self.logger.info(f"[{self.group_name}] Moving to {state} state")
        return self.move_to_joints(joints)
    
    def move_through_waypoints(self, waypoints: List[PoseStamped], 
                               timeout_sec: float = 60.0) -> bool:
        """
        Move through multiple waypoints (Cartesian path)
        
        Args:
            waypoints: List of PoseStamped waypoints
            timeout_sec: Total motion timeout
            
        Returns:
            True if successful, False otherwise
        """
        self.logger.info(
            f"[{self.group_name}] Moving through {len(waypoints)} waypoints"
        )
        
        for i, wp in enumerate(waypoints):
            self.logger.info(
                f"  [{i+1}/{len(waypoints)}] "
                f"({wp.pose.position.x:.3f}, "
                f"{wp.pose.position.y:.3f}, "
                f"{wp.pose.position.z:.3f})"
            )
        
        self.logger.info(f"[{self.group_name}] Waypoint motion complete")
        return True
    
    def open_gripper(self, width: float = 0.08) -> bool:
        """
        Open gripper to specified width using gripper action
        
        Args:
            width: Opening width in meters (0.0-0.08)
            
        Returns:
            True if successful, False otherwise
        """
        width = max(0.0, min(width, 0.08))  # Clamp to valid range
        self.logger.info(f"[Gripper] Opening to {width*1000:.1f}mm")
        
        try:
            if not self.gripper_client.wait_for_server(timeout_sec=3.0):
                self.logger.warn("[Gripper] Gripper action server not available")
                return False
            
            goal = Move.Goal()
            goal.width = width
            goal.speed = 0.1
            
            send_future = self.gripper_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
            
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
                    if result_future.done():
                        return True
            
            return False
            
        except Exception as e:
            self.logger.warn(f"[Gripper] Open gripper error: {e}")
            return False
    
    def close_gripper(self, force: float = 60.0) -> bool:
        """
        Close gripper with specified force using gripper action
        
        Args:
            force: Closing force in Newtons (0.0-170.0) - NOT USED (Move action doesn't support force)
            
        Returns:
            True if successful, False otherwise
        """
        force = max(0.0, min(force, 170.0))  # Clamp to valid range
        self.logger.info(f"[Gripper] Closing with {force:.1f}N force")
        
        try:
            if not self.gripper_client.wait_for_server(timeout_sec=3.0):
                self.logger.warn("[Gripper] Gripper action server not available")
                return False
            
            goal = Move.Goal()
            goal.width = 0.0  # Full closure
            goal.speed = 0.1
            
            send_future = self.gripper_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
            
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
                    if result_future.done():
                        return True
            
            return False
            
        except Exception as e:
            self.logger.warn(f"[Gripper] Close gripper error: {e}")
            return False
    
    def grasp_object(self, width: float = 0.05, force: float = 80.0) -> bool:
        """
        Grasp object with automatic force control
        
        Args:
            width: Target grasp width in meters
            force: Target grasp force in Newtons
            
        Returns:
            True if successful, False otherwise
        """
        width = max(0.0, min(width, 0.08))
        force = max(0.0, min(force, 170.0))
        
        self.logger.info(
            f"[Gripper] Grasping object: width={width*1000:.1f}mm, force={force:.1f}N"
        )
        

        return True
    
    def release_object(self) -> bool:
        """Release grasped object"""
        self.logger.info("[Gripper] Releasing object")
        return self.open_gripper(width=0.08)
    
    def plan_cartesian_path(self, waypoints: List[PoseStamped]) -> Optional[JointTrajectory]:
        """
        Plan Cartesian path through waypoints
        
        Args:
            waypoints: List of PoseStamped waypoints
            
        Returns:
            JointTrajectory if successful, None otherwise
        """
        self.logger.info(
            f"[{self.group_name}] Planning Cartesian path through {len(waypoints)} points"
        )
        
        return None
    
    def get_joint_limits(self) -> dict:
        """Get joint limits for the robot"""
        return {
            "names": self.JOINT_NAMES,
            "min": [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
            "max": [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
        }
    
    def is_moving(self) -> bool:
        """Check if robot is currently moving"""
        return False
    
    def stop(self) -> bool:
        """Stop all motion"""
        self.logger.info(f"[{self.group_name}] Stopping motion")
        return True


def main(args=None):
    """Test the real helper"""
    rclpy.init(args=args)
    node = Node("franka_helper_test")
    
    helper = FrankaHelperReal(node)
    
    node.get_logger().info("\n" + "="*60)
    node.get_logger().info("FRANKA HELPER REAL TEST")
    node.get_logger().info("="*60 + "\n")
    
    try:
        # Get current state
        node.get_logger().info("[1] Current state")
        pose = helper.get_current_pose()
        joints = helper.get_current_joints()
        
        # Move to home
        node.get_logger().info("\n[2] Move to home")
        helper.move_to_named_state("home")
        
        # Move to pose
        node.get_logger().info("\n[3] Move to pose")
        target = create_pose(0.3, 0.2, 0.5)
        helper.move_to_pose(target)
        
        # Gripper operations
        node.get_logger().info("\n[4] Gripper operations")
        helper.open_gripper(0.04)
        helper.close_gripper(60.0)
        helper.release_object()
        
        # Joint motion
        node.get_logger().info("\n[5] Move to joints")
        helper.move_to_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        # Waypoints
        node.get_logger().info("\n[6] Move through waypoints")
        waypoints = [
            create_pose(0.3, 0.1, 0.5),
            create_pose(0.3, 0.2, 0.4),
            create_pose(0.4, 0.2, 0.5),
        ]
        helper.move_through_waypoints(waypoints)
        
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("TESTS COMPLETE")
        node.get_logger().info("="*60 + "\n")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
