#!/usr/bin/env python3
"""
SIMPLEST Franka FR3 Control

Just two functions:
- move_to(x, y, z) - Move end effector to position
- gripper(open/close) - Control gripper

Usage:
    python3 simple_move.py
    
Or import:
    from simple_move import move_to, gripper
    move_to(0.4, 0.0, 0.4)
    gripper('close')
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from franka_msgs.action import Grasp, Move as GripperMove
import math

_node = None


def _init():
    global _node
    if _node is None:
        if not rclpy.ok():
            rclpy.init()
        _node = Node('simple_move')
        _node.arm = ActionClient(_node, MoveGroup, '/move_action')
        _node.grip_open = ActionClient(_node, GripperMove, '/franka_gripper/move')
        _node.grip_close = ActionClient(_node, Grasp, '/franka_gripper/grasp')
        _node.arm.wait_for_server(timeout_sec=10.0)
    return _node


def move_to(x: float, y: float, z: float, speed: float = 0.1) -> bool:
    """
    Move end effector to position (x, y, z) in meters.
    Gripper points down by default.
    
    Args:
        x, y, z: Position in meters (robot base frame)
        speed: 0.0 to 1.0 (default 0.1 = slow/safe)
    
    Returns:
        True if successful
    
    Example:
        move_to(0.4, 0.0, 0.4)
        move_to(0.3, 0.2, 0.5, speed=0.2)
    """
    node = _init()
    
    # Build goal
    goal = MoveGroup.Goal()
    goal.request.group_name = 'fr3_arm'
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = speed
    goal.request.max_acceleration_scaling_factor = speed
    
    # Target pose
    pose = PoseStamped()
    pose.header.frame_id = 'fr3_link0'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # Gripper pointing down (180° around X)
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    
    # Position constraint
    pos_constraint = PositionConstraint()
    pos_constraint.header = pose.header
    pos_constraint.link_name = 'fr3_link8'
    bv = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.01]
    bv.primitives.append(sphere)
    bv.primitive_poses.append(pose.pose)
    pos_constraint.constraint_region = bv
    pos_constraint.weight = 1.0
    
    # Orientation constraint
    ori_constraint = OrientationConstraint()
    ori_constraint.header = pose.header
    ori_constraint.link_name = 'fr3_link8'
    ori_constraint.orientation = pose.pose.orientation
    ori_constraint.absolute_x_axis_tolerance = 0.1
    ori_constraint.absolute_y_axis_tolerance = 0.1
    ori_constraint.absolute_z_axis_tolerance = 0.1
    ori_constraint.weight = 1.0
    
    constraints = Constraints()
    constraints.position_constraints.append(pos_constraint)
    constraints.orientation_constraints.append(ori_constraint)
    goal.request.goal_constraints.append(constraints)
    
    # Send
    print(f"Moving to ({x}, {y}, {z})...")
    future = node.arm.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    
    handle = future.result()
    if not handle or not handle.accepted:
        print("Goal rejected!")
        return False
    
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=60)
    
    success = result.result().result.error_code.val == 1
    print("Done!" if success else "Failed!")
    return success


def gripper(action: str, force: float = 50.0) -> bool:
    """
    Control gripper.
    
    Args:
        action: 'open' or 'close'
        force: Grip force in N (only for close, max ~70)
    
    Returns:
        True if successful
    
    Example:
        gripper('open')
        gripper('close')
        gripper('close', force=30)
    """
    node = _init()
    
    if action == 'open':
        if not node.grip_open.wait_for_server(timeout_sec=2.0):
            print("Gripper not available")
            return False
        
        goal = GripperMove.Goal()
        goal.width = 0.08  # Fully open
        goal.speed = 0.1
        
        print("Opening gripper...")
        future = node.grip_open.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        
        handle = future.result()
        if not handle or not handle.accepted:
            return False
        
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=10)
        success = result.result().result.success
        
    elif action == 'close':
        if not node.grip_close.wait_for_server(timeout_sec=2.0):
            print("Gripper not available")
            return False
        
        goal = Grasp.Goal()
        goal.width = 0.0
        goal.speed = 0.1
        goal.force = force
        goal.epsilon.inner = 0.01
        goal.epsilon.outer = 0.01
        
        print(f"Closing gripper ({force}N)...")
        future = node.grip_close.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        
        handle = future.result()
        if not handle or not handle.accepted:
            return False
        
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=10)
        success = result.result().result.success
    else:
        print("Use 'open' or 'close'")
        return False
    
    print("Done!" if success else "Failed!")
    return success


def cleanup():
    global _node
    if _node:
        _node.destroy_node()
        _node = None
    if rclpy.ok():
        rclpy.shutdown()


# ============================================================
# TEST
# ============================================================

if __name__ == '__main__':
    try:
        print("=== Simple Move Test ===\n")
        
        # Move to position
        move_to(0.4, 0.0, 0.4)
        
        # Open gripper
        gripper('open')
        
        # Close gripper
        gripper('close')
        
        # Move to another position
        move_to(0.4, 0.2, 0.3)
        
        # Back to center
        move_to(0.4, 0.0, 0.4)
        
        print("\n✓ All done!")
        
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        cleanup()
