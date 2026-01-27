#!/usr/bin/env python3
"""
Simple Franka Arm + Gripper Control

This is the SIMPLEST possible interface to move the Franka arm and gripper.
Uses MoveIt2 action clients directly.

Usage:
    from franka_llm_planner.simple_arm import move_arm, open_gripper, close_gripper
    
    # Move arm to xyz position
    move_arm(x=0.4, y=0.0, z=0.4)
    
    # Control gripper
    open_gripper()
    close_gripper()
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from franka_msgs.action import Grasp, Move as GripperMove
import math
import time

# Global node for reuse
_node = None


def _get_node():
    """Get or create the ROS node."""
    global _node
    if _node is None:
        if not rclpy.ok():
            rclpy.init()
        _node = SimpleArmNode()
    return _node


class SimpleArmNode(Node):
    """Simple node for arm and gripper control."""
    
    def __init__(self):
        super().__init__('simple_arm_controller')
        
        # MoveIt action client for arm
        self._move_group_client = ActionClient(
            self, MoveGroup, '/move_action'
        )
        
        # Gripper action clients (FR3 uses /franka_gripper/)
        self._gripper_grasp_client = ActionClient(
            self, Grasp, '/franka_gripper/grasp'
        )
        self._gripper_move_client = ActionClient(
            self, GripperMove, '/franka_gripper/move'
        )
        
        self.get_logger().info('Simple arm controller ready')


def move_arm(x: float, y: float, z: float, 
             roll: float = math.pi, pitch: float = 0.0, yaw: float = 0.0,
             wait: bool = True, timeout: float = 30.0) -> bool:
    """
    Move the arm to a Cartesian position.
    
    Args:
        x, y, z: Position in meters (robot base frame)
        roll, pitch, yaw: Orientation in radians (default: gripper pointing down)
        wait: Wait for motion to complete
        timeout: Max seconds to wait
    
    Returns:
        True if motion succeeded
    
    Example:
        move_arm(0.4, 0.0, 0.4)  # Move to position
        move_arm(0.3, 0.2, 0.5, roll=3.14)  # With orientation
    """
    node = _get_node()
    
    # Wait for action server
    if not node._move_group_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('MoveGroup action server not available!')
        node.get_logger().info('Make sure MoveIt is running:')
        node.get_logger().info('  ros2 launch franka_fr3_moveit_config moveit.launch.py')
        return False
    
    # Build the goal pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'fr3_link0'
    target_pose.header.stamp = node.get_clock().now().to_msg()
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    
    # Convert roll/pitch/yaw to quaternion
    quat = _euler_to_quaternion(roll, pitch, yaw)
    target_pose.pose.orientation.x = quat[0]
    target_pose.pose.orientation.y = quat[1]
    target_pose.pose.orientation.z = quat[2]
    target_pose.pose.orientation.w = quat[3]
    
    # Build motion plan request
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = 'fr3_arm'
    goal_msg.request.num_planning_attempts = 10
    goal_msg.request.allowed_planning_time = 5.0
    goal_msg.request.max_velocity_scaling_factor = 0.1
    goal_msg.request.max_acceleration_scaling_factor = 0.1
    
    # Position constraint
    position_constraint = PositionConstraint()
    position_constraint.header = target_pose.header
    position_constraint.link_name = 'fr3_link8'
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    
    # Bounding region (small sphere at target)
    bounding_volume = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.01]  # 1cm tolerance
    bounding_volume.primitives.append(sphere)
    bounding_volume.primitive_poses.append(target_pose.pose)
    position_constraint.constraint_region = bounding_volume
    position_constraint.weight = 1.0
    
    # Orientation constraint
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = target_pose.header
    orientation_constraint.link_name = 'fr3_link8'
    orientation_constraint.orientation = target_pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    orientation_constraint.weight = 1.0
    
    # Add constraints to goal
    constraints = Constraints()
    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    goal_msg.request.goal_constraints.append(constraints)
    
    # Send goal
    node.get_logger().info(f'Moving arm to: x={x:.3f}, y={y:.3f}, z={z:.3f}')
    
    future = node._move_group_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected!')
        return False
    
    if wait:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
        result = result_future.result()
        
        if result.result.error_code.val == 1:  # SUCCESS
            node.get_logger().info('Motion complete!')
            return True
        else:
            node.get_logger().error(f'Motion failed: {result.result.error_code.val}')
            return False
    
    return True


def move_arm_joints(joint_positions: dict, wait: bool = True) -> bool:
    """
    Move arm by specifying joint angles.
    
    Args:
        joint_positions: Dict of joint_name -> angle (radians)
        wait: Wait for completion
    
    Example:
        move_arm_joints({
            'panda_joint1': 0.0,
            'panda_joint2': -0.785,
            'panda_joint3': 0.0,
            'panda_joint4': -2.356,
            'panda_joint5': 0.0,
            'panda_joint6': 1.571,
            'panda_joint7': 0.785
        })
    """
    node = _get_node()
    
    if not node._move_group_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('MoveGroup action server not available!')
        return False
    
    from moveit_msgs.msg import RobotState, JointConstraint
    from sensor_msgs.msg import JointState
    
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = 'fr3_arm'
    goal_msg.request.num_planning_attempts = 10
    goal_msg.request.allowed_planning_time = 5.0
    goal_msg.request.max_velocity_scaling_factor = 0.1
    goal_msg.request.max_acceleration_scaling_factor = 0.1
    
    constraints = Constraints()
    for joint_name, position in joint_positions.items():
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = position
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    
    goal_msg.request.goal_constraints.append(constraints)
    
    node.get_logger().info(f'Moving to joint positions...')
    
    future = node._move_group_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    if not goal_handle.accepted:
        return False
    
    if wait:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
        return result_future.result().result.error_code.val == 1
    
    return True


def go_home() -> bool:
    """Move arm to home position."""
    return move_arm_joints({
        'fr3_joint1': 0.0,
        'fr3_joint2': -0.785,
        'fr3_joint3': 0.0,
        'fr3_joint4': -2.356,
        'fr3_joint5': 0.0,
        'fr3_joint6': 1.571,
        'fr3_joint7': 0.785
    })


# ============== GRIPPER FUNCTIONS ==============

def open_gripper(width: float = 0.08, speed: float = 0.1) -> bool:
    """
    Open the gripper.
    
    Args:
        width: How wide to open (meters). Max ~0.08m
        speed: Opening speed (m/s)
    
    Returns:
        True if succeeded
    """
    node = _get_node()
    
    if not node._gripper_move_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Gripper move action not available!')
        node.get_logger().info('Make sure Franka bringup is running')
        return False
    
    goal = GripperMove.Goal()
    goal.width = width
    goal.speed = speed
    
    node.get_logger().info(f'Opening gripper to {width*1000:.1f}mm')
    
    future = node._gripper_move_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    if not goal_handle.accepted:
        return False
    
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    
    return result_future.result().result.success


def close_gripper(width: float = 0.0, speed: float = 0.1, 
                  force: float = 50.0) -> bool:
    """
    Close the gripper (grasp).
    
    Args:
        width: Target width (meters). 0.0 = fully closed
        speed: Closing speed (m/s)
        force: Gripping force (N). Max ~70N
    
    Returns:
        True if succeeded
    """
    node = _get_node()
    
    if not node._gripper_grasp_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Gripper grasp action not available!')
        return False
    
    goal = Grasp.Goal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon.inner = 0.01
    goal.epsilon.outer = 0.01
    
    node.get_logger().info(f'Closing gripper with {force:.1f}N force')
    
    future = node._gripper_grasp_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    if not goal_handle.accepted:
        return False
    
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    
    return result_future.result().result.success


def set_gripper(width: float, speed: float = 0.1) -> bool:
    """
    Set gripper to specific width.
    
    Args:
        width: Target width in meters (0.0 to 0.08)
        speed: Movement speed (m/s)
    """
    return open_gripper(width=width, speed=speed)


# ============== HELPER FUNCTIONS ==============

def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]


def shutdown():
    """Clean up ROS resources."""
    global _node
    if _node is not None:
        _node.destroy_node()
        _node = None
    if rclpy.ok():
        rclpy.shutdown()


# ============== MAIN (for testing) ==============

def main():
    """Test the simple arm functions."""
    print("=" * 50)
    print("Simple Arm Control Test")
    print("=" * 50)
    
    try:
        # Go home first
        print("\n1. Going to home position...")
        go_home()
        time.sleep(1)
        
        # Move to a position
        print("\n2. Moving to test position...")
        move_arm(x=0.4, y=0.0, z=0.4)
        time.sleep(1)
        
        # Open gripper
        print("\n3. Opening gripper...")
        open_gripper()
        time.sleep(1)
        
        # Close gripper
        print("\n4. Closing gripper...")
        close_gripper()
        time.sleep(1)
        
        # Return home
        print("\n5. Returning home...")
        go_home()
        
        print("\n✓ All tests complete!")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        shutdown()


if __name__ == '__main__':
    main()
