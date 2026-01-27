#!/usr/bin/env python3
"""
SIMPLEST Franka Control Script

This is the absolute minimum code to move the Franka arm.
Copy this file and modify for your needs.

Prerequisites:
1. Start the Franka robot:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false

2. Run this script:
   python3 simple_control.py
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from franka_msgs.action import Grasp, Move as GripperMove
import math
import time


class FrankaController(Node):
    def __init__(self):
        super().__init__('simple_franka')
        
        # Arm control via MoveIt
        self.arm = ActionClient(self, MoveGroup, '/move_action')
        
        # Gripper control (FR3 uses /franka_gripper/, Panda uses /panda_gripper/)
        self.gripper_move = ActionClient(self, GripperMove, '/franka_gripper/move')
        self.gripper_grasp = ActionClient(self, Grasp, '/franka_gripper/grasp')
        
        self.get_logger().info('Waiting for MoveIt...')
        self.arm.wait_for_server()
        self.get_logger().info('Ready!')


    def move_joints(self, joints: dict, speed: float = 0.1) -> bool:
        """
        Move arm to joint positions.
        
        Args:
            joints: Dict like {'fr3_joint1': 0.0, 'fr3_joint2': -0.785, ...}
            speed: Velocity scaling (0.0 to 1.0)
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = 'fr3_arm'
        goal.request.max_velocity_scaling_factor = speed
        goal.request.max_acceleration_scaling_factor = speed
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        for name, pos in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(constraints)
        
        self.get_logger().info('Moving joints...')
        future = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=30)
        
        success = result.result().result.error_code.val == 1
        if success:
            self.get_logger().info('Done!')
        else:
            self.get_logger().error('Failed')
        return success


    def home(self):
        """Go to home position."""
        return self.move_joints({
            'fr3_joint1': 0.0,
            'fr3_joint2': -0.785,
            'fr3_joint3': 0.0,
            'fr3_joint4': -2.356,
            'fr3_joint5': 0.0,
            'fr3_joint6': 1.571,
            'fr3_joint7': 0.785
        })


    def open_gripper(self, width: float = 0.08):
        """Open gripper. Max width ~0.08m"""
        if not self.gripper_move.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper not available')
            return False
        
        goal = GripperMove.Goal()
        goal.width = width
        goal.speed = 0.1
        
        self.get_logger().info(f'Opening gripper to {width*100:.0f}cm')
        future = self.gripper_move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=10)
        return result.result().result.success


    def close_gripper(self, force: float = 50.0):
        """Close gripper with force (N). Max ~70N"""
        if not self.gripper_grasp.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper not available')
            return False
        
        goal = Grasp.Goal()
        goal.width = 0.0
        goal.speed = 0.1
        goal.force = force
        goal.epsilon.inner = 0.01
        goal.epsilon.outer = 0.01
        
        self.get_logger().info(f'Closing gripper with {force}N')
        future = self.gripper_grasp.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=10)
        return result.result().result.success


# ============================================================
# SIMPLE FUNCTIONS - Use these in your code!
# ============================================================

_ctrl = None

def _get():
    global _ctrl
    if _ctrl is None:
        if not rclpy.ok():
            rclpy.init()
        _ctrl = FrankaController()
    return _ctrl


def move_arm(joints: dict, speed: float = 0.1):
    """Move arm joints. Example: move_arm({'panda_joint1': 0.5})"""
    return _get().move_joints(joints, speed)


def go_home():
    """Go to home position."""
    return _get().home()


def open_gripper(width: float = 0.08):
    """Open gripper."""
    return _get().open_gripper(width)


def close_gripper(force: float = 50.0):
    """Close/grasp with force."""
    return _get().close_gripper(force)


def cleanup():
    """Clean up ROS. Wait for robot to settle before destroying node."""
    global _ctrl
    if _ctrl:
        # Give the robot time to finish any pending operations
        _ctrl.get_logger().info('Cleaning up... waiting for robot to settle')
        time.sleep(0.5)
        _ctrl.destroy_node()
        _ctrl = None
    if rclpy.ok():
        rclpy.shutdown()


def keep_alive():
    """
    Keep the node alive. Call this if you want the robot to stay in control mode.
    Press Ctrl+C to exit.
    """
    global _ctrl
    if _ctrl:
        print("Node running. Press Ctrl+C to exit.")
        try:
            rclpy.spin(_ctrl)
        except KeyboardInterrupt:
            pass


# ============================================================
# PREDEFINED POSITIONS
# ============================================================

HOME = {
    'fr3_joint1': 0.0,
    'fr3_joint2': -0.785,
    'fr3_joint3': 0.0,
    'fr3_joint4': -2.356,
    'fr3_joint5': 0.0,
    'fr3_joint6': 1.571,
    'fr3_joint7': 0.785
}

READY = {
    'fr3_joint1': 0.0,
    'fr3_joint2': -0.5,
    'fr3_joint3': 0.0,
    'fr3_joint4': -2.0,
    'fr3_joint5': 0.0,
    'fr3_joint6': 1.5,
    'fr3_joint7': 0.785
}


# ============================================================
# TEST
# ============================================================

if __name__ == '__main__':
    print("Franka Simple Control Test")
    print("=" * 40)
    
    try:
        print("\n1. Going home...")
        go_home()
        time.sleep(1)
        
        print("\n2. Opening gripper...")
        open_gripper()
        time.sleep(0.5)
        
        print("\n3. Closing gripper...")
        close_gripper()
        time.sleep(0.5)
        
        print("\n4. Moving joint 1...")
        move_arm({'fr3_joint1': 0.5})
        time.sleep(1)
        
        print("\n5. Returning home...")
        go_home()
        
        print("\n✓ All operations completed successfully!")
        print("Waiting 2 seconds before cleanup...")
        time.sleep(2)  # Let robot fully settle
        
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        cleanup()
        print("Cleanup complete.")
