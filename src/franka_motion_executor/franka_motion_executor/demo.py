#!/usr/bin/env python3
"""
Franka FR3 Robot Motion Control Demo
Edit this file to test different movements
"""

import sys
import os

# CRITICAL: Fix sys.path BEFORE any ROS imports
# When Python runs a script directly, it adds the script's directory to sys.path[0]
# This causes import conflicts with ROS2 C extensions
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Remove script directory and parent directories from sys.path
    paths_to_remove = [
        script_dir,
        os.path.dirname(script_dir),  # franka_motion_executor package dir
        os.path.dirname(os.path.dirname(script_dir))  # src dir
    ]
    for path in paths_to_remove:
        while path in sys.path:
            sys.path.remove(path)
    
    # Add the correct package path if not already there
    package_root = os.path.dirname(os.path.dirname(script_dir))  # src/
    if package_root not in sys.path:
        sys.path.insert(0, package_root)

import rclpy
import time
from rclpy.node import Node
from franka_motion_executor import FrankaHelperReal, FrankaManipulation
import math

rclpy.init()
node = Node("franka_demo")
helper = FrankaHelperReal(node, group_name="fr3_arm")
manip = FrankaManipulation(helper)
new_home = [0.0022672999184578657, -0.7775366306304932, -0.0010969223221763968, -2.3660757541656494, 0.0067384387366473675, 1.5651696920394897, 0.7884734272956848]
start_point = [0.5281668901443481, 0.7094740271568298, 0.022555384784936905, -1.8724782466888428, -0.05140479654073715, 2.5743796825408936, 1.3898661136627197]
destination_point = [-0.3441236913204193, 0.6516157388687134, -0.01045239344239235, -2.0039167404174805, -0.0032070199958980083, 2.6653807163238525, 0.4480981230735779]
# print(math.pi)
print("new cashed code")


# manip.move_to_joints(new_home, velocity_scaling=0.8)
# time.sleep(1)
# manip.open_gripper(width=0.03)
# manip.move_home()
# manip.open_gripper(width=0.08)

# # Move to position - defaults handle gripper-down orientation
# manip.move_to_position(0.282, 0.136, 0.25, velocity_scaling=0.1)
# manip.open_gripper(width=0.03)
# time.sleep(1)
manip.move_to_position(0.6, 0, 0.55, velocity_scaling=0.6)

time.sleep(3)
manip.open_gripper(width=0.08)

manip.move_home()
# time.sleep(1)
# # manip.move_to_position(0.6, 0, 0.14, velocity_scaling=0.1)
# manip.move_home()

# current_joints = helper.get_current_joints()
# print(f"Current joints: {current_joints}")
rclpy.shutdown()
print("\nDone!")
