#!/usr/bin/env python3
"""
Franka FR3 Robot Motion Control Demo
Edit this file to test different movements
"""

import rclpy
import time
from rclpy.node import Node
from franka_motion_executor import FrankaHelperReal, FrankaManipulation

rclpy.init()
node = Node("franka_demo")
helper = FrankaHelperReal(node, group_name="fr3_arm")
manip = FrankaManipulation(helper)
start_point = [0.5281668901443481, 0.7094740271568298, 0.022555384784936905, -1.8724782466888428, -0.05140479654073715, 2.5743796825408936, 1.3898661136627197]
destination_point = [-0.3441236913204193, 0.6516157388687134, -0.01045239344239235, -2.0039167404174805, -0.0032070199958980083, 2.6653807163238525, 0.4480981230735779]

manip.move_home()
time.sleep(1)
manip.open_gripper(width=0.08)
time.sleep(2)
manip.move_to_joints(start_point, velocity_scaling=0.8)
time.sleep(1)
manip.open_gripper(width=0.03)
time.sleep(1)
manip.move_home()
time.sleep(2) # time.sleep(2) # manip.move_to_joints(start_point, velocity_scaling=0.5) # time.sleep(2) # manip.open_gripper(width=0.08) # time.sleep(2)
# manip.move_home()
# time.sleep(1)
manip.move_to_joints(destination_point, velocity_scaling=0.1)
time.sleep(1)
manip.open_gripper(width=0.08)
time.sleep(1)
manip.move_home()


# current_joints = helper.get_current_joints()
# print(f"Current joints: {current_joints}")
rclpy.shutdown()
print("\nDone!")
