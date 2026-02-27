#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time
from datetime import datetime
from franka_motion_executor import FrankaHelperReal, FrankaManipulation


class MotionExecutorNode(Node):
    
    def __init__(self):
        super().__init__('motion_executor')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.get_logger().info('Initializing Franka robot interface')
        self.helper = FrankaHelperReal(self, group_name="fr3_arm")
        self.manip = FrankaManipulation(self.helper)
        
        self.latest_target_position = None
        self.current_action = None
        self.target_object = None
        
        self.motion_command_sub = self.create_subscription(
            String, '/motion/command', self.handle_motion_command, 10,
            callback_group=self.callback_group
        )
        
        self.target_position_sub = self.create_subscription(
            PoseStamped, '/target_position', self.handle_target_position, 10,
            callback_group=self.callback_group
        )
        
        self.status_pub = self.create_publisher(String, '/motion/status', 10)
        
        self.get_logger().info('Motion Executor initialized')
    
    def handle_target_position(self, msg: PoseStamped):
        self.latest_target_position = msg
        
        self.get_logger().info(
            f'Received target position: X={msg.pose.position.x:.3f}m, '
            f'Y={msg.pose.position.y:.3f}m, '
            f'Z={msg.pose.position.z:.3f}m'
        )
        
        if self.current_action == 'pick' and self.target_object:
            self.execute_pick_object()
        elif self.current_action == 'place' and self.target_object:
            self.execute_place_object()
    
    def handle_motion_command(self, msg: String):
        try:
            command = json.loads(msg.data)
            action = command.get('action', '')
            parameters = command.get('parameters', {})
            
            self.get_logger().info(f'Motion command: {action}')
            
            self.current_action = action
            self.target_object = parameters.get('object', 'unknown object')
            
            if action == 'pick':
                self.handle_pick_request(parameters)
            elif action == 'place':
                self.handle_place_request(parameters)
            elif action == 'handover':
                self.handle_handover_request(parameters)
            else:
                self.get_logger().warn(f'Unknown action: {action}')
                self.publish_status('failed', f'Unknown action: {action}')
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
            self.publish_status('failed', 'Invalid command format')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.publish_status('failed', str(e))
    
    def handle_pick_request(self, parameters: dict):
        object_name = parameters.get('object', 'unknown object')
        
        self.get_logger().info(f'Pick request: {object_name}')
        
        if self.latest_target_position:
            self.execute_pick_object()
        else:
            self.get_logger().info('Waiting for target position')
    
    def execute_pick_object(self):
        if not self.latest_target_position:
            self.get_logger().error('No target position available')
            self.publish_status('failed', 'No target position')
            return
        
        target = self.latest_target_position
        object_name = self.target_object or 'object'
        
        self.get_logger().info(f'Executing pick: {object_name}')
        self.get_logger().info(f'Position: X={target.pose.position.x:.4f}m, '
                              f'Y={target.pose.position.y:.4f}m, '
                              f'Z={target.pose.position.z:.4f}m')
        
        try:
            self.publish_status('executing', f'Picking {object_name}')
            
            self._pick_at_position(
                x=target.pose.position.x,
                y=target.pose.position.y,
                z=target.pose.position.z
            )
            
            self.publish_status('completed', f'Successfully picked {object_name}')
            self.get_logger().info(f'Pick completed: {object_name}')
            
            self.current_action = None
            self.latest_target_position = None
            
        except Exception as e:
            self.get_logger().error(f'Pick failed: {e}')
            self.publish_status('failed', str(e))
    
    def _pick_at_position(self, x: float, y: float, z: float):
        self.get_logger().info('Step 1: Moving to position above object')
        self.manip.move_to_position(x + 0.01, y, 0.6, velocity_scaling=0.1)
        time.sleep(1.0)
        
        self.get_logger().info('Step 2: Opening gripper')
        self.manip.open_gripper(width=0.08)
        time.sleep(1.0)
        
        self.get_logger().info('Step 3: Moving down to grasp')
        self.manip.move_to_position(x + 0.01, y, 0.14, velocity_scaling=0.1)
        time.sleep(1.0)
        
        self.get_logger().info('Step 4: Closing gripper')
        self.manip.open_gripper(width=0.03)
        time.sleep(1.0)
        
        self.get_logger().info('Step 5: Moving to home')
        self.manip.move_home()
        time.sleep(1.0)
    
    def handle_place_request(self, parameters: dict):
        location = parameters.get('location', 'unknown location')
        
        self.get_logger().info(f'Place request: {location}')
        
        if self.latest_target_position:
            self.execute_place_object()
        else:
            self.get_logger().info('Waiting for target position')
    
    def handle_handover_request(self, parameters: dict):
        object_name = parameters.get('object', 'object')
        
        self.get_logger().info(f'Handover request: {object_name}')
        self.target_object = object_name
        self.current_action = 'handover'
        
        # First pick the object if we don't have it yet
        if self.latest_target_position:
            self.get_logger().info('Target position available, executing handover sequence')
            # If we have a position, this is the hand position for delivery
            # We assume object is already picked (two-step process)
            self.execute_handover()
        else:
            self.get_logger().info('Waiting for hand position from VLM')
    
    def execute_place_object(self):
        if not self.latest_target_position:
            self.get_logger().error('No target position available')
            self.publish_status('failed', 'No target position')
            return
        
        target = self.latest_target_position
        location_name = self.target_object or 'location'
        
        self.get_logger().info(f'Executing place: {location_name}')
        self.get_logger().info(f'Position: X={target.pose.position.x:.4f}m, '
                              f'Y={target.pose.position.y:.4f}m, '
                              f'Z={target.pose.position.z:.4f}m')
        
        try:
            self.publish_status('executing', f'Placing at {location_name}')
            
            self._place_at_position(
                x=target.pose.position.x,
                y=target.pose.position.y,
                z=target.pose.position.z
            )
            
            self.publish_status('completed', f'Successfully placed at {location_name}')
            self.get_logger().info(f'Place completed: {location_name}')
            
            self.current_action = None
            self.latest_target_position = None
            
        except Exception as e:
            self.get_logger().error(f'Place failed: {e}')
            self.publish_status('failed', str(e))
    
    def _place_at_position(self, x: float, y: float, z: float):
        self.get_logger().info('Step 1: Moving to position above placement location')
        self.manip.move_to_position(x, y, 0.6, velocity_scaling=0.1)
        time.sleep(1.0)
        
        self.get_logger().info(f'Step 2: Moving down to placement height (Z={z:.3f}m)')
        # Use actual Z coordinate from target position (includes object height for stacking)
        self.manip.move_to_position(x, y, z, velocity_scaling=0.1)
        time.sleep(1.0)
        
        self.get_logger().info('Step 3: Opening gripper to release')
        self.manip.open_gripper(width=0.05)
        time.sleep(1.0)
        
        self.get_logger().info('Step 4: Moving back up')
        self.manip.move_to_position(x, y, 0.6, velocity_scaling=0.1)
        time.sleep(1.0)
        
        self.get_logger().info('Step 5: Moving to home')
        self.manip.move_home()
        time.sleep(1.0)
    
    def execute_handover(self):
        """Execute handover: bring object to detected hand position (Z=0.30m fixed)"""
        if not self.latest_target_position:
            self.get_logger().error('No hand position available')
            self.publish_status('failed', 'No hand position')
            return
        
        target = self.latest_target_position
        object_name = self.target_object or 'object'
        
        self.get_logger().info(f'Executing handover: {object_name}')
        self.get_logger().info(f'Hand position: X={target.pose.position.x:.4f}m, '
                              f'Y={target.pose.position.y:.4f}m, '
                              f'Z={target.pose.position.z:.4f}m (fixed at 0.30m)')
        
        try:
            self.publish_status('executing', f'Delivering {object_name} to hand')
            
            # Move to handover position
            self.get_logger().info('Step 1: Moving to position above hand')
            self.manip.move_to_position(
                target.pose.position.x,
                target.pose.position.y,
                0.6,
                velocity_scaling=0.1
            )
            time.sleep(1.0)
            
            self.get_logger().info(f'Step 2: Moving down to handover height (Z={target.pose.position.z:.3f}m)')
            self.manip.move_to_position(
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,  # Z=0.30m from coordinator
                velocity_scaling=0.1
            )
            time.sleep(1.0)
            
            self.get_logger().info('Step 3: Waiting for user to take object (5 seconds)')
            time.sleep(2.5)  # Wait for user to grasp
            
            self.get_logger().info('Step 4: Opening gripper to release')
            self.manip.open_gripper(width=0.08)
            time.sleep(1.0)
            
            self.get_logger().info('Step 5: Moving back up')
            self.manip.move_to_position(
                target.pose.position.x,
                target.pose.position.y,
                0.6,
                velocity_scaling=0.1
            )
            time.sleep(1.0)
            
            self.get_logger().info('Step 6: Moving to home')
            self.manip.move_home()
            time.sleep(1.0)
            
            self.publish_status('completed', f'Successfully handed over {object_name}')
            self.get_logger().info(f'Handover completed: {object_name}')
            
            self.current_action = None
            self.latest_target_position = None
            
        except Exception as e:
            self.get_logger().error(f'Handover failed: {e}')
            self.publish_status('failed', str(e))
    
    def publish_status(self, status: str, message: str = ''):
        status_msg = {
            'timestamp': datetime.now().isoformat(),
            'status': status,
            'message': message,
            'action': self.current_action,
            'object': self.target_object
        }
        
        self.status_pub.publish(String(data=json.dumps(status_msg)))


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
