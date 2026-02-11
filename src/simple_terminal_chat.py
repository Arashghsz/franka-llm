#!/usr/bin/env python3
"""
Simple terminal chat for LLM → VLM → Depth pipeline
No web UI, just command line interaction

Usage:
    python3 simple_terminal_chat.py

Example commands:
    - "pick up the red cup"
    - "locate the apple"
    - "find the screwdriver"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import sys


class SimpleChatNode(Node):
    def __init__(self):
        super().__init__('simple_chat')
        
        # Publishers
        self.user_command_pub = self.create_publisher(String, '/user_command', 10)
        self.vlm_request_pub = self.create_publisher(String, '/vlm_request', 10)
        
        # Subscribers
        self.llm_response_sub = self.create_subscription(
            String, '/llm_response', self.llm_callback, 10
        )
        self.vlm_grounding_sub = self.create_subscription(
            String, '/vlm_grounding', self.grounding_callback, 10
        )
        self.vlm_explanation_sub = self.create_subscription(
            String, '/vlm/explanation', self.explanation_callback, 10
        )
        self.target_position_sub = self.create_subscription(
            PoseStamped, '/target_position', self.position_callback, 10
        )
        
        self.use_llm = True  # Toggle to use LLM or go direct to VLM
        self.get_logger().info('Simple Chat Terminal Ready')
        self.get_logger().info(f'Mode: {"LLM → VLM" if self.use_llm else "Direct VLM"}')
        
    def llm_callback(self, msg: String):
        """Handle LLM response and extract target for VLM"""
        try:
            response = json.loads(msg.data)
            print(f'\n[LLM] Understood: {response.get("understood", False)}')
            
            tasks = response.get('tasks', [])
            if tasks:
                task = tasks[0]  # Handle first task
                action = task.get('action', '')
                target = task.get('object', '')
                
                print(f'[LLM] Action: {action}, Target: {target}')
                
                # Handle inspection/observation - just describe the scene
                if action in ['inspect', 'look', 'observe', 'see', 'describe']:
                    print(f'[System] Requesting scene description from VLM...')
                    # Trigger scene analysis by requesting analysis
                    # VLM will publish to /vlm/explanation
                    analyze_request = {'type': 'scene_description', 'query': target if target else 'table'}
                    msg = String()
                    msg.data = json.dumps(analyze_request)
                    self.vlm_request_pub.publish(msg)
                    
                elif target:
                    # Grounding request - locate specific object
                    print(f'[System] Requesting VLM to locate "{target}"...')
                    vlm_request = {'target': target, 'task': action}
                    
                    msg = String()
                    msg.data = json.dumps(vlm_request)
                    self.vlm_request_pub.publish(msg)
                else:
                    print('[LLM] No target object identified')
            else:
                print(f'[LLM] Response: {msg.data}')
                
        except json.JSONDecodeError:
            print(f'[LLM] {msg.data}')
    
    def grounding_callback(self, msg: String):
        """Handle VLM grounding result"""
        try:
            grounding = json.loads(msg.data)
            target = grounding.get('target', 'N/A')
            bbox = grounding.get('bbox', None)
            confidence = grounding.get('confidence', 'N/A')
            rationale = grounding.get('rationale', 'N/A')
            
            print(f'\n[VLM] Target: "{target}"')
            print(f'[VLM] Bbox: {bbox}')
            print(f'[VLM] Confidence: {confidence}')
            print(f'[VLM] Rationale: {rationale}')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing VLM grounding: {e}')
    
    def explanation_callback(self, msg: String):
        """Handle VLM scene description/explanation"""
        print(f'\n[VLM] {msg.data}')
        print('\n' + '='*60)
        print('Ready for next command')
        print('> ', end='', flush=True)
    
    def position_callback(self, msg: PoseStamped):
        """Handle 3D position result"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        print(f'\n[Depth Resolver] 3D Position in {msg.header.frame_id}:')
        print(f'[Depth Resolver] X: {x:>7.3f} m')
        print(f'[Depth Resolver] Y: {y:>7.3f} m')
        print(f'[Depth Resolver] Z: {z:>7.3f} m')
        print('\n' + '='*60)
        print('Ready for next command')
        print('> ', end='', flush=True)
    
    def send_command(self, user_input: str):
        """Send user command to pipeline"""
        if not user_input.strip():
            return
        
        print('='*60)
        
        if self.use_llm:
            # Send to LLM for parsing
            print(f'[User] {user_input}')
            print('[System] Sending to LLM for task planning...')
            
            msg = String()
            msg.data = user_input
            self.user_command_pub.publish(msg)
        else:
            # Direct to VLM (skip LLM)
            print(f'[User] Directly locating: {user_input}')
            print('[System] Sending to VLM for grounding...')
            
            vlm_request = {'target': user_input, 'task': 'locate'}
            msg = String()
            msg.data = json.dumps(vlm_request)
            self.vlm_request_pub.publish(msg)


def main():
    rclpy.init()
    chat_node = SimpleChatNode()
    
    print('\n' + '='*60)
    print('Simple Terminal Chat - LLM → VLM → Depth Pipeline')
    print('='*60)
    print('\nCommands:')
    print('  - Natural language: "pick up the red cup"')
    print('  - Direct object: "apple" (if LLM disabled)')
    print('  - Quit: Ctrl+C or "quit"')
    print('\nMake sure these nodes are running:')
    print('  1. RealSense cameras (ros2 launch realsense_cameras cameras.launch.py)')
    print('  2. LLM node (ros2 run franka_llm_planner llm_node)')
    print('  3. VLM node (ros2 run franka_vlm_agent vlm_node)')
    print('  4. Coordinator (ros2 run franka_coordinator coordinator_node)')
    print('='*60 + '\n')
    
    # Spin in background
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(chat_node,), daemon=True)
    spin_thread.start()
    
    # Interactive terminal loop
    try:
        while True:
            user_input = input('> ')
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                print('Exiting...')
                break
            
            if user_input.strip():
                chat_node.send_command(user_input)
    
    except KeyboardInterrupt:
        print('\n\nShutting down...')
    
    finally:
        chat_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
