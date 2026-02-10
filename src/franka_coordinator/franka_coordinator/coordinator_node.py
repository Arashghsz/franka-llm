#!/usr/bin/env python3
"""
Coordinator Node - Main orchestrator for the Franka LLM pipeline
Handles:
- Web requests from the dashboard
- Routing between LLM, VLM, Vision Detection, and Motion Executor
- Status monitoring and reporting
- Request/response flow management
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import json
from datetime import datetime
from enum import Enum

# Message types (we'll use std_msgs.String with JSON for now)
class TaskStatus(Enum):
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class FrankaCoordinator(Node):
    """Main coordinator node that orchestrates the robot pipeline."""
    
    def __init__(self):
        super().__init__('franka_coordinator')
        
        # Status tracking
        self.robot_status = "disconnected"
        self.vision_status = "offline"
        self.llm_status = "offline"
        self.current_task = None
        self.task_status = TaskStatus.PENDING.value
        
        # Publisher for outgoing commands
        self.llm_request_pub = self.create_publisher(
            String, '/llm/request', 10
        )
        self.motion_command_pub = self.create_publisher(
            String, '/motion/command', 10
        )
        self.vlm_request_pub = self.create_publisher(
            String, '/vlm/analyze_request', 10
        )
        
        # Status publishers
        self.status_pub = self.create_publisher(
            String, '/coordinator/status', 10
        )
        
        # Subscriber for web dashboard requests
        self.web_request_sub = self.create_subscription(
            String, '/web/request', self.handle_web_request, 10
        )
        
        # Subscribers for pipeline responses
        self.llm_response_sub = self.create_subscription(
            String, '/llm/response', self.handle_llm_response, 10
        )
        self.vision_response_sub = self.create_subscription(
            String, '/vision/detections', self.handle_vision_response, 10
        )
        self.vlm_response_sub = self.create_subscription(
            String, '/vlm/analysis', self.handle_vlm_response, 10
        )
        self.motion_status_sub = self.create_subscription(
            String, '/motion/status', self.handle_motion_status, 10
        )
        
        # Subscriber for robot state
        self.robot_state_sub = self.create_subscription(
            String, '/robot/state', self.handle_robot_state, 10
        )
        
        # Timer for periodic status checks
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info('Franka Coordinator initialized')
        self.publish_status()
    
    def handle_web_request(self, msg: String):
        """
        Handle incoming requests from the web dashboard.
        
        Expected message format:
        {
            "type": "chat" | "command" | "status",
            "data": {
                "message": "user input",
                ...
            }
        }
        """
        try:
            request = json.loads(msg.data)
            request_type = request.get('type', 'chat')
            data = request.get('data', {})
            
            self.get_logger().info(f'Web request: {request_type} - {data.get("message", "")}')
            
            if request_type == 'chat':
                # User asked something - send to LLM
                self.process_chat(data.get('message', ''))
            elif request_type == 'command':
                # Direct command - send to motion executor
                self.process_command(data.get('command', ''))
            elif request_type == 'clear':
                # Clear history - just log
                self.get_logger().info('Chat history cleared by user')
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in web request: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error handling web request: {e}')
    
    def process_chat(self, message: str):
        """Send chat message to LLM for processing."""
        if not message.strip():
            return
        
        self.current_task = message
        self.task_status = TaskStatus.PROCESSING.value
        
        # Create LLM request
        llm_request = {
            'timestamp': datetime.now().isoformat(),
            'message': message,
            'task_id': hash(message + datetime.now().isoformat()) % 10000
        }
        
        # Publish to LLM
        self.llm_request_pub.publish(
            String(data=json.dumps(llm_request))
        )
        
        self.get_logger().info(f'LLM request sent: {message}')
    
    def process_command(self, command: str):
        """Send command directly to motion executor."""
        self.current_task = command
        self.task_status = TaskStatus.PROCESSING.value
        
        motion_cmd = {
            'timestamp': datetime.now().isoformat(),
            'command': command,
            'task_id': hash(command + datetime.now().isoformat()) % 10000
        }
        
        self.motion_command_pub.publish(
            String(data=json.dumps(motion_cmd))
        )
        
        self.get_logger().info(f'Motion command sent: {command}')
    
    def handle_llm_response(self, msg: String):
        """Handle response from LLM node."""
        try:
            response = json.loads(msg.data)
            self.get_logger().info(f'LLM response: {response.get("response", "")}')
            
            # If LLM returns a command, send it to motion executor
            if 'command' in response:
                self.process_command(response['command'])
            
            # Publish response back to web
            web_response = {
                'type': 'llm_response',
                'sender': 'robot',
                'message': response.get('response', ''),
                'timestamp': datetime.now().isoformat()
            }
            
            self.motion_command_pub.publish(
                String(data=json.dumps(web_response))
            )
            
        except Exception as e:
            self.get_logger().error(f'Error handling LLM response: {e}')
    
    def handle_vision_response(self, msg: String):
        """Handle vision detection results."""
        try:
            detections = json.loads(msg.data)
            self.vision_status = "online"
            self.get_logger().debug(f'Vision detections: {len(detections)} objects')
        except Exception as e:
            self.get_logger().error(f'Error handling vision response: {e}')
    
    def handle_vlm_response(self, msg: String):
        """Handle VLM analysis results."""
        try:
            analysis = json.loads(msg.data)
            self.get_logger().info(f'VLM analysis: {analysis.get("result", "")}')
        except Exception as e:
            self.get_logger().error(f'Error handling VLM response: {e}')
    
    def handle_motion_status(self, msg: String):
        """Handle motion executor status updates."""
        try:
            status = json.loads(msg.data)
            motion_status = status.get('status', 'unknown')
            
            if motion_status == 'completed':
                self.task_status = TaskStatus.COMPLETED.value
                self.get_logger().info('Task completed successfully')
            elif motion_status == 'failed':
                self.task_status = TaskStatus.FAILED.value
                self.get_logger().error('Task failed')
            elif motion_status == 'executing':
                self.task_status = TaskStatus.PROCESSING.value
            
        except Exception as e:
            self.get_logger().error(f'Error handling motion status: {e}')
    
    def handle_robot_state(self, msg: String):
        """Handle robot state updates."""
        try:
            state = json.loads(msg.data)
            self.robot_status = "connected" if state.get('connected', False) else "disconnected"
        except Exception as e:
            self.robot_status = "disconnected"
    
    def publish_status(self):
        """Publish current status for monitoring."""
        status_msg = {
            'timestamp': datetime.now().isoformat(),
            'robot_state': self.robot_status,
            'vision_state': self.vision_status,
            'llm_state': self.llm_status,
            'bridge_state': 'online',  # If we're running, bridge is online
            'current_task': self.current_task,
            'task_status': self.task_status
        }
        
        self.status_pub.publish(
            String(data=json.dumps(status_msg))
        )


def main(args=None):
    rclpy.init(args=args)
    coordinator = FrankaCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
