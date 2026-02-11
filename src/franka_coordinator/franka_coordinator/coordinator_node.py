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
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo
import json
from datetime import datetime
from enum import Enum
import numpy as np
from cv_bridge import CvBridge

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
        
        # Depth resolution data
        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.bridge = CvBridge()
        
        # Publisher for outgoing commands
        self.llm_request_pub = self.create_publisher(
            String, '/llm/request', 10
        )
        self.motion_command_pub = self.create_publisher(
            String, '/motion/command', 10
        )
        self.vlm_request_pub = self.create_publisher(
            String, '/vlm_request', 10  # Updated topic name
        )
        
        # Status publishers
        self.status_pub = self.create_publisher(
            String, '/coordinator/status', 10
        )
        
        # Publisher for 3D target position
        self.target_position_pub = self.create_publisher(
            PoseStamped, '/target_position', 10
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
        self.vlm_grounding_sub = self.create_subscription(
            String, '/vlm_grounding', self.handle_vlm_grounding, 10
        )
        self.motion_status_sub = self.create_subscription(
            String, '/motion/status', self.handle_motion_status, 10
        )
        
        # Subscriber for robot state
        self.robot_state_sub = self.create_subscription(
            String, '/robot/state', self.handle_robot_state, 10
        )
        
        # Subscribers for depth image and camera info
        self.depth_sub = self.create_subscription(
            Image,
            '/cameras/ee/ee_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/cameras/ee/ee_camera/color/camera_info',
            self.camera_info_callback,
            10
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
    
    def depth_callback(self, msg: Image):
        """Store latest depth image for 3D position resolution"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Depth is in mm for RealSense, convert to meters
            if self.latest_depth_image.dtype == np.uint16:
                self.latest_depth_image = self.latest_depth_image.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics for deprojection"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(f'Camera intrinsics loaded: {self.camera_intrinsics}')
    
    def handle_vlm_grounding(self, msg: String):
        """
        Handle VLM grounding results (bbox + target)
        Convert bbox to 3D coordinates using depth
        """
        try:
            grounding = json.loads(msg.data)
            target = grounding.get('target', '')
            bbox = grounding.get('bbox', None)
            
            if not bbox or self.latest_depth_image is None or self.camera_intrinsics is None:
                self.get_logger().warn(
                    f'Cannot resolve 3D position: bbox={bbox is not None}, '
                    f'depth={self.latest_depth_image is not None}, '
                    f'intrinsics={self.camera_intrinsics is not None}'
                )
                return
            
            # Resolve 3D position from bbox and depth
            position_3d = self.resolve_3d_position(bbox, self.latest_depth_image, self.camera_intrinsics)
            
            if position_3d is not None:
                self.get_logger().info(
                    f'Target "{target}" located at 3D position: '
                    f'[{position_3d[0]:.3f}, {position_3d[1]:.3f}, {position_3d[2]:.3f}] meters'
                )
                
                # Publish 3D target position
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_color_optical_frame'
                pose_msg.pose.position.x = position_3d[0]
                pose_msg.pose.position.y = position_3d[1]
                pose_msg.pose.position.z = position_3d[2]
                pose_msg.pose.orientation.w = 1.0
                
                self.target_position_pub.publish(pose_msg)
            else:
                self.get_logger().error(f'Failed to resolve 3D position for {target}')
                
        except Exception as e:
            self.get_logger().error(f'Error handling VLM grounding: {e}')
    
    def resolve_3d_position(self, bbox, depth_image, camera_intrinsics):
        """
        Convert VLM bounding box to 3D coordinates
        
        Args:
            bbox: [x1, y1, x2, y2] in pixel coordinates
            depth_image: numpy array with depth in meters
            camera_intrinsics: dict with fx, fy, ppx, ppy
        
        Returns:
            [x, y, z] in meters in camera frame, or None if invalid
        """
        try:
            # 1. Get bbox centroid
            x1, y1, x2, y2 = bbox
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            # Clamp to image bounds
            cy = max(0, min(cy, depth_image.shape[0] - 1))
            cx = max(0, min(cx, depth_image.shape[1] - 1))
            y1 = max(0, min(int(y1), depth_image.shape[0] - 1))
            y2 = max(0, min(int(y2), depth_image.shape[0] - 1))
            x1 = max(0, min(int(x1), depth_image.shape[1] - 1))
            x2 = max(0, min(int(x2), depth_image.shape[1] - 1))
            
            # 2. Look up depth at centroid (with median filter for robustness)
            roi = depth_image[y1:y2, x1:x2]
            valid_depths = roi[roi > 0]  # Ignore zero (invalid) depths
            
            if len(valid_depths) == 0:
                self.get_logger().warn('No valid depth values in ROI')
                return None
            
            depth_z = float(np.median(valid_depths))
            
            self.get_logger().info(f'Bbox centroid: ({cx}, {cy}), Depth: {depth_z:.3f}m')
            
            # 3. Deproject pixel to 3D point using camera intrinsics
            fx = camera_intrinsics['fx']
            fy = camera_intrinsics['fy']
            ppx = camera_intrinsics['ppx']
            ppy = camera_intrinsics['ppy']
            
            x = (cx - ppx) * depth_z / fx
            y = (cy - ppy) * depth_z / fy
            z = depth_z
            
            # 4. Transform from camera frame to robot base frame (TODO: add TF)
            # For now, return in camera frame
            return [x, y, z]
            
        except Exception as e:
            self.get_logger().error(f'Error in 3D position resolution: {e}')
            return None
    
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
