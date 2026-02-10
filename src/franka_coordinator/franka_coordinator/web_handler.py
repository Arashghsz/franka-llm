#!/usr/bin/env python3
"""
Web Handler Node - Bridge between web dashboard and ROS2
Handles:
- Publishing web requests to ROS2 topics
- Subscribing to ROS2 topics and sending updates to web
- Status monitoring
- Camera feed streaming (placeholder)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
from threading import Lock


class WebHandler(Node):
    """Node that bridges web dashboard communication with ROS2."""
    
    def __init__(self):
        super().__init__('web_handler')
        self.status_lock = Lock()
        
        # Current system status
        self.status_cache = {
            'robot_state': 'Disconnected',
            'vision_state': 'Offline',
            'llm_state': 'Offline',
            'bridge_state': 'Online',
            'timestamp': datetime.now().isoformat()
        }
        
        # Publisher for web requests going to coordinator
        self.web_request_pub = self.create_publisher(
            String, '/web/request', 10
        )
        
        # Publisher for status updates going to web
        self.web_status_pub = self.create_publisher(
            String, '/web/status', 10
        )
        
        # Publisher for chat responses going to web
        self.web_response_pub = self.create_publisher(
            String, '/web/response', 10
        )
        
        # Subscriber to coordinator status
        self.coordinator_status_sub = self.create_subscription(
            String, '/coordinator/status', self.handle_coordinator_status, 10
        )
        
        # Subscriber to LLM responses
        self.llm_response_sub = self.create_subscription(
            String, '/llm/response', self.handle_llm_response, 10
        )
        
        # Subscriber to motion status
        self.motion_status_sub = self.create_subscription(
            String, '/motion/status', self.handle_motion_status, 10
        )
        
        # Subscriber to vision data
        self.vision_sub = self.create_subscription(
            String, '/vision/detections', self.handle_vision_data, 10
        )
        
        # Timer to publish status updates periodically
        self.status_timer = self.create_timer(1.0, self.publish_web_status)
        
        self.get_logger().info('Web Handler initialized')
    
    def handle_coordinator_status(self, msg: String):
        """Handle status updates from coordinator."""
        try:
            status = json.loads(msg.data)
            
            with self.status_lock:
                self.status_cache['robot_state'] = status.get('robot_state', 'Unknown').title()
                self.status_cache['vision_state'] = status.get('vision_state', 'Unknown').title()
                self.status_cache['llm_state'] = status.get('llm_state', 'Unknown').title()
                self.status_cache['bridge_state'] = 'Online'
                self.status_cache['timestamp'] = status.get('timestamp', datetime.now().isoformat())
            
            self.get_logger().debug(f'Status updated: {self.status_cache}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling coordinator status: {e}')
    
    def handle_llm_response(self, msg: String):
        """Forward LLM response to web dashboard."""
        try:
            response = json.loads(msg.data)
            
            web_message = {
                'type': 'message',
                'sender': 'robot',
                'message': response.get('response', 'Task processing...'),
                'timestamp': datetime.now().isoformat()
            }
            
            self.web_response_pub.publish(
                String(data=json.dumps(web_message))
            )
            
            self.get_logger().info(f'Forwarded LLM response to web')
            
        except Exception as e:
            self.get_logger().error(f'Error handling LLM response: {e}')
    
    def handle_motion_status(self, msg: String):
        """Forward motion status to web dashboard."""
        try:
            status = json.loads(msg.data)
            status_text = status.get('status', 'Unknown')
            
            if status_text == 'completed':
                message = f'✅ Task completed successfully!'
            elif status_text == 'failed':
                message = f'❌ Task failed: {status.get("error", "Unknown error")}'
            elif status_text == 'executing':
                message = f'⚙️ Executing task...'
            else:
                message = f'Status: {status_text}'
            
            web_message = {
                'type': 'status',
                'sender': 'system',
                'message': message,
                'timestamp': datetime.now().isoformat()
            }
            
            self.web_response_pub.publish(
                String(data=json.dumps(web_message))
            )
            
        except Exception as e:
            self.get_logger().error(f'Error handling motion status: {e}')
    
    def handle_vision_data(self, msg: String):
        """Handle vision detection data."""
        try:
            detections = json.loads(msg.data)
            with self.status_lock:
                self.status_cache['vision_state'] = 'Online'
        except Exception as e:
            self.get_logger().error(f'Error handling vision data: {e}')
    
    def publish_web_status(self):
        """Publish status updates to web."""
        with self.status_lock:
            status_to_send = dict(self.status_cache)
        
        self.web_status_pub.publish(
            String(data=json.dumps(status_to_send))
        )
    
    def process_web_input(self, user_input: str):
        """
        Process input from web and send to coordinator.
        This method would be called by a web interface adapter.
        """
        request = {
            'type': 'chat',
            'data': {
                'message': user_input,
                'timestamp': datetime.now().isoformat()
            }
        }
        
        self.web_request_pub.publish(
            String(data=json.dumps(request))
        )


def main(args=None):
    rclpy.init(args=args)
    web_handler = WebHandler()
    
    try:
        rclpy.spin(web_handler)
    except KeyboardInterrupt:
        pass
    finally:
        web_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
