#!/usr/bin/env python3
"""
Web Handler Node - Bridge between web dashboard and ROS2
Handles:
- Publishing web requests to coordinator (LLM routing)
- Subscribing to coordinator and VLM responses
- Forwarding VLM images to web dashboard
- Handling motion confirmation flow
- Status monitoring
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
from threading import Lock
from pathlib import Path
import base64
import yaml


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
        
        # Pending motion confirmation state
        self.pending_motion = None
        self.vlm_model_name = 'VLM'  # Default, updated from system info
        self.llm_model_name = 'LLM'  # Default, updated from system info
        
        # System configuration cache
        self.system_config = {
            'llm': {},
            'vlm': {},
            'camera': {}
        }
        
        # Debug images directory
        self.debug_images_dir = Path('/home/arash/franka-llm/debug_images')
        
        # Load system configuration
        self._load_system_config()
        
        # Publishers
        # Publish user commands directly to LLM coordinator
        self.user_command_pub = self.create_publisher(
            String, '/user_command', 10
        )
        
        # Subscriber to web requests from browser
        self.web_request_sub = self.create_subscription(
            String, '/web/request', self.handle_web_request, 10
        )
        
        # Publisher for status updates going to web
        self.web_status_pub = self.create_publisher(
            String, '/web/status', 10
        )
        
        # Publisher for chat responses going to web
        self.web_response_pub = self.create_publisher(
            String, '/web/response', 10
        )
        
        # Publisher for user confirmation responses to coordinator
        self.user_confirmation_pub = self.create_publisher(
            String, '/user_confirmation', 10
        )
        
        # Subscribers
        # Coordinator response - main chat responses
        self.coordinator_response_sub = self.create_subscription(
            String, '/coordinator_response', self.handle_coordinator_response, 10
        )
        
        # VLM grounding info with bbox
        self.vlm_grounding_sub = self.create_subscription(
            String, '/vlm_grounding', self.handle_vlm_grounding, 10
        )
        
        # VLM explanation text
        self.vlm_explanation_sub = self.create_subscription(
            String, '/vlm/explanation', self.handle_vlm_explanation, 10
        )
        
        # Subscriber to coordinator status
        self.coordinator_status_sub = self.create_subscription(
            String, '/coordinator/status', self.handle_coordinator_status, 10
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
        
        self.get_logger().info('Web Handler initialized - integrated with LLM Coordinator')
        
        # Send welcome message with system info after a short delay (one-shot)
        self.welcome_timer = self.create_timer(2.0, self._send_welcome_message_once)
    
    def _load_system_config(self):
        """Load configuration from config.yaml."""
        try:
            config_path = Path('/home/arash/franka-llm/config.yaml')
            if config_path.exists():
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    self.system_config['llm'] = config.get('llm', {})
                    self.system_config['vlm'] = config.get('vlm', {})
                    self.system_config['camera'] = config.get('camera', {})
                    self.llm_model_name = self.system_config['llm'].get('model', 'LLM')
                    self.vlm_model_name = self.system_config['vlm'].get('model', 'VLM')
                self.get_logger().info(f'Loaded config: LLM={self.llm_model_name}, VLM={self.vlm_model_name}')
        except Exception as e:
            self.get_logger().warn(f'Could not load config.yaml: {e}')
    
    def _send_welcome_message_once(self):
        """Send welcome message once and cancel timer."""
        try:
            self._send_welcome_message()
            # Cancel the timer so it doesn't repeat
            self.welcome_timer.cancel()
        except Exception as e:
            self.get_logger().error(f'Error in welcome timer: {e}')
    
    def _send_welcome_message(self):
        """Send welcome message with system configuration to web."""
        try:
            llm_cfg = self.system_config.get('llm', {})
            vlm_cfg = self.system_config.get('vlm', {})
            
            config_text = f"""**Hello! I'm Franka Assistant**, your intelligent robot coordinator.

**System Configuration:**

🧠 **LLM Coordinator**
• Model: {llm_cfg.get('model', 'Not configured')}
• Temperature: {llm_cfg.get('temperature', 'N/A')}
• Timeout: {llm_cfg.get('timeout', 'N/A')}s

🔍 **Vision Agent (VLM)**
• Model: {vlm_cfg.get('model', 'Not configured')}
• Temperature: {vlm_cfg.get('temperature', 'N/A')}
• Timeout: {vlm_cfg.get('timeout', 'N/A')}s

📷 **Camera System**
• RealSense D435i
• Resolution: 640x480
• FPS: 30

✨ **I can help you with:**
• Scene understanding and object detection
• Natural language motion planning
• Real-time visual feedback
• Interactive motion approval

💡 **Tip:** All actions require your approval before execution."""
            
            welcome_message = {
                'type': 'message',
                'sender': 'system',
                'agent_name': 'Franka Assistant',
                'message': config_text,
                'timestamp': datetime.now().isoformat()
            }
            
            self.web_response_pub.publish(
                String(data=json.dumps(welcome_message))
            )
            self.get_logger().info('Sent welcome message with system configuration')
        except Exception as e:
            self.get_logger().error(f'Error sending welcome message: {e}')
    
    def handle_web_request(self, msg: String):
        """Handle incoming requests from web dashboard."""
        try:
            request = json.loads(msg.data)
            req_type = request.get('type', 'chat')
            
            if req_type == 'chat':
                # Forward chat message to LLM coordinator
                user_message = request['data']['message']
                self.get_logger().info(f'Web chat: {user_message}')
                
                # Send log to web
                self._send_log_message('Web Handler', 'Request received, forwarding to Coordinator...')
                
                # Publish to coordinator
                command_msg = String()
                command_msg.data = user_message
                self.user_command_pub.publish(command_msg)
                
                # Confirm forwarding
                self._send_log_message('Web Handler', 'Sent to LLM Coordinator')
                
            elif req_type == 'confirmation':
                # Handle motion confirmation response from user
                confirmed = request.get('confirmed', False)
                self.handle_user_confirmation(confirmed)
                
        except Exception as e:
            self.get_logger().error(f'Error handling web request: {e}')
    
    def handle_coordinator_response(self, msg: String):
        """Forward coordinator response to web dashboard."""
        try:
            response_text = msg.data
            
            # Update LLM state as online
            with self.status_lock:
                self.status_cache['llm_state'] = 'Online'
            
            # Check if this is system info
            if response_text.startswith('{'):
                try:
                    info = json.loads(response_text)
                    if info.get('type') == 'system_info' and info.get('component') == 'llm':
                        self.llm_model_name = info.get('model', 'LLM')
                        self.system_config['llm'] = info.get('config', {})
                        self.get_logger().info(f'Updated LLM model name: {self.llm_model_name}')
                        return  # Don't forward system info to web
                except (json.JSONDecodeError, ValueError):
                    pass  # Not JSON, continue with normal processing
            
            # Check if this is a log message
            if response_text.startswith('[LOG]'):
                # Extract and forward log message
                log_data = json.loads(response_text[5:])
                self.web_response_pub.publish(
                    String(data=json.dumps(log_data))
                )
                return
            
            # Detect which agent based on prefix
            agent_name = f'🧠 LLM Coordinator ({self.llm_model_name})'
            if response_text.startswith('[VLM]'):
                agent_name = f'🔍 Vision Agent ({self.vlm_model_name})'
                response_text = response_text[5:].strip()
            elif response_text.startswith('[Motion]'):
                agent_name = '⚙️ Motion Controller'
                response_text = response_text[8:].strip()
            
            web_message = {
                'type': 'message',
                'sender': 'robot',
                'agent_name': agent_name,
                'message': response_text,
                'timestamp': datetime.now().isoformat()
            }
            
            self.web_response_pub.publish(
                String(data=json.dumps(web_message))
            )
            
            self.get_logger().info(f'Forwarded coordinator response to web')
            
        except Exception as e:
            self.get_logger().error(f'Error handling coordinator response: {e}')
    
    def handle_vlm_explanation(self, msg: String):
        """Handle VLM explanation/description responses."""
        try:
            explanation = msg.data
            
            # Update vision state as online
            with self.status_lock:
                self.status_cache['vision_state'] = 'Online'
            
            # Check if this is system info
            try:
                info = json.loads(explanation)
                if info.get('type') == 'system_info' and info.get('component') == 'vlm':
                    self.vlm_model_name = info.get('model', 'VLM')
                    self.get_logger().info(f'Updated VLM model name: {self.vlm_model_name}')
                    return  # Don't forward system info to web
            except (json.JSONDecodeError, ValueError):
                pass  # Not JSON, continue with normal processing
            
            # Only include image if this is a scene description (not JSON object detection)
            latest_image = None
            formatted_message = explanation
            
            try:
                # If it parses as JSON, it's likely an object detection result
                detection_data = json.loads(explanation)
                
                # Format the JSON detection data nicely
                if isinstance(detection_data, dict):
                    center = detection_data.get('center', [])
                    description = detection_data.get('description', '')
                    confidence = detection_data.get('confidence', 'unknown')
                    
                    if center and len(center) == 2:
                        formatted_message = f"**Object Located**\n\n{description}\n\n**Position:** Pixel ({center[0]}, {center[1]})\n**Confidence:** {confidence}"
                    else:
                        formatted_message = f"**Detection Result**\n\n{description}\n\n**Confidence:** {confidence}"
                
                # Get most recent image (not object-specific to avoid old files)
                latest_image = self._get_latest_debug_image(None)
                self._send_log_message(f'Vision Agent ({self.vlm_model_name})', 'Object detection analysis complete')
            except (json.JSONDecodeError, ValueError):
                # It's a scene description - include the scene image
                latest_image = self._get_latest_debug_image('scene_description')
                self._send_log_message(f'Vision Agent ({self.vlm_model_name})', 'Scene analysis complete')
            
            web_message = {
                'type': 'message',
                'sender': 'vlm',
                'agent_name': f'Vision Agent ({self.vlm_model_name})',
                'message': formatted_message,
                'timestamp': datetime.now().isoformat()
            }
            
            if latest_image:
                web_message['image'] = latest_image
                self.get_logger().info(f'Image attached: {len(latest_image)} chars')
                
            json_str = json.dumps(web_message)
            self.get_logger().info(f'Publishing message: {len(json_str)} chars total')
                
            self.web_response_pub.publish(
                String(data=json_str)
            )
            
            self.get_logger().info(f'Forwarded VLM explanation to web')
            
        except Exception as e:
            self.get_logger().error(f'Error handling VLM explanation: {e}')
    
    def handle_vlm_grounding(self, msg: String):
        """Handle VLM grounding info (object detection with bbox)."""
        try:
            grounding_data = json.loads(msg.data)
            target = grounding_data.get('target', 'object')
            action = grounding_data.get('action', 'pick')
            bbox = grounding_data.get('bbox', [])
            center = grounding_data.get('center', [])
            
            self.get_logger().info(f'VLM grounding: {target} at {center}')
            
            # Store this for motion confirmation
            self.pending_motion = {
                'target': target,
                'action': action,
                'bbox': bbox,
                'center': center
            }
            
            # Send planning message
            self._send_log_message('Motion Planner', 'Analyzing approach path...')
            
            # Now request confirmation from user for motion execution (inline, not modal)
            self._request_motion_confirmation(target, action)
            
        except Exception as e:
            self.get_logger().error(f'Error handling VLM grounding: {e}')
    
    def _request_motion_confirmation(self, target: str, action: str):
        """Request user confirmation before executing motion."""
        try:
            confirmation_message = {
                'type': 'confirmation_request',
                'sender': 'system',
                'agent_name': '⚙️ Motion Controller',
                'message': f'Ready to execute: **{action} {target}**\n\n📋 Action plan:\n• Approach object at detected location\n• Execute {action} maneuver\n• Return to safe position',
                'action': action,
                'target': target,
                'timestamp': datetime.now().isoformat()
            }
            
            self.web_response_pub.publish(
                String(data=json.dumps(confirmation_message))
            )
            
            self.get_logger().info(f'Requested motion confirmation for: {action} {target}')
            
        except Exception as e:
            self.get_logger().error(f'Error requesting confirmation: {e}')
    
    def _send_log_message(self, agent_name: str, message: str):
        """Send a log message to the chat."""
        try:
            web_message = {
                'type': 'log',
                'sender': 'system',
                'agent_name': agent_name,
                'message': message,
                'timestamp': datetime.now().isoformat()
            }
            
            self.web_response_pub.publish(
                String(data=json.dumps(web_message))
            )
        except Exception as e:
            self.get_logger().error(f'Error sending log message: {e}')
    
    def handle_user_confirmation(self, confirmed: bool):
        """Handle user confirmation response."""
        try:
            if not self.pending_motion:
                self.get_logger().warn('No pending motion to confirm')
                return
            
            # Forward user confirmation to coordinator
            confirmation_msg = {
                'confirmed': confirmed,
                'motion': self.pending_motion,
                'timestamp': datetime.now().isoformat()
            }
            
            self.user_confirmation_pub.publish(
                String(data=json.dumps(confirmation_msg))
            )
            
            self.get_logger().info(f'User {"approved" if confirmed else "cancelled"} motion')
            
            # Send feedback to web
            if confirmed:
                web_message = {
                    'type': 'message',
                    'sender': 'system',
                    'agent_name': 'User Approval',
                    'message': f'Motion approved. Executing {self.pending_motion["action"]} operation...',
                    'timestamp': datetime.now().isoformat()
                }
            else:
                web_message = {
                    'type': 'message',
                    'sender': 'system',
                    'agent_name': 'User Cancelled',
                    'message': 'Motion execution cancelled.',
                    'timestamp': datetime.now().isoformat()
                }
            
            self.web_response_pub.publish(
                String(data=json.dumps(web_message))
            )
            
            self.get_logger().info(f'Motion {"approved" if confirmed else "cancelled"}')
            
            # Clear pending motion
            if not confirmed:
                self.pending_motion = None
            
        except Exception as e:
            self.get_logger().error(f'Error handling confirmation: {e}')
    
    def _get_latest_debug_image(self, object_name: str = None) -> str:
        """Get the latest debug image as base64 data URL."""
        try:
            if not self.debug_images_dir.exists():
                return None
            image_files = list(self.debug_images_dir.glob('*.jpg'))
            if not image_files:
                return None
            latest_file = None
            # If object_name specified, try to find the most recent matching image
            if object_name:
                normalized_name = object_name.lower().replace(' ', '_')
                # Find all files containing the normalized name
                matching_files = [f for f in image_files if normalized_name in f.name.lower()]
                if matching_files:
                    # Pick the most recent matching file
                    latest_file = max(matching_files, key=lambda f: f.stat().st_mtime)
            # If no match or no object_name, and specifically for scene_description, try to get the most recent scene_description image
            if object_name == 'scene_description' and (not latest_file):
                scene_files = [f for f in image_files if 'scene_description' in f.name.lower()]
                if scene_files:
                    latest_file = max(scene_files, key=lambda f: f.stat().st_mtime)
            # Fallback: just get the most recent image
            if not latest_file:
                latest_file = max(image_files, key=lambda f: f.stat().st_mtime)
            # Read and encode as base64
            with open(latest_file, 'rb') as f:
                image_data = f.read()
                if not image_data:
                    self.get_logger().error(f'Empty image file: {latest_file.name}')
                    return None
                image_base64 = base64.b64encode(image_data).decode('utf-8')
                data_url = f'data:image/jpeg;base64,{image_base64}'
                self.get_logger().info(
                    f'Image encoding complete:\n'
                    f'  File: {latest_file.name}\n'
                    f'  Raw bytes: {len(image_data)}\n'
                    f'  Base64 length: {len(image_base64)}\n'
                    f'  Data URL length: {len(data_url)}\n'
                    f'  First 50 chars: {data_url[:50]}'
                )
                return data_url
            
        except Exception as e:
            self.get_logger().error(f'Error loading debug image: {e}')
            return None
    
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
        """Forward LLM response to web dashboard (legacy, now using coordinator_response)."""
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
                message = f'✓ Motion sequence completed successfully'
                msg_type = 'message'
                # Clear pending motion on completion
                self.pending_motion = None
            elif status_text == 'failed':
                message = f'✗ Motion failed: {status.get("error", "Unknown error")}'
                msg_type = 'message'
                self.pending_motion = None
            elif status_text == 'executing':
                message = f'Executing motion sequence...'
                msg_type = 'log'
            elif status_text == 'planning':
                message = f'Planning trajectory...'
                msg_type = 'log'
            elif status_text == 'approaching':
                message = f'Moving to approach position...'
                msg_type = 'log'
            else:
                message = f'Status: {status_text}'
                msg_type = 'log'
            
            web_message = {
                'type': msg_type,
                'sender': 'system',
                'agent_name': '⚙️ Motion Executor',
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
        DEPRECATED: Now handled by handle_web_request
        """
        request = {
            'type': 'chat',
            'data': {
                'message': user_input,
                'timestamp': datetime.now().isoformat()
            }
        }
        
        self.user_command_pub.publish(
            String(data=request['data']['message'])
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
