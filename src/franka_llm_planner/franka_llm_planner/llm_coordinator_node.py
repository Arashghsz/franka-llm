#!/usr/bin/env python3
"""
LLM Coordinator Node - Intelligent Router for Franka LLM Pipeline

This node acts as the main coordinator that:
1. Receives user commands
2. Uses LLM to understand intent and decide which agent to route to
3. Routes requests to appropriate agents (VLM, Motion, etc.)
4. Aggregates responses and sends back to user

Subscribes to: /user_command (String)
Publishes to: 
  - /vlm_request (String) - for scene analysis requests
  - /motion_command (String) - for motion execution requests
  - /coordinator_response (String) - responses back to user
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import requests
import json
from datetime import datetime
import yaml
from pathlib import Path


class LLMCoordinator(Node):
    
    def __init__(self):
        super().__init__('llm_coordinator')
        
        # Load configuration from config.yaml
        config_path = Path(__file__).parents[4] / 'config.yaml'
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # LLM settings from config file
        self.model = config['llm']['model']
        self.ollama_url = config['llm']['ollama_url']
        self.temperature = config['llm']['temperature']
        self.timeout = config['llm']['timeout']
        
        # Publishers for different agents
        self.vlm_request_pub = self.create_publisher(String, '/vlm_request', 10)
        self.motion_command_pub = self.create_publisher(String, '/motion_command', 10)
        self.response_pub = self.create_publisher(String, '/coordinator_response', 10)
        
        # Subscribers
        self.user_command_sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )
        
        self.vlm_response_sub = self.create_subscription(
            String,
            '/vlm/explanation',
            self.vlm_response_callback,
            10
        )
        
        self.vlm_position_sub = self.create_subscription(
            PoseStamped,
            '/vlm_center_position',
            self.vlm_position_callback,
            10
        )
        
        self.motion_status_sub = self.create_subscription(
            String,
            '/motion/status',
            self.motion_status_callback,
            10
        )
        
        # State tracking
        self.current_request = None
        self.waiting_for_vlm = False
        self.waiting_for_motion = False
        self.last_vlm_position = None
        
        # System prompt for routing decisions
        self.system_prompt = """You are an intelligent, friendly robot coordinator for a Franka manipulator arm.
Your name is "Franka Assistant" and you help users interact with the robot system.

You are conversational and helpful. Handle these types of interactions:

1. GREETINGS & SMALL TALK:
   - "hi", "hello", "hey" → Respond warmly, introduce yourself
   - "how are you", "what's up" → Respond conversationally
   - "who are you", "what are you" → Introduce yourself as Franka Assistant
   - For these, set target_agent to "none" and provide a friendly response

2. TASK ROUTING:
Available agents:
- VLM (Vision-Language Model): Analyzes the scene, describes what's on the table, locates objects
- MOTION: Executes physical movements (pick, place, move)

Decision rules:
- "what do you see", "describe the table", etc. → Route to VLM (scene description)
- "where is the [object]" → Route to VLM (object localization)
- "pick", "place", "move", "grasp" → Route to BOTH (VLM finds object, MOTION executes)

Respond in JSON format:
{
  "intent": "greeting" | "inspect" | "manipulate" | "locate" | "query",
  "target_agent": "none" | "vlm" | "motion" | "both",
  "action": "greet" | "describe_scene" | "locate_object" | "pick" | "place" | "move",
  "response": "your direct response for greetings/queries (if target_agent is none)",
  "parameters": {
    "object": "target object name if applicable",
    "location": "target location if applicable"
  },
  "reasoning": "brief explanation"
}

Examples:
- "hello" → {"intent": "greeting", "target_agent": "none", "action": "greet", "response": "Hello! I'm Franka Assistant, your robot coordinator. I can help you inspect the workspace or manipulate objects. What would you like to do?", ...}
- "what do you see?" → {"intent": "inspect", "target_agent": "vlm", "action": "describe_scene", ...}
- "where is the red cup?" → {"intent": "locate", "target_agent": "vlm", "action": "locate_object", "parameters": {"object": "red cup"}, ...}
- "pick up the apple" → {"intent": "manipulate", "target_agent": "both", "action": "pick", "parameters": {"object": "apple"}, ...}
"""
        
        self.get_logger().info(f'LLM Coordinator started. Model: {self.model}')
        self.get_logger().info('This node routes commands to VLM or Motion agents')
        self.get_logger().info('Listening on: /user_command')
    
    def query_llm(self, prompt: str) -> dict:
        """Send query to Ollama API for routing decision."""
        try:
            response = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model': self.model,
                    'prompt': f"{self.system_prompt}\n\nUser command: {prompt}",
                    'stream': False,
                    'format': 'json',
                    'options': {
                        'temperature': self.temperature,
                        'num_predict': 500,
                    }
                },
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                llm_response = response.json().get('response', '{}')
                try:
                    return json.loads(llm_response)
                except json.JSONDecodeError:
                    self.get_logger().error(f'LLM returned invalid JSON: {llm_response}')
                    return None
            else:
                self.get_logger().error(f'Ollama error: {response.status_code}')
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error('Ollama timeout')
            return None
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return None
    
    def command_callback(self, msg: String):
        """Handle incoming user commands and route to appropriate agent."""
        user_input = msg.data
        self.get_logger().info(f'Received command: {user_input}')
        
        # Store current request
        self.current_request = user_input
        
        # Query LLM for routing decision
        self.get_logger().info('Consulting LLM for routing decision...')
        decision = self.query_llm(user_input)
        
        if not decision:
            self.get_logger().error('Failed to get routing decision from LLM')
            self.publish_response('Sorry, I could not understand the request.')
            return
        
        self.get_logger().info(f'LLM Decision: {decision}')
        self.get_logger().info(f'  Intent: {decision.get("intent")}')
        self.get_logger().info(f'  Target Agent: {decision.get("target_agent")}')
        self.get_logger().info(f'  Action: {decision.get("action")}')
        self.get_logger().info(f'  Reasoning: {decision.get("reasoning")}')
        
        # Route based on decision
        target_agent = decision.get('target_agent', '').lower()
        action = decision.get('action', '')
        
        # Handle direct responses (greetings, queries, etc.)
        if target_agent == 'none':
            response = decision.get('response', 'I\'m here to help!')
            self.publish_response(response)
            return
        
        if target_agent == 'vlm' or target_agent == 'both':
            self.route_to_vlm(decision)
        
        if target_agent == 'motion':
            self.route_to_motion(decision)
        
        if target_agent == 'both':
            # For now, VLM first, then motion will use the position
            self.get_logger().info('Both agents needed - VLM will provide object location for motion planning')
    
    def route_to_vlm(self, decision: dict):
        """Route request to VLM agent."""
        self.get_logger().info('Routing to VLM agent...')
        self.waiting_for_vlm = True
        
        action = decision.get('action', '')
        parameters = decision.get('parameters', {})
        
        # Create VLM request
        vlm_request = {
            'type': 'scene_description',
            'timestamp': datetime.now().isoformat()
        }
        
        # For locate or pick actions, we need object localization
        if action in ['locate_object', 'pick', 'place', 'move']:
            target_object = parameters.get('object', '')
            if target_object:
                vlm_request['type'] = 'locate'
                vlm_request['object'] = target_object
                self.get_logger().info(f'Requesting VLM to locate: {target_object}')
        
        msg = String()
        msg.data = json.dumps(vlm_request)
        self.vlm_request_pub.publish(msg)
        
        self.get_logger().info(f'Sent request to VLM: {vlm_request}')
    
    def route_to_motion(self, decision: dict):
        """Route request to Motion agent."""
        self.get_logger().info('Routing to Motion agent...')
        self.waiting_for_motion = True
        
        action = decision.get('action', '')
        parameters = decision.get('parameters', {})
        
        # Create motion command
        motion_cmd = {
            'action': action,
            'parameters': parameters,
            'timestamp': datetime.now().isoformat()
        }
        
        # If we have a recent VLM position, include it
        if self.last_vlm_position:
            motion_cmd['target_position'] = self.last_vlm_position
        
        msg = String()
        msg.data = json.dumps(motion_cmd)
        self.motion_command_pub.publish(msg)
        
        self.get_logger().info(f'Sent command to Motion: {motion_cmd}')
    
    def vlm_response_callback(self, msg: String):
        """Handle VLM scene description response."""
        if not self.waiting_for_vlm:
            return
        
        self.waiting_for_vlm = False
        explanation = msg.data
        
        self.get_logger().info(f'Received VLM response: {explanation[:100]}...')
        
        # Publish back to user
        self.publish_response(f'[VLM] {explanation}')
    
    def vlm_position_callback(self, msg: PoseStamped):
        """Handle VLM 3D position updates."""
        self.last_vlm_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'frame': msg.header.frame_id
        }
        
        self.get_logger().info(
            f'VLM center position updated: '
            f'X={msg.pose.position.x:.3f}, '
            f'Y={msg.pose.position.y:.3f}, '
            f'Z={msg.pose.position.z:.3f}'
        )
    
    def motion_status_callback(self, msg: String):
        """Handle motion execution status updates."""
        if not self.waiting_for_motion:
            return
        
        try:
            status = json.loads(msg.data)
            motion_status = status.get('status', '')
            
            if motion_status in ['completed', 'failed', 'error']:
                self.waiting_for_motion = False
                self.publish_response(f'[Motion] {status.get("message", motion_status)}')
        except:
            pass
    
    def publish_response(self, message: str):
        """Publish response back to user."""
        msg = String()
        msg.data = message
        self.response_pub.publish(msg)
        self.get_logger().info(f'Response: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
