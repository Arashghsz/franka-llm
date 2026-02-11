#!/usr/bin/env python3
"""
LLM Node - Connects Ollama to ROS 2

Subscribes to: /user_command (String)
Publishes to: /llm_response (String)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json


class LLMNode(Node):
    
    def __init__(self):
        super().__init__('llm_node')
        
        # Parameters
        self.declare_parameter('model', 'llama3.1:8b')
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        
        self.model = self.get_parameter('model').value
        self.ollama_url = self.get_parameter('ollama_url').value
        
        # Publisher
        self.publisher = self.create_publisher(String, '/llm_response', 10)
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )
        
        # System prompt for robot task planning
        self.system_prompt = """You are a robot task planner for a Franka arm.
Extract the action and target object from natural language commands.

Available actions: pick, place, move, locate, inspect

Examples:
- "pick up the red cup" → {"understood": true, "tasks": [{"action": "pick", "object": "red cup"}]}
- "move the apple to the left" → {"understood": true, "tasks": [{"action": "move", "object": "apple", "location": "left"}]}
- "find the screwdriver" → {"understood": true, "tasks": [{"action": "locate", "object": "screwdriver"}]}

Respond with JSON only. Keep object descriptions exactly as mentioned by the user.
Format:
{
  "understood": true,
  "tasks": [
    {"action": "pick|place|move|locate|inspect", "object": "target object name", "location": "optional"}
  ]
}
"""
        
        self.get_logger().info(f'LLM Node started. Model: {self.model}')
        self.get_logger().info('Listening on: /user_command')
        self.get_logger().info('Publishing to: /llm_response')
    
    def query_ollama(self, prompt: str) -> str:
        """Send query to Ollama API."""
        try:
            response = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model': self.model,
                    'prompt': f"{self.system_prompt}\n\nCommand: {prompt}",
                    'stream': False,
                    'format': 'json',  # Force JSON output
                    'options': {
                        'temperature': 0.3,
                        'num_predict': 500,
                    }
                },
                timeout=60
            )
            
            if response.status_code == 200:
                return response.json().get('response', '')
            else:
                self.get_logger().error(f'Ollama error: {response.status_code}')
                return ''
                
        except requests.exceptions.Timeout:
            self.get_logger().error('Ollama timeout')
            return ''
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return ''
    
    def command_callback(self, msg: String):
        """Handle incoming commands."""
        user_input = msg.data
        self.get_logger().info(f'Received: {user_input}')
        
        # Query LLM
        self.get_logger().info('Querying LLM...')
        response = self.query_ollama(user_input)
        
        if response:
            self.get_logger().info(f'LLM response: {response}')
            
            # Publish response
            out_msg = String()
            out_msg.data = response
            self.publisher.publish(out_msg)
        else:
            self.get_logger().error('No response from LLM')


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
