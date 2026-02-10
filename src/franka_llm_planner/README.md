# LLM Task Planner

Converts natural language commands to robot task plans using Ollama.

## Setup

```bash
# Install Ollama (if not already installed)
curl -fsSL https://ollama.ai/install.sh | sh

# Pull LLaMA model
ollama pull llama3.1:8b

# Build package
cd /home/arash/franka-llm
colcon build --packages-select franka_llm_planner
source install/setup.zsh  # or setup.bash
```

## Run

**Start Ollama:**
```bash
ollama serve &
```

**Start LLM Node:**
```bash
export ROS_DOMAIN_ID=0
ros2 run franka_llm_planner llm_node
```

With custom model:
```bash
ros2 run franka_llm_planner llm_node --ros-args \
  -p model:=llama3.1:8b \
  -p ollama_url:=http://localhost:11434
```

## Use

**Send commands:**
```bash
ros2 topic pub /user_command std_msgs/String "data: 'pick up the red cube'"
```

**View responses:**
```bash
ros2 topic echo /llm_response
```

## Topics

- **Subscribe:** `/user_command` (String) - Natural language commands
- **Publish:** `/llm_response` (String) - JSON task plans

## Example

**Input:**
```
"pick the blue cube and place it on the table"
```

**Output:**
```json
{
  "understood": true,
  "tasks": [
    {"action": "pick", "object": "blue cube"},
    {"action": "place", "location": "table"}
  ]
}
```
