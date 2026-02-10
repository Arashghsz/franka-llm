#!/bin/bash
# quick_start_vlm.sh
# Quick start script for VLM setup on Jetson

set -e

WORKSPACE_DIR="/home/arash/franka-llm"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "============================================"
echo "VLM Agent - Quick Start"
echo "============================================"
echo ""

# Check if running on Jetson
if ! command -v jetson_release &> /dev/null; then
    echo "⚠ Warning: Not running on Jetson"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check Ollama
echo "1. Checking Ollama..."
if ! command -v ollama &> /dev/null; then
    echo "   Installing Ollama..."
    curl -fsSL https://ollama.ai/install.sh | sh
else
    echo "   ✓ Ollama is installed"
fi

# Start Ollama if not running
if ! pgrep -x "ollama" > /dev/null; then
    echo "   Starting Ollama daemon..."
    ollama serve &
    OLLAMA_PID=$!
    sleep 3
else
    echo "   ✓ Ollama is running"
fi

# Check/Pull LLaVA model
echo ""
echo "2. Checking LLaVA 7B model..."
if ollama list | grep -q llava; then
    echo "   ✓ LLaVA model is available"
else
    echo "   ⏳ Pulling llava:7b (this takes 5-10 minutes)..."
    ollama pull llava:7b
fi

# Build VLM package
echo ""
echo "3. Building VLM package..."
cd "$WORKSPACE_DIR"
colcon build --packages-select franka_vlm_agent
source install/setup.bash

# Test Ollama
echo ""
echo "4. Testing Ollama connection..."
ros2 run franka_vlm_agent vlm_test --model llava:7b

echo ""
echo "============================================"
echo "✓ Setup Complete!"
echo "============================================"
echo ""
echo "Next steps:"
echo ""
echo "1. On PC Controller, start publishing camera:"
echo "   export ROS_DOMAIN_ID=0"
echo "   ros2 launch realsense2_camera rs_launch.py"
echo ""
echo "2. On Jetson, launch VLM node:"
echo "   export ROS_DOMAIN_ID=0"
echo "   source ~/franka-llm/install/setup.bash"
echo "   ros2 launch franka_vlm_agent vlm.launch.py"
echo ""
echo "3. Monitor VLM output:"
echo "   ros2 topic echo /vlm/explanation"
echo ""
