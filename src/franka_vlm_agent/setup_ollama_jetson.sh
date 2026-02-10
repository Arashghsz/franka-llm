#!/bin/bash
# setup_ollama_jetson.sh
# Install and configure Ollama with LLaVA 7B on Jetson AGX Orin

set -e  # Exit on error

echo "============================================"
echo "Setting up Ollama + LLaVA 7B on Jetson"
echo "============================================"
echo ""

# Check if running on Jetson
if ! command -v jetson_release &> /dev/null; then
    echo "⚠ Warning: jetson_release not found. Are you running on Jetson?"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install Ollama
echo "Step 1: Installing Ollama..."
if command -v ollama &> /dev/null; then
    echo "✓ Ollama is already installed"
else
    # Download and install Ollama for ARM64 (Jetson)
    echo "Downloading Ollama for ARM64..."
    
    # Try the official Ollama installation method
    if curl -fsSL https://ollama.ai/install.sh | sh; then
        echo "✓ Ollama installed successfully"
    else
        echo "⚠ Warning: Official installation failed. You may need to install manually."
        echo "Visit: https://ollama.ai/download"
    fi
fi

echo ""
echo "Step 2: Downloading LLaVA 7B model..."
echo "⏳ This will take a few minutes (LLaVA 7B is ~5GB)..."
echo ""

# Start Ollama server in background
if ! pgrep -x "ollama" > /dev/null; then
    echo "Starting Ollama daemon..."
    ollama serve &
    OLLAMA_PID=$!
    sleep 5
else
    echo "✓ Ollama is already running"
fi

# Pull the LLaVA model
ollama pull llava:7b

# Verify installation
echo ""
echo "Step 3: Verifying installation..."
ollama list | grep llava

echo ""
echo "✓ Setup complete!"
echo ""
echo "============================================"
echo "Next steps:"
echo "============================================"
echo ""
echo "1. Start Ollama server (if not already running):"
echo "   ollama serve"
echo ""
echo "2. Test the VLM with a sample image:"
echo "   ros2 run franka_vlm_agent vlm_test"
echo ""
echo "3. Launch the VLM node on Jetson:"
echo "   ros2 launch franka_vlm_agent vlm.launch.py"
echo ""
echo "4. From PC controller, publish camera images to ROS 2:"
echo "   Make sure ROS_DOMAIN_ID matches on both machines"
echo ""
echo "============================================"
