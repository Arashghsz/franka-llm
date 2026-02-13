# Franka Multi-Agent Manipulation Web Dashboard

A modern, ChatGPT-like web interface for controlling your Franka robot with natural language. Integrated with ROS2 backend for real-time task execution and status monitoring.

## Architecture

The dashboard communicates with a dynamic ROS2 backend:

```
┌─────────────────────────────────────────────────────┐
│          Web Dashboard (Browser)                     │
│  Chat | Status | Camera                              │
└──────────────────┬──────────────────────────────────┘
                   │ WebSocket
                   ↓
            ROSBridge Server (ws://localhost:9090)
                   │
        ┌──────────┴──────────┐
        ↓                     ↓
    /web/request         /web/response
        ↓                     ↑
    Coordinator Node      Web Handler
        │                  (franka_coordinator)
    ┌───┼───┬──────────────┐
    ↓   ↓   ↓              ↓
   LLM VLM Vision        Motion
   Planner   Detect      Executor
```

## Features

✨ **Modern UI**
- Clean, intuitive interface similar to ChatGPT
- Dark theme optimized for robot control
- Responsive design works on desktop and tablet
- Sidebar navigation between Chat, Status, and Camera views

🤖 **Dynamic Backend**
- Coordinator node orchestrates the pipeline
- Web handler bridges web to ROS2
- Real-time message routing and status updates
- Automatic task processing and responses

🔌 **ROS2 Integration**
- WebSocket-based ROSBridge communication
- Topic publishing and subscription
- Service calls support
- Automatic reconnection
- Status monitoring from all pipeline stages

📊 **Real-time Monitoring**
- Robot state tracking
- Vision system status
- LLM engine status
- ROS2 bridge health
- Current task display

## Quick Start

### Prerequisites
- Python 3.10+
- ROS2 Jazzy with ROSBridge installed
- colcon build system (for building ROS2 packages)

### Step 1: Build ROS2 Packages

```bash
cd /home/arash/franka-multiagent-manipulation

# Build the coordinator and web handler
colcon build --packages-select franka_coordinator

# Source the installation
source install/setup.bash
```

### Step 2: Start ROSBridge WebSocket Server

```bash
# In a terminal
source /opt/ros/jazzy/setup.bash

# Install if needed
sudo apt install ros-jazzy-rosbridge-server

# Launch
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Step 3: Start the ROS2 Coordinator and Web Handler

```bash
# In another terminal
cd /home/arash/franka-multiagent-manipulation
source install/setup.bash

# Option A: Run both nodes individually
# Terminal 3:
ros2 run franka_coordinator coordinator_node

# Terminal 4:
ros2 run franka_coordinator web_handler

# Option B: Run using launch file (when ready)
# ros2 launch franka_coordinator run_all.launch.py
```

### Step 4: Start the Web Server

```bash
# In another terminal
cd /home/arash/franka-multiagent-manipulation/ui/web_chat_dashboard
python3 -m http.server 8000
```

### Step 5: Open in Browser

Navigate to: **http://localhost:8000**

You should see:
- ✅ "Connected" status in sidebar (green indicator)
- ✅ Robot status, Vision, LLM, and ROS2 bridge status cards
- ✅ Chat input ready for commands

## Testing the System

### Test 1: Chat with ROSBridge Running

1. Ensure all 4 terminals are running (ROSBridge, Coordinator, Web Handler, HTTP Server)
2. Open http://localhost:8000
3. Type: "Pick up the red cube"
4. Click Send or press Enter
5. Check:
   - Message appears in chat (right side, green)
   - Coordinator logs show: "LLM request sent: Pick up the red cube"
   - Processing message appears
   - Status shows active task

### Test 2: Monitor Status Updates

1. In Status tab, watch the status cards
2. They should update every 1-2 seconds
3. If coordinator is running, Bridge state should show "Online"
4. Robot state depends on actual robot connection

### Test 3: Sidebar Connection Indicator

1. Stop ROSBridge server
2. Sidebar should show "Disconnected" (red indicator)
3. Restart ROSBridge
4. Should show "Connected" (green indicator) within 1 second

## ROS2 Topics Reference

The dashboard communicates with these topics:

**Web → ROS2:**
- `/web/request` (std_msgs/String): Chat messages and commands from web

**ROS2 → Web:**
- `/web/response` (std_msgs/String): Chat responses and task updates
- `/web/status` (std_msgs/String): System status (robot, vision, LLM, bridge)

**Internal Pipeline:**
- `/coordinator/status` (std_msgs/String): Coordinator status updates
- `/llm/request` (std_msgs/String): Requests to LLM
- `/llm/response` (std_msgs/String): Responses from LLM
- `/motion/command` (std_msgs/String): Motion executor commands
- `/motion/status` (std_msgs/String): Motion executor status
- `/vision/detections` (std_msgs/String): Vision detection results
- `/vlm/analyze_request` (std_msgs/String): VLM analysis requests
- `/vlm/analysis` (std_msgs/String): VLM analysis results

## File Structure

```
ui/web_chat_dashboard/
├── index.html           # Main HTML structure
├── styles.css           # Modern dark theme styling
├── app.js               # Application initialization & navigation
├── rosbridge.js         # ROSBridge WebSocket client
├── modules/
│   ├── chat.js          # Chat message handling
│   ├── camera.js        # Camera feed display
│   ├── status.js        # Status monitoring
│   ├── confirm.js       # Confirmation dialogs
│   └── utils.js         # Utility functions
├── assets/
│   └── logo.png         # Dashboard logo
└── README.md            # This file
```

## Development

### Adding New Features

1. **New View:**
   - Add HTML container in index.html
   - Add CSS styles in styles.css
   - Create module in modules/ if needed
   - Add nav button and route in app.js

2. **New ROS Topic:**
   - Subscribe in appropriate module
   - Handle in message callback
   - Update status display if needed

3. **New Status Indicator:**
   - Add status-card HTML
   - Initialize in status.js constructor
   - Update in handleCoordinatorStatus()

### Debugging

**Check browser console:**
```javascript
console.log('Debug: ' + variable);
```

**Check ROS topics:**
```bash
ros2 topic list              # List all topics
ros2 topic echo /topic/name  # View messages
```

**Check coordinator logs:**
```bash
ros2 run franka_coordinator coordinator_node --ros-args --log-level debug
```

## Troubleshooting

### "Cannot send message" Warning
- ROSBridge server not running
- Wrong WebSocket URL (should be ws://localhost:9090)
- Firewall blocking port 9090

**Solution:**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Status Shows "Offline" 
- Coordinator or Web Handler not running
- Status topic not being published
- Subscription failed

**Solution:**
```bash
ros2 run franka_coordinator coordinator_node
ros2 run franka_coordinator web_handler
```

### Messages Not Appearing
- Chat.js not publishing to /web/request
- Coordinator not receiving messages
- Web handler not publishing responses

**Check logs:**
```bash
ros2 run franka_coordinator coordinator_node --ros-args --log-level debug
```

### Input Field Too Narrow
- Fixed in latest version with full-width input
- Padding adjusted on chat-input-area
- Input wrapper uses flex: 1 to fill space
│ 🟢 Connected     │ [Input] [Send]   │
└─────────────────────────────────────┘
```

## Navigation

- **Chat**: Send commands and receive responses from LLM
- **Status**: View real-time system status (Robot, Vision, LLM, ROS2)
- **Camera**: Live camera feed from robot's end-effector

## Commands

Try these example commands:

```
"Pick up the red cube"
"Move to home position"
"Open the gripper"
"Identify objects on the table"
"Move 10cm forward"
"Take a photo"
```

## File Structure

```
web_chat_dashboard/
├── index.html              # Main HTML structure
├── styles.css              # Modern styling
├── app.js                  # Application logic
├── rosbridge.js            # ROS2 WebSocket client
├── modules/
│   ├── chat.js            # Chat functionality
│   ├── camera.js          # Camera feed handler
│   ├── status.js          # System status monitoring
│   ├── confirm.js         # Confirmation dialogs
│   └── utils.js           # Utility functions
└── assets/
    └── logo.png           # Logo placeholder
```

## Development

### Theme Customization

Edit CSS variables in `styles.css`:

```css
:root {
    --primary: #10a37f;        /* Green like OpenAI */
    --background: #ffffff;     /* Light mode */
    --text: #0d0d0d;
}

body.dark-theme {
    --background: #1f2937;     /* Dark mode */
    --text: #ececf1;
}
```

### Adding New Modules

1. Create `modules/your_feature.js`
2. Define a class with your feature logic
3. Initialize in `app.js`
4. Add UI elements to `index.html`

### Troubleshooting

**Dashboard shows "Disconnected"**
- Ensure ROSBridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check ROSBridge URL in `app.js` (should be `ws://localhost:9090`)

**Messages not sending**
- Verify ROS2 topics are being published: `ros2 topic list`
- Check browser console for errors (F12 → Console)

**Camera feed not working**
- Ensure camera topic `/cameras/ee/ee_camera/color/image_raw` is publishing
- Check ROS2 compatibility of image encoding

## Browser Support

- Chrome/Chromium 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Performance Tips

- Use Chrome/Chromium for best performance
- Close other tabs if experiencing lag
- Reduce camera refresh rate if needed

## Future Enhancements

- [ ] Voice input/output
- [ ] Gesture recognition
- [ ] Task history and replay
- [ ] Collision visualization
- [ ] RViz integration
- [ ] Settings panel
- [ ] Export conversations

---

Built with ❤️ for Franka Research 3 + ROS2 Jazzy
