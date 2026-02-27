# 🌐 Web Dashboard - Full Coordinator Integration

## 🎯 What's New

The web dashboard now fully integrates with the LLM Coordinator pipeline, providing:

### ✅ Complete Features:
1. **LLM Coordinator Integration** - Chat directly with the intelligent coordinator
2. **VLM Image Display** - Scene descriptions and object detections shown with images
3. **Motion Confirmation** - User approval required before robot executes actions
4. **Removed Camera Tab** - Streamlined UI focused on chat interaction
5. **Real-time Updates** - Status monitoring and response streaming

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│          Web Browser Dashboard                          │
│  - Chat Interface                                       │
│  - Image Display (VLM outputs)                          │
│  - Confirmation Dialog                                  │
│  - Status Monitor                                       │
└──────────────────┬──────────────────────────────────────┘
                   │ WebSocket (ws://localhost:9090)
                   ↓
            ROSBridge Server
                   │
        ┌──────────┼──────────┐
        ↓          ↓          ↓
   /web/request  /web/response  /web/status
        │          ↑          ↑
        ↓          │          │
    Web Handler ───┴──────────┘
        │
        ↓ /user_command
    LLM Coordinator (llm_coordinator)
        │
    ┌───┼────┬──────────────┐
    ↓   ↓    ↓              ↓
   VLM  LLM  Motion      Vision
   Agent     Executor    Detection
    │
    └─→ /vlm/explanation (text responses)
    └─→ /vlm_grounding (object detection with bbox)
    └─→ Debug images saved to /debug_images/
```

---

## 🚀 Complete Setup Guide

### Prerequisites
- ROS2 Jazzy
- ROSBridge Server
- Python 3.10+
- Web browser (Chrome/Firefox/Edge)

### Step 1: Build Updated Packages

```bash
cd /home/arash/franka-llm

# Build franka_coordinator with updated web_handler
colcon build --packages-select franka_coordinator

# Source the installation
source install/setup.zsh
```

### Step 2: Start ROSBridge WebSocket Server

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Step 3: Start RealSense Camera

```bash
# Terminal 2
cd /home/arash/franka-llm
source install/setup.zsh
ros2 launch realsense_cameras cameras.launch.py
```

### Step 4: Start LLM Coordinator

```bash
# Terminal 3
cd /home/arash/franka-llm
source install/setup.zsh
ros2 run franka_llm_planner llm_coordinator
```

### Step 5: Start VLM Agent

```bash
# Terminal 4
cd /home/arash/franka-llm
source install/setup.zsh
ros2 run franka_vlm_agent vlm_node
```

### Step 6: Start Web Handler

```bash
# Terminal 5
cd /home/arash/franka-llm
source install/setup.zsh
ros2 run franka_coordinator web_handler
```

### Step 7: Start Web Server

```bash
# Terminal 6
cd /home/arash/franka-llm/ui/web_chat_dashboard
python3 -m http.server 8000
```

### Step 8: Start Coordinator Node (ArUco transform + motion routing)

```bash
# Terminal 7
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_coordinator coordinator_node
```

> This node loads the ArUco calibration from `src/realsense_cameras/new_calibration/`
> and converts VLM pixel detections into robot-base-frame coordinates.

### Step 9: Start Motion Executor

```bash
# Terminal 8
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_motion_executor motion_executor
```

### Step 10: Launch MoveIt (real robot)

```bash
# Terminal 9
cd ~/franka_ros2_ws
source install/setup.zsh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false
```

### Step 11: Open Browser

Navigate to: **http://localhost:8000**

You should see:
- ✅ "Connected" status in sidebar (green indicator)
- ✅ Chat interface ready
- ✅ Status cards showing system state

---



















### Example 1: Scene Inspection with Image

**User:** "What do you see on the table?"

**Flow:**
1. Web → Web Handler → LLM Coordinator
2. LLM routes to VLM agent
3. VLM analyzes scene, saves debug image
4. Web Handler loads latest debug image
5. Response with image appears in chat

**Result in Chat:**
```
🔍 I can see several objects on the table including a red dice
   on the left side, a yellow dice in the center, and a screwdriver
   near the right edge.

[Image showing the scene]
```

### Example 2: Object Localization with Detection Image

**User:** "Where is the yellow dice?"

**Flow:**
1. Web → Web Handler → LLM Coordinator → VLM
2. VLM locates object, marks it, saves debug image
3. Web Handler loads debug image with marker
4. Response with marked image appears in chat

**Result in Chat:**
```
📍 Located: yellow dice at pixel (320, 240)

[Image with red marker on the yellow dice]
```

### Example 3: Pick Action with Confirmation

**User:** "Pick up the yellow dice"

**Flow:**
1. Web → Web Handler → LLM Coordinator
2. LLM routes to VLM (locate object)
3. VLM finds object, publishes grounding
4. Web Handler receives grounding, shows detection image
5. **Confirmation dialog appears**
6. User clicks "Approve" or "Cancel"
7. If approved, motion executor proceeds

**Result in Chat:**
```
📍 Located: yellow dice at pixel (320, 240)

[Image with marker]

[Confirmation Dialog]
⚠️ Confirm Action

The robot will pick the yellow dice.

⚠️ Do you approve this action?

[❌ Cancel]  [✅ Approve]
```

If approved:
```
✅ Action approved - proceeding with execution
⚙️ Executing motion...
✅ Motion completed successfully!
```

If cancelled:
```
❌ Action cancelled
```

### Example 4: Greeting (No Images)

**User:** "Hello!"

**Result:**
```
Hello! I'm Franka Assistant, your robot coordinator. I can help you 
inspect the workspace or manipulate objects. What would you like to do?
```

---

## 🔧 Key Components

### Web Handler (`web_handler.py`)

**Subscriptions:**
- `/web/request` - Incoming chat messages from browser
- `/coordinator_response` - LLM coordinator responses
- `/vlm/explanation` - VLM text explanations
- `/vlm_grounding` - Object detection with bbox
- `/motion/status` - Motion execution status

**Publications:**
- `/user_command` - Forward chat to LLM coordinator
- `/web/response` - Send messages/images back to browser
- `/web/status` - System status updates
- `/motion/user_confirmation` - User approval/rejection

**Key Features:**
- Loads debug images from `/debug_images/` directory
- Matches images to detected objects by name
- Encodes images as base64 for web display
- Manages confirmation flow with pending motion state

### Chat Module (`chat.js`)

**Features:**
- Displays text messages from robot/system
- Shows images inline with messages
- Handles confirmation requests
- Sends user messages and confirmations to ROS

**Message Types:**
- `message` - Regular chat message (optional image)
- `confirmation_request` - Triggers confirmation dialog
- `confirmation` - User's approval/rejection response

### Confirm Module (`confirm.js`)

**Features:**
- Creates modal dialog dynamically
- Styled for robot control context
- Returns promise with user decision
- Auto-closes after selection

---

## 🎨 Image Display

### How Images Work

1. **VLM saves images** to `/home/arash/franka-llm/debug_images/`
   - Format: `{object_name}_{model}_{timestamp}.jpg`
   - Examples:
     - `yellow_dice_qwen2.5vl-32b_Feb19_16-21-11.jpg`
     - `scene_description_qwen2.5vl-32b_Feb19_15-08-03.jpg`

2. **Web Handler loads images:**
   - Finds most recent image matching object name
   - Falls back to most recent image if no match
   - Encodes as base64 data URL

3. **Chat displays images:**
   - Rendered as `<img>` elements
   - Max width 100%, responsive
   - Border radius for rounded corners
   - Placed below message text

### Debug Image Types

- **Scene descriptions**: `scene_description_*.jpg`
- **Object detections**: `{object_name}_*.jpg` with red marker
- **Localization**: Object marked with red circle + depth value

---

## 📊 Status Monitoring

The Status tab shows:

| Card | Shows | Source |
|------|-------|--------|
| **Robot State** | Disconnected/Idle/Moving | Coordinator status |
| **Vision System** | Offline/Online | VLM activity |
| **LLM Engine** | Offline/Online | LLM responses |
| **ROS2 Bridge** | Offline/Online | WebSocket connection |

Updates every 1 second.

---

## 🐛 Troubleshooting

### "Cannot send message" Warning
- ROSBridge not running
- Check WebSocket connection: ws://localhost:9090

**Solution:**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Images Not Appearing
- Debug images not being saved
- Check VLM config: `debug.save_images: true`
- Check directory exists: `/home/arash/franka-llm/debug_images/`

**Solution:**
```bash
ls -la /home/arash/franka-llm/debug_images/
# Should show recent .jpg files
```

### Confirmation Dialog Not Showing
- Confirm module not initialized
- Check browser console for errors

**Solution:**
- Refresh browser page
- Check console logs

### Status Shows "Offline"
- Nodes not running
- Topics not published

**Solution:**
```bash
# Check running nodes
ros2 node list

# Should see:
# /llm_coordinator
# /vlm_node
# /web_handler

# Check topics
ros2 topic list | grep web
# Should see:
# /web/request
# /web/response
# /web/status
```

---

## 🔄 ROS Topics Reference

### Web → ROS2

| Topic | Type | Purpose |
|-------|------|---------|
| `/web/request` | String | Chat messages and confirmations |

**Chat Message Format:**
```json
{
  "type": "chat",
  "data": {
    "message": "pick up the red dice",
    "timestamp": "2026-02-19T15:30:00.000Z"
  }
}
```

**Confirmation Format:**
```json
{
  "type": "confirmation",
  "confirmed": true,
  "timestamp": "2026-02-19T15:30:05.000Z"
}
```

### ROS2 → Web

| Topic | Type | Purpose |
|-------|------|---------|
| `/web/response` | String | Messages and images to display |
| `/web/status` | String | System status updates |

**Response Format:**
```json
{
  "type": "message",
  "sender": "vlm",
  "message": "📍 Located: yellow dice at pixel (320, 240)",
  "image": "data:image/jpeg;base64,/9j/4AAQSkZJ...",
  "timestamp": "2026-02-19T15:30:02.000Z"
}
```

**Confirmation Request Format:**
```json
{
  "type": "confirmation_request",
  "sender": "system",
  "message": "🤖 Ready to pick yellow dice...",
  "action": "pick",
  "target": "yellow dice",
  "timestamp": "2026-02-19T15:30:03.000Z"
}
```

---

## 🚦 Development Tips

### Testing Without Robot

You can test the web interface with just:
1. ROSBridge
2. Web Handler
3. LLM Coordinator
4. VLM Agent

No physical robot needed for chat/vision testing!

### Adding New Features

**To add a new message type:**

1. Update `web_handler.py` to publish the new type
2. Update `chat.js` `setupROSSubscriptions()` to handle it
3. Add display logic in `addMessage()` or create new handler

**To modify confirmation flow:**

1. Edit `web_handler.py` `_request_motion_confirmation()`
2. Update `chat.js` `handleConfirmationRequest()`
3. Modify `confirm.js` modal styling/behavior

### Debugging

**Browser Console:**
```javascript
// Check ROS connection
window.ros.isConnected()

// View chat history
window.chatModule.getHistory()

// Manual confirmation test
window.confirmModule.show("Test message").then(r => console.log(r))
```

**ROS2 Commands:**
```bash
# Monitor web requests
ros2 topic echo /web/request

# Monitor web responses
ros2 topic echo /web/response

# Test web handler
ros2 topic pub /coordinator_response std_msgs/String "{data: 'Test message'}"
```

---

## 📝 Summary

You now have:
- ✅ **Streamlined UI** - Chat-focused interface
- ✅ **Full integration** - LLM Coordinator + VLM + Web Handler
- ✅ **Visual feedback** - Images from VLM shown inline
- ✅ **Safety confirmation** - User approval before motion
- ✅ **Real-time status** - System monitoring
- ✅ **No camera tab** - Removed redundant view

Enjoy your fully integrated web-based robot control system! 🤖🌐
