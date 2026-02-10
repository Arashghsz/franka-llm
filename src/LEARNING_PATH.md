# 🚀 Learning Path: LLM + VLM Multi-Agent Pick & Place

##  Goal
Build a multi-agent robotic system:
- **LLM** → Task Planner (high-level reasoning)
- **VLM/VLA** → Safety Agent (visual verification)
- **cuMotion** → Motion Planning (GPU-accelerated)
- **Franka FR3** → Execution
---

### Phase: LLM Integration
**Goal**: Natural language to robot actions

**Architecture Options**:

1. **Direct Prompting** (Simple)
```
User: "Pick up the red cup"
LLM → Parse → {"action": "pick", "object": "red cup"}
```

2. **Code Generation** (More flexible)
```
User: "Pick up the red cup and place it on the shelf"
LLM → Generate Python code → Execute
```

3. **ReAct Pattern** (Reasoning + Acting)
```
LLM thinks: "I need to locate the red cup first"
LLM acts: call_perception("find red cup")
LLM observes: "Red cup at position (0.5, 0.2, 0.1)"
LLM thinks: "Now I can pick it"
LLM acts: call_motion("pick", position)
```

**LLM Options**:
- OpenAI GPT-4 (API)
- Claude (API)
- LLaMA 3 (local on Jetson!)
- Ollama (easy local deployment)

---
**VLM Options for Jetson**:
- LLaVA (can run locally)
- VILA (NVIDIA's VLM)
- GPT-4V (API)

**Safety Checks**:
```python
# Before executing motion
def safety_check(image, planned_action):
    prompt = f"""
    Image shows the robot workspace.
    Planned action: {planned_action}
    
    Is this safe to execute? Check for:
    1. Obstacles in path
    2. Human presence
    3. Object still in expected position
    
    Respond: SAFE or UNSAFE with reason
    """
    return vlm.query(image, prompt)
```
