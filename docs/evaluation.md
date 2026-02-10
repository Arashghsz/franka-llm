# Evaluation Metrics and Tools

## Task Success Metrics

### Primary Metrics
- **Task Completion Rate**: % of requested tasks completed successfully
- **Trajectory Quality**: Smoothness and efficiency of executed paths
- **Object Detection Accuracy**: YOLO detection precision and recall
- **Planning Time**: Time taken to plan motion
- **Execution Time**: Actual motion execution duration

## Evaluation Categories

### 1. Perception Evaluation
- YOLO detection precision, recall, F1-score
- Segmentation accuracy
- False positive/negative rates

### 2. Planning Evaluation
- Collision-free path success rate
- Plan computation time
- Path length and smoothness

### 3. Execution Evaluation
- Trajectory tracking error
- Task completion time
- Gripper success rate

### 4. End-to-End Evaluation
- Task success rate
- Total pipeline latency
- User satisfaction metrics

## Metrics Collection

- Use `franka_eval_tools` to log metrics
- Store evaluation results in timestamped files
- Generate reports for analysis

## Benchmarks

| Task | Target Completion Time | Min Success Rate |
|------|----------------------|-----------------|
| Pick object | < 30s | 90% |
| Place object | < 20s | 90% |
| Grasp recovery | < 10s | 85% |

