# ROS communication report

This document describes how the Robotics Middleware and robot nodes communicate over ROS 2. It captures the topics, QoS, message formats, and expected request/response sequence. It is based on the current implementation in `app/ros/middleware_node.py` and the demo robots in `../ros_robot/`.

---

## Roles

- Middleware
  - Publishes command messages to robot-specific command topics.
  - Subscribes to robot feedback topics and correlates responses to pending tasks.

- Robot node (one per robot)
  - Subscribes to its command topic.
  - Executes commands, then publishes feedback (`task.completed` or `task.failed`).

Both sides use ROS 2 `std_msgs/String` with JSON-encoded payloads.

---

## Topics and QoS

Topics are configured per device via the `DEVICES` environment variable. The example Compose setup includes two demo devices:

**Robot 1** (inserted via `02_seed.sql`):

- Command topic: `COMMAND_TOPIC_1` (default: `/robot_1/commands`)
- Feedback topic: `FEEDBACK_TOPIC_1` (default: `/robot_1/feedback`)

**Robot 2** (inserted via `02_seed.sql`):

- Command topic: `COMMAND_TOPIC_2` (default: `/robot_2/commands`)
- Feedback topic: `FEEDBACK_TOPIC_2` (default: `/robot_2/feedback`)

**QoS settings** (both directions):

- Reliability: RELIABLE
- History: KEEP_LAST
- Depth: 10

Message type: `std_msgs/String` with a JSON object in `data`.

Environment variables configure device->topic mapping. See `app/core/config.py` and `.env.example`:

- `DEVICES` (array of objects with `device_id`, `command_topic`, `feedback_topic`)

---

## Message schemas

All fields are case-sensitive. Unknown/extra fields are ignored by the current middleware/robot demos.

**Command** (published by middleware to a robot command topic):

- `event` (string, required): must be `"task.execute"`.
- `stack_id` (string, required): UUID of the task stack. Correlation key.
- `task_index` (integer, required): 0-based index of the task in the stack. Correlation key.
- `task` (object, required): task details; current types are `"pick"` and `"place"` with your domain-specific payload.

**Feedback** (published by robot to its feedback topic):

- `event` (string, required): `"task.completed"` on success or `"task.failed"` on failure.
- `stack_id` (string, required): must match the command.
- `task_index` (integer, required): must match the command.
- `error` (string, optional): In case of error, error field is provided.

Correlation: Middleware matches feedback to the pending command using the tuple `(stack_id, task_index)`.

---

## Request/response sequence

1. Middleware publishes a `task.execute` command to the robot command topic associated with the target `device_id`.
2. Robot receives command, executes the task (simulated in demos), then publishes feedback to its feedback topic.
3. Middleware receives feedback; if `event == "task.completed"`, the task is considered successful. Any other event or a timeout marks the task as failed.

Timeouts: Middleware waits up to 20 seconds per task by default (see `execute_task_stack(..., timeout=20.0)`). A timeout is treated as failure.

---

## Examples

### Single task (success)

Command (published by middleware to `/robot_1/commands` for the target device):

```json
{
  "event": "task.execute",
  "stack_id": "4a3b5a3e-31ce-4a2a-8a5f-40c5d2e6f9b9",
  "task_index": 0,
  "task": {
    "type": "pick",
    "payload": { "x": 1.0, "y": 2.0, "z": 0.1 }
  }
}
```

**Feedback - Success** (published by robot to its feedback topic):

```json
{
  "event": "task.completed",
  "stack_id": "4a3b5a3e-31ce-4a2a-8a5f-40c5d2e6f9b9",
  "task_index": 0
}
```

**Feedback - Failed** (published by robot to its feedback topic):

```json
{
  "event": "task.failed",
  "stack_id": "4a3b5a3e-31ce-4a2a-8a5f-40c5d2e6f9b9",
  "task_index": 0,
  "error": "Task failed"
}
```

### Multi-task stack (two tasks)

Task 0 (pick):

- Command -> device's command topic with `taskIndex: 0`
- Feedback <- device's feedback topic with `event: task.completed`, `taskIndex: 0`

Task 1 (place):

- Command -> device's command topic with `taskIndex: 1`
- Feedback <- device's feedback topic with `event: task.completed`, `taskIndex: 1`

Middleware treats the stack as successful only if all tasks complete. Any `task.failed` or timeout fails the stack.

---

## Error modes and edge cases

- Non-JSON message: ignored. A warning is logged (middleware and demo robots both guard with JSON parsing). Because of timeout configuration, the stack gets updated in the database as status `failed`.
- Unknown `device_id` (no publisher configured): middleware logs an error and does not publish.
- Unknown task `type`: middleware logs an error and fails the stack.
- Feedback with missing `stackId`/`taskIndex`: ignored for correlation.
- Timeout waiting for feedback: treated as failure (per-task timeout).

---

## Notes

- Observe topics, **after connecting to a running container**:

```zsh
ros2 topic echo /robot_1/commands
ros2 topic echo /robot_1/feedback
```

- Configure topics per device via `DEVICES` env var in `.env` or container environment. Example defaults are `/robot_1/commands` and `/robot_1/feedback` for the first device, and `/robot_2/commands` and `/robot_2/feedback` for the second.

---

## Implementation references

- Middleware node: `app/ros/middleware_node.py`
- Demo robot: `../ros_robot/robot_node.py` (used by both demo containers)
- DTO schemas: `app/core/dto.py` (Command, Feedback)
