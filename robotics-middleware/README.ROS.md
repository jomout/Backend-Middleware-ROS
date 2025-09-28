# ROS communication report

This document describes how the Robotics Middleware and robot nodes communicate over ROS 2. It captures the topics, QoS, message formats, and expected request/response sequence. It is based on the current implementation in `app/ros/middleware_node.py` and the demo robots in `../ros_robots/`.

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

**Robot 1** (Belongs to user1 - Inserted via `02_seed.sql`):

- Command topic: `COMMAND_TOPIC_1` (default: `/robot_1/commands`)
- Feedback topic: `FEEDBACK_TOPIC_1` (default: `/robot_1/feedback`)

**Robot 2** (Belongs to user1 - Inserted via `02_seed.sql`):

- Command topic: `COMMAND_TOPIC_2` (default: `/robot_2/commands`)
- Feedback topic: `FEEDBACK_TOPIC_2` (default: `/robot_2/feedback`)

**QoS settings** (both directions):

- Reliability: RELIABLE
- History: KEEP_LAST
- Depth: 10

Message type: `std_msgs/String` with a JSON object in `data`.

Environment variables configure topic names. See `app/core/config.py` and `.env.example`:

- `COMMAND_TOPIC_1`, `FEEDBACK_TOPIC_1`, `COMMAND_TOPIC_2`, `FEEDBACK_TOPIC_2`

---

## Message schemas

All fields are case-sensitive. Unknown/extra fields are ignored by the current middleware/robot demos.

**Command** (published by middleware to a robot command topic):

- `deviceName` (string, required): target robot (e.g., `"robot_1"` or `"robot_2"`). Used to select the topic in middleware. **For more scalability change to `deviceId`.**
- `event` (string, required): must be `"task.execute"`.
- `stackId` (string, required): UUID of the task stack. Correlation key.
- `taskIndex` (integer, required): 0-based index of the task in the stack. Correlation key.
- `task` (object, required): task details; current types are `"pick"` and `"place"` with your domain-specific payload.

**Feedback** (published by robot to its feedback topic):

- `event` (string, required): `"task.completed"` on success or `"task.failed"` on failure.
- `deviceName` (string, required): robot name (demo robots use `"robot_1"`/`"robot_2"`). **For more scalability change to `deviceId`.**
- `stackId` (string, required): must match the command.
- `taskIndex` (integer, required): must match the command.
- `error` (string, optional): In case of error, error field is provided.

Correlation: Middleware matches feedback to the pending command using the tuple `(stackId, taskIndex)`.

---

## Request/response sequence

1. Middleware publishes a `task.execute` command to the appropriate robot command topic based on `deviceName`.
2. Robot receives command, executes the task (simulated in demos), then publishes feedback to its feedback topic.
3. Middleware receives feedback; if `event == "task.completed"`, the task is considered successful. Any other event or a timeout marks the task as failed.

Timeouts: Middleware waits up to 20 seconds per task by default (see `execute_task_stack(..., timeout=20.0)`). A timeout is treated as failure.

---

## Examples

### Single task on robot_1 (success)

Command (published by middleware to `/robot_1/commands`):

```json
{
  "deviceName": "robot_1",
  "event": "task.execute",
  "stackId": "4a3b5a3e-31ce-4a2a-8a5f-40c5d2e6f9b9",
  "taskIndex": 0,
  "task": {
    "type": "pick",
    "payload": { "x": 1.0, "y": 2.0, "z": 0.1 }
  }
}
```

**Feedback - Success** (published by robot to `/robot_1/feedback`):

```json
{
  "event": "task.completed",
  "deviceId": "robot_1",
  "stackId": "4a3b5a3e-31ce-4a2a-8a5f-40c5d2e6f9b9",
  "taskIndex": 0
}
```

**Feedback - Failed** (published by robot to `/robot_1/feedback`):

```json
{
  "event": "task.failed",
  "deviceId": "robot_1",
  "stackId": "4a3b5a3e-31ce-4a2a-8a5f-40c5d2e6f9b9",
  "taskIndex": 0,
  "error": "Task failed"
}
```

### Multi-task stack (two tasks)

Task 0 (pick on robot_1):

- Command -> `/robot_1/commands` with `taskIndex: 0`
- Feedback <- `/robot_1/feedback` with `event: task.completed`, `taskIndex: 0`

Task 1 (place on robot_1):

- Command -> `/robot_1/commands` with `taskIndex: 1`
- Feedback <- `/robot_1/feedback` with `event: task.completed`, `taskIndex: 1`

Middleware treats the stack as successful only if all tasks complete. Any `task.failed` or timeout fails the stack.

---

## Error modes and edge cases

- Non-JSON message: ignored; a warning is logged (middleware and demo robots both guard with JSON parsing).
- Unknown `deviceName` (in command): middleware logs an error and does not publish.
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

- Configure topic names via env vars in `.env` or container environment. Defaults are:

  - `/robot_1/commands`, `/robot_1/feedback`
  - `/robot_2/commands`, `/robot_2/feedback`

---

## Implementation references

- Middleware node: `app/ros/middleware_node.py`
- Demo robots: `../ros_robots/robot_1_node.py`, `../ros_robots/robot_2_node.py`
- DTO schemas: `app/core/dto.py` (Command, Feedback)
