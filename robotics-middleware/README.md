# Robotics Middleware

A FastAPI-based worker service that consumes task events from Redis Streams, coordinates execution (optionally via ROS 2), and updates task statuses in Postgres. It’s designed to work with the Backend Gateway, which writes task stacks and emits events to the `task_stacks` Redis stream.

This middleware can run in two modes:

- Non-ROS mode (default): simulates execution and marks stacks completed/failed.
- ROS mode: starts a background ROS 2 executor and dispatches tasks to a ROS node.

---

## What it does

- Subscribes to a Redis Stream (default: `task_stacks`) using a consumer group (default: `robots`).
- For each `task_stack.created` (or `task_stack.pending`) event:
  - Checks the device status (must be `online`).
  - Loads the corresponding task stack (`pending`).
  - Transitions it to `in_progress` and executes:
    - Non-ROS mode: logs simulated commands, then marks `completed` or `failed`.
    - ROS mode: uses a ROS 2 bridge (`app/core/ros_bridge.py`) to invoke `app/ros/middleware_node.py`, then marks `completed` or `failed`.
- Acknowledges the stream message only after successful processing; errors cause retries.

Key modules:

- `app/core/redis_consumer.py`: Redis XREADGROUP loop, message handling, ownership/status checks.
- `app/core/worker.py`: simulated executor (non-ROS).
- `app/core/ros_worker.py`: ROS-backed executor.
- `app/core/ros_bridge.py`: background ROS 2 executor thread + task dispatch helpers.
- `app/core/models.py`: SQLAlchemy models aligned with the Postgres schema used by the gateway.
- `app/core/config.py`: Pydantic Settings with `.env` support.

---

## Configuration

Environment variables (see `.env.example`):

- Gateway (used by the Backend Gateway via Compose)
  - `JWT_SECRET` (default in example): secret used by the gateway to sign tokens
  - `JWT_EXPIRES_IN` (default in example): token TTL for the gateway
  - `GATEWAY_PORT` (default in example: `3000`): host port to expose the gateway
  - `LOG_LEVEL` (default in example: `info`): gateway log level

- Database (Compose inputs for both services)
  - `POSTGRES_HOST` (default in example: `localhost`)
  - `POSTGRES_USER` (default in example: `user`)
  - `POSTGRES_PASSWORD` (default in example: `password`)
  - `POSTGRES_DB` (default in example: `db`)

- Database (middleware standalone)
  - `DATABASE_URL` (SQLAlchemy format): e.g. `postgresql+psycopg://user:pass@host:5432/db`
    - Note: The Compose file constructs and injects the correct `DATABASE_URL` for the middleware based on the `POSTGRES_*` variables. You only need to set `DATABASE_URL` when running the middleware outside Compose.

- Redis
  - `REDIS_URL` (default: `redis://localhost:6379` locally; `redis://redis:6379` in Compose)
  - `TASK_STACK_STREAM` (default: `task_stacks`)
  - `TASK_STACK_CONSUMER_GROUP` (default: `robots`)
  - `TASK_STACK_CONSUMER_NAME` (default: `worker-1`)

- ROS
  - `ROS_ENABLED` (default: `false`): if `true`, starts the ROS 2 bridge and uses `ros_worker`
  - `COMMAND_TOPIC_1`, `FEEDBACK_TOPIC_1`, `COMMAND_TOPIC_2`, `FEEDBACK_TOPIC_2`: topic names used by the ROS node(s)

Notes:

- Place a `.env` file in the `robotics-middleware/` folder. `app/core/config.py` loads from `.env` automatically.
- The DB models expect the same schema as the gateway’s `prisma/schema.prisma` and SQL init.

Additional notes on URLs:

- When running locally, `REDIS_URL` typically uses `redis://localhost:6379`. In Docker Compose, services use the internal host `redis://redis:6379`.
- The middleware accepts SQLAlchemy-style `DATABASE_URL` (e.g., `postgresql+psycopg://user:pass@host:5432/db`). The gateway uses Prisma-style (e.g., `postgresql://user:pass@host:5432/db`). The compose file sets the correct format for each service automatically.

---

## Run with Docker Compose (gateway + middleware + demo robots)

This folder includes a `docker-compose.yml` that starts:

- Postgres (`db`) initialized from `../database/*.sql`
- Redis (`redis`)
- Backend Gateway (`backend`) exposed on `${GATEWAY_PORT:-3000}`
- Robotics Middleware (`middleware`) internal-only (no port exposed by default)
- Two demo ROS containers (`robot_1`, `robot_2`) that simulate robot topics

### Quick start

```zsh
# from robotics-middleware/
cp -n .env.example .env  # review and adjust values as needed
docker compose up -d --build

# follow logs
docker compose logs -f middleware
docker compose logs -f backend
```

### Notes

- The demo ROS containers mount `../ros_robots/robot_1_node.py` and `../ros_robots/robot_2_node.py` and use topic env vars from `.env`.
- The Gateway and Middleware share the same Postgres and Redis via the internal network configured by compose.

---

## Demo robots (`ros_robots/`)

This repository includes two minimal demo robot nodes to help with local testing:

- `../ros_robots/robot_1_node.py`
- `../ros_robots/robot_2_node.py`

They run in separate containers (`robot_1`, `robot_2`) based on `ros:jazzy-ros-core` and execute the scripts above. Topic names are provided via environment variables from `.env` and mapped in `docker-compose.yml`:

- Robot 1 uses `COMMAND_TOPIC_1` and `FEEDBACK_TOPIC_1` (defaults: `/robot_1/commands`, `/robot_1/feedback`).
- Robot 2 uses `COMMAND_TOPIC_2` and `FEEDBACK_TOPIC_2` (defaults: `/robot_2/commands`, `/robot_2/feedback`).

These are reference/demo nodes. Adapt them to match your real robots’ interfaces or replace them entirely in production.

---

## Development notes

See also: the ROS communication contract and message examples in [README.ROS.md](./README.ROS.md).

- Logging uses Uvicorn’s logger (`uvicorn.error`) via `app/core/logging.py`.
- Non-ROS execution simulates 1s per task. Adjust in `worker.py`.
- ROS mapping: device names are mapped to `robot_1`/`robot_2` namespaces in `ros_worker.py`.
- Consumer group creation is idempotent (`XGROUP CREATE MKSTREAM`); BUSYGROUP errors are ignored.

---

## End-to-end flow (with gateway)

1. Client calls the gateway: `POST /api/device/stack`, DB insert (status `pending`) and `XADD` on `task_stacks`.
2. Middleware consumes the stream entry:

    - Validates device online status and ownership
    - Loads `pending` stack
    - Marks `in_progress`
    - Executes (ROS or simulated)
    - Updates to `completed` or `failed`
    - `XACK` message

If the middleware is offline, entries remain in the stream and will be processed when it starts.

---

## API reference

For the **full set of endpoints** and **end-to-end pipeline cURL examples**, see the Backend Gateway API docs:

- [Backend Gateway API](../backend-gateway/README.API.md)
