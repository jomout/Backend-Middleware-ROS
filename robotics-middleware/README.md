# Robotics Middleware

A FastAPI-based worker service that consumes task events from Redis Streams, coordinates execution (optionally via ROS 2), and updates task statuses in Postgres. It works alongside the Backend Gateway, which writes task stacks and emits events to the `task_stacks` Redis stream.

This middleware can run in two modes:

- **Non-ROS mode**: simulates execution and marks stacks completed/failed.
- **ROS mode**: starts a background ROS 2 executor and dispatches tasks to a ROS node.

Note: In Docker Compose, ROS is enabled by default via `ROS_ENABLED: ${ROS_ENABLED:-true}`. Locally, the default in code is `False` unless set in `.env`.

---

## What it does

Subscribes to a Redis Stream (default: `task_stacks`) using a consumer group (default: `robots`).

For each `task_stack.created` event:

- Checks the device status (must be `online`).
- Loads the corresponding task stack (`pending`) from database.
- Transitions it to `in_progress` and executes:
  - **Non-ROS mode**: logs simulated commands, then marks `completed` or `failed`.
  - **ROS mode**: uses a ROS 2 bridge (`app/core/ros_bridge.py`) to invoke `app/ros/middleware_node.py`, then marks `completed` or `failed`.
- Acknowledges the stream message after processing (success or failure). There are no automatic retries from the stream, failures are reflected in the DB status.

Key modules:

- `app/main.py`: FastAPI app lifecycle (starts/stops the consumer and ROS bridge).
- `app/core/redis_consumer.py`: Redis XREADGROUP loop, message handling, status checks.
- `app/core/worker.py`: simulated executor (non-ROS).
- `app/core/ros_worker.py`: ROS-backed executor.
- `app/core/ros_bridge.py`: background ROS 2 executor thread + task dispatch helpers.
- `app/core/models.py`: SQLAlchemy models aligned with the DB schema used by the gateway.
- `app/core/config.py`: Pydantic Settings with `.env` support.

---

## Configuration

Environment variables (see `.env.example`):

- Gateway (used by the Backend Gateway service via Compose)
  - `JWT_SECRET`: secret used by the gateway to sign tokens
  - `JWT_EXPIRES_IN`: token TTL for the gateway
  - `GATEWAY_PORT` (default: `3000`): host port to expose the gateway
  - `LOG_LEVEL` (default: `info`): gateway and middleware log level

- Database (Compose inputs for both services)
  - `POSTGRES_USER` (default in example: `user`)
  - `POSTGRES_PASSWORD` (default in example: `password`)
  - `POSTGRES_DB` (default in example: `db`)

- Database (middleware standalone)
  - `DATABASE_URL` (SQLAlchemy format): e.g. `postgresql+psycopg://user:pass@host:5432/db`
    - Compose injects the correct `DATABASE_URL` for the middleware from `POSTGRES_*`. Set `DATABASE_URL` only when running outside Compose.

- Redis
  - `REDIS_URL` (default: `redis://localhost:6379` locally; `redis://redis:6379` in Compose)
  - `TASK_STACK_STREAM` (default: `task_stacks`)
  - `TASK_STACK_CONSUMER_GROUP` (default: `robots`)
  - `TASK_STACK_CONSUMER_NAME` (default: `worker-1`)

- ROS and devices
  - `ROS_ENABLED` (default in code: `false`; Compose default: `true`)
  - `DEVICES`: JSON list mapping device UUIDs to topics, for example:

    `[ {"device_id": "<uuid>", "command_topic": "/robot_1/commands", "feedback_topic": "/robot_1/feedback" }, ... ]`

    The middleware uses this mapping to publish commands and subscribe to feedback per device. See `.env.example` for two sample devices and `DEVICE_1`/`DEVICE_2` used by demo robots.

Notes:

- Place a `.env` file in the `robotics-middleware/` folder. `app/core/config.py` loads from `.env` automatically.
- The DB models expect the same schema as the gateway’s `prisma/schema.prisma` and SQL init.
- JSON values inside `.env` must be quoted. The example shows single-quoted values for convenience.

Additional notes on URLs:

- Locally, `REDIS_URL` typically uses `redis://localhost:6379`. In Docker Compose, services use `redis://redis:6379`.
- The middleware accepts SQLAlchemy-style `DATABASE_URL` (e.g., `postgresql+psycopg://user:pass@host:5432/db`). The gateway uses Prisma-style (e.g., `postgresql://user:pass@host:5432/db`). Compose sets the correct format for each service automatically.

---

## Run with Docker Compose (gateway + middleware + demo robots)

This folder includes a `docker-compose.yml` that starts:

- Postgres (`db`) initialized from `../database/*.sql`
- Redis (`redis`)
- Backend Gateway (`backend`) exposed on `${GATEWAY_PORT:-3000}`
- Robotics Middleware (`middleware`) internal-only (no port exposed by default)
- Two demo ROS containers (`robot_1`, `robot_2`) that simulate robots

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

- Demo robot containers build from `../ros_robot` and run the same `robot_node.py` with different `DEVICE` JSON env values (`DEVICE_1`, `DEVICE_2`).
- The Gateway and Middleware share Postgres and Redis via the internal network configured by Compose.

---

## Demo robots (`ros_robot/`)

This repo includes a minimal demo robot node used by both containers:

- `../ros_robot/robot_node.py`

Each robot container is passed a different `DEVICE` JSON via environment variables (`DEVICE_1` or `DEVICE_2`) to point it at its command/feedback topics.

These are reference/demo nodes. Adapt them to match your robots’ interfaces or replace them entirely in production.

---

## Development notes

See also: the ROS communication contract and message examples in [README.ROS.md](./README.ROS.md).

- Logging uses Uvicorn’s logger (`uvicorn.error`) via `app/core/logging.py`.
- Non-ROS execution simulates ~1s per task. Adjust in `app/core/worker.py`.
- ROS device mapping is driven by `DEVICES` (device UUID -> topics) and handled by `app/ros/middleware_node.py`.
- Consumer group creation is idempotent (`XGROUP CREATE MKSTREAM`); BUSYGROUP errors are ignored.

---

## End-to-end flow (with gateway)

1. Client calls the gateway: `POST /api/device/stack`, DB insert (status `pending`) and `XADD` on `task_stacks`.
2. Middleware consumes the stream entry:

    - Validates device online status
    - Loads `pending` stack
    - Marks `in_progress`
    - Executes (ROS or simulated)
    - Updates to `completed` or `failed`
    - Always `XACK` the message after processing

If the middleware is offline, entries remain in the stream and will be processed when it starts.

---

## Assumptions

- We assume that the robots marked as **online** in the database correspond to the ones that are actually launched in containers. This is why their IDs are the same across the system.

- In a real-world setup, the robots and the middleware would be deployed on a separate network from the rest of the system. For demonstration purposes, this separation was omitted.

---

## API reference

For the endpoints and end-to-end pipeline cURL examples, see the Backend Gateway API docs:

- [Backend Gateway API](../backend-gateway/README.API.md)
