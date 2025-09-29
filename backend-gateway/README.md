# Backend Gateway (Next.js API)

Backend service for device/task orchestration with Postgres - Prisma and Redis Streams. The robotics middleware is not wired yet. The gateway already publishes events to a Redis Stream for later consumption.

- API docs and cURL examples: see [README.API.md](README.API.md)
- Production deploy: docker-compose-backend-prod.yml (db + redis + gateway)

---

## Tech stack

- Next.js App Router (API routes only)
- Prisma ORM + PostgreSQL
- Auth: JWT, bcrypt password hashing
- Validation: Zod with unified error payloads
- Redis Streams for task delivery (notification - webhook) to the (future) robotics middleware
- Structured JSON logging to stdout with configurable level

---

## Data model (Postgres/Prisma)

Prisma models: see `prisma/schema.prisma`. Database is created by SQL in `../database/*.sql` mounted by the compose file.

### Enums

1. **DeviceStatus:** 'online' | 'offline'
2. **TaskStackStatus:** 'pending' | 'in_progress' | 'completed' | 'failed'

### Tables

- **users**
  - user_id (UUID, PK)
  - email (unique)
  - name
  - password (bcrypt hash)
  - created_at
  - updated_at

- **devices**
  - device_id (UUID, PK)
  - name
  - status (DeviceStatus)
  - user_id (FK)
  - created_at
  - updated_at

- **task_stacks**
  - stack_id (UUID, PK)
  - device_id (FK)
  - tasks (JSONB) — array of discriminated tasks:
    - pick:  { "type": "pick",  "from": { x, y, z } }
    - place: { "type": "place", "to":   { x, y, z } }
  - status (TaskStackStatus, default 'pending')
  - created_at
  - updated_at

### Seed data

Provided by `02_seed.sql`: two demo users, a few devices, and completed stacks.Injects initial records for testing and demonstration.

#### Users

Two sample users are created with bcrypt-hashed passwords:

- **user1@example.com** (`user1`)
- **user2@example.com** (`user2`)

#### Devices

Each user is associated with one or more robots:

- **User 1**
  - `robot_1` (ID: `18f2d2a4-d391-4346-8dc2-2f195b05d52a`) – **online**
  - `robot_3` (ID: `ab1e7619-b3d1-474d-9d95-90e84598ee7d`) – **offline**
- **User 2**
  - `robot_2` (ID: `a3369c60-c84d-49d5-85b8-972d007f8933`) – **online**

#### Task Stacks

Predefined task stacks simulate completed workflows:

- For **User 1 / robot_1**:  
  A ***completed*** stack containing:
  1. `pick` from `{x:1, y:2, z:3}`
  2. `place` to `{x:4, y:5, z:6}`

- For **User 2 / robot_2**:  
  A ***completed*** stack containing:
  1. `place` to `{x:7, y:8, z:9}`

---

## Redis Streams integration

Redis Streams is used as a Notification - Webhook system to notify the middleware that a Task stack has been created, and needs to be processed.

- Stream key: `task_stacks` (override via `TASK_STACK_STREAM`)
- On `POST /api/device/stack` the gateway persists the stack and `XADDs` an event:
  - fields: `event=task_stack.created, stackId, deviceId, userId`
- Middleware (when available) should `XREADGROUP` from this stream and transition stacks (e.g., to `in_progress/completed`).

---

## API endpoints (summary)

First step before any API call, is the Authentication of the user.
Authentication is Bearer JWT. Obtain a token via **register** or **login**.

`POST /api/device/stack`

- Create a new task stack for a device owned by the caller.
- If `deviceId` omitted and the user owns exactly one device, it is inferred.
- Returns 201 `{ stackId, status: 'pending' }`
- Publishes to Redis stream `task_stacks`.

`GET /api/device/summary`

- Lists all user devices with counts:
- pendingTasks: stacks with status `'pending'`
- activeTasks: stacks with status `'in_progress'`

Details, validation, and cURL examples: [README.API.md](README.API.md)

---

## Environment variables

Environmental Variables examples are located in `.env.example`.
From `.env.example`, derive `.env` and use for `docker-compose`:

#### Database

- `POSTGRES_USER`, `POSTGRES_PASSWORD`, `POSTGRES_DB`: database bootstrap values.

#### Backend

- `JWT_SECRET`: secret for signing JWTs
- `JWT_EXPIRES_IN`: token TTL (e.g., 7d)
- `GATEWAY_PORT`: host port to expose the gateway (default 3000)
- `DATABASE_URL`: injected by compose for the backend container, iniialized by `POSTGRES_USER`, `POSTGRES_PASSWORD`, `POSTGRES_DB`.
- `REDIS_URL`: `redis://HOST:6379` (compose sets `redis://redis:6379`)
- `TASK_STACK_STREAM`: Redis stream key (default `task_stacks`)
- `LOG_LEVEL`: `debug` | `info` | `warn` | `error` (default: `debug` in dev, `info` in prod)

---

## Production deploy with Docker Compose

The file `docker-compose-backend-prod.yml` defines three services: db (Postgres), redis, backend (Next.js API).

### Prepare environment file

- Copy and edit vars as needed:
  - `cp backend-gateway/.env.example backend-gateway/.env`
  - set environment variables in `.env`

### Start the stack

```zsh
docker compose -f backend-gateway/docker-compose-backend-prod.yml up -d --build
```

### What happens

- Postgres starts with a volume (pgdata) and runs init scripts:
  - `../database/00_extensions.sql` (pgcrypto)
  - `../database/01_init.sql` (tables, enums, triggers, indexes)
  - `../database/02_seed.sql` (demo users/devices/stacks)
- Redis starts and becomes healthy.
- Backend builds, connects to db/redis, and listens on host port `${GATEWAY_PORT:-3000}`.

### Health and logs

```zsh
# Wait for health checks and confirm
docker compose -f backend-gateway/docker-compose-backend-prod.yml ps

# Tail logs (Ctrl+C to stop)
docker compose -f backend-gateway/docker-compose-backend-prod.yml logs -f backend
```

### Test the API

Use the cURL snippets in [README.API.md](README.API.md). Default base URL: <http://localhost:3000> (or <http://localhost:$GATEWAY_PORT>).

### Inspect Redis stream

```zsh
# Enter redis shell
docker exec -it redis redis-cli

# Show stream info
XINFO STREAM task_stacks
```

### Stop and clean up

```zsh
docker compose -f backend-gateway/docker-compose-backend-prod.yml down -v
```

---

## Notes

The robotics middleware is not running yet. Stacks will remain `'pending'` until a consumer processes the stream and updates statuses.
