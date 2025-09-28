# Backend Gateway (Next.js API)

Backend service for device/task orchestration with Postgres (Prisma) and Redis Streams. The robotics middleware is not wired yet. The gateway already publishes events to a Redis Stream for later consumption.

- API docs and cURL examples: see README.API.md
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
  - user_id (FK -> users.user_id, cascade on delete)
  - created_at
  - updated_at
  - indexes: idx_device_user_id(user_id)

- **task_stacks**
  - stack_id (UUID, PK)
  - device_id (FK -> devices.device_id, cascade on delete)
  - tasks (JSONB) — array of discriminated tasks:
    - pick:  { "type": "pick",  "from": { x, y, z } }
    - place: { "type": "place", "to":   { x, y, z } }
  - status (TaskStackStatus, default 'pending')
  - created_at
  - updated_at
  - indexes: idx_taskstack_device_id(device_id), idx_taskstack_status(status)

### Seed data

- Provided by `02_seed.sql`: two demo users, a few devices, and completed stacks.

---

## Redis Streams integration

- Stream key: `task_stacks` (override via `TASK_STACK_STREAM`)
- On `POST /api/device/stack` the gateway persists the stack and `XADDs` an event:
  - fields: `event=task_stack.created, stackId, deviceId, userId`
- Middleware (when available) should `XREADGROUP` from this stream and transition stacks (e.g., to `in_progress/completed`).

---

## API endpoints (summary)

Authentication is Bearer JWT. Obtain a token via register or login.

- `POST /api/device/stack`
  - Create a new task stack for a device owned by the caller.
  - If `deviceId` omitted and the user owns exactly one device, it is inferred.
  - Returns 201 `{ stackId, status: 'pending' }`
  - Publishes to Redis stream `task_stacks`.

- `GET /api/device/summary`
  - Lists all user devices with counts:
    - pendingTasks: stacks with status `'pending'`
    - activeTasks: stacks with status `'in_progress'`

Details, validation, and cURL examples: README.API.md

---

## Environment variables

From `.env.example`, derive `.env` and use for `docker-compose`:

- `JWT_SECRET`: secret for signing JWTs
- `JWT_EXPIRES_IN`: token TTL (e.g., 7d)
- `POSTGRES_USER`, `POSTGRES_PASSWORD`, `POSTGRES_DB`: database bootstrap values
- `GATEWAY_PORT`: host port to expose the gateway (default 3000)

App-only (with defaults in code/compose):

- `DATABASE_URL`: injected by compose for the backend container
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

Use the cURL snippets in README.API.md. Default base URL: <http://localhost:3000> (or <http://localhost:$GATEWAY_PORT>).

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

- The robotics middleware is not running yet. Stacks will remain `'pending'` until a consumer processes the stream and updates statuses.
