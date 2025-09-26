# API Reference

This document describes the available REST endpoints under `src/app/api`.

- Base URL (dev): `http://localhost:3000`
- Authentication: JWT in `Authorization: Bearer <token>` (obtain via `/api/auth/register` or `/api/auth/login`).
- Content type: `application/json`
- Error format (unified):

  ```json
  {
    "error": "Human-readable message",
    "details": { "optional": "validation details or context" }
  }
  ```

---

## Authentication

Use JWT Bearer tokens for authenticated requests. First register or log in to obtain a token, then include it in the `Authorization` header.

- Register a new user

  - Endpoint: `POST /api/auth/register`
  - Body:

    ```json
    { "email": "alice@example.com", "password": "ChangeMe123!", "name": "Alice" }
    ```

  - Responses:
    - 201 Created: `{ "token": "<jwt>" }`
    - 400 Validation error
    - 409 Email already registered

  - cURL:

    ```zsh
    curl -X POST http://localhost:3000/api/auth/register \
      -H 'Content-Type: application/json' \
      -d '{"email":"alice@example.com","password":"ChangeMe123!","name":"Alice"}'
    ```

- Log in

  - Endpoint: `POST /api/auth/login`
  - Body:

    ```json
    { "email": "alice@example.com", "password": "ChangeMe123!" }
    ```

  - Responses:
    - 200 OK: `{ "token": "<jwt>" }`
    - 400 Validation error
    - 401 Invalid credentials

  - cURL:

    ```zsh
    curl -X POST http://localhost:3000/api/auth/login \
      -H 'Content-Type: application/json' \
      -d '{"email":"alice@example.com","password":"ChangeMe123!"}'
    ```

- Get current user (test token)

  - Endpoint: `GET /api/auth/me`
  - Header: `Authorization: Bearer <token>`
  - Response: `{ "userId": "...", "email": "..." }`

  - cURL:

    ```zsh
    curl http://localhost:3000/api/auth/me \
      -H 'Authorization: Bearer <jwt>'
    ```

## POST /api/device/stack

Create a new task stack for a device owned by the authenticated user.

- Auth: required (`Authorization: Bearer <token>`)
- Body JSON:

  ```json
  {
    "deviceId": "dev_1", // optional if the user owns exactly one device
    "tasks": [
      { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } },
      { "type": "place", "to": { "x": 4, "y": 5, "z": 6 } }
    ]
  }
  ```

- Validation (Zod):
  - `deviceId` optional string; required if the user owns multiple devices.
  - `tasks` is a non-empty array of discriminated union on `type`:
    - `pick`: `{ type: 'pick', from: { x: number, y: number, z: number } }`
    - `place`: `{ type: 'place', to: { x: number, y: number, z: number } }`

- Responses:
  - 201 Created

    ```json
    { "stackId": "stack_abc123", "status": "pending" }
    ```

  - 400 Bad Request (validation or missing deviceId when needed)

    ```json
    { "error": "Validation error", "details": { "fieldErrors": { "tasks": ["..."] } } }
    ```

  - 401 Unauthorized

    ```json
    { "error": "Unauthorized" }
    ```

  - 403 Forbidden (device not owned by user)

    ```json
    { "error": "Forbidden: device not owned by user" }
    ```

  - 404 Not Found (deviceId does not exist)

    ```json
    { "error": "Device not found" }
    ```

- cURL examples:
  - With explicit deviceId (Bearer token)

    ```zsh
    curl -X POST http://localhost:3000/api/device/stack \
      -H 'Content-Type: application/json' \
      -H 'Authorization: Bearer <jwt>' \
      -d '{
        "deviceId": "dev_1",
        "tasks": [
          { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } },
          { "type": "place", "to": { "x": 4, "y": 5, "z": 6 } }
        ]
      }'
    ```

  - Inferring deviceId when user owns exactly one device (Bearer token)

    ```zsh
    curl -X POST http://localhost:3000/api/device/stack \
      -H 'Content-Type: application/json' \
      -H 'Authorization: Bearer <jwt>' \
      -d '{
        "tasks": [ { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } } ]
      }'
    ```

---

## POST /api/device

Create a device owned by the authenticated user.

- Auth: required (`Authorization: Bearer <token>`)
- Body JSON:

  ```json
  { "name": "My Robot", "status": "offline" }
  ```

  - `status` is optional; defaults to `offline`. Allowed values: `online`, `offline`.

- Responses:
  - 201 Created

    ```json
    { "deviceId": "dev_xxx", "name": "My Robot", "status": "offline", "userId": "user_abc" }
    ```

  - 400 Validation error
  - 401 Unauthorized

- cURL example:

  ```zsh
  curl -X POST http://localhost:3000/api/device \
    -H 'Content-Type: application/json' \
    -H 'Authorization: Bearer <jwt>' \
    -d '{"name":"My Robot","status":"offline"}'
  ```

---

## GET /api/device/summary

Return a summary for all devices belonging to the authenticated user, including counts of `pending` and `in_progress` task stacks.

- Auth: required (`Authorization: Bearer <token>`)
- Response JSON (array):

  ```json
  [
    {
      "deviceId": "dev_1",
      "name": "device1",
      "status": "online",
      "pendingTasks": 2,
      "activeTasks": 1
    }
  ]
  ```

- Responses:
  - 200 OK: returns empty array `[]` if the user has no devices.
  - 401 Unauthorized: `{ "error": "Unauthorized" }`

- cURL example:

  ```zsh
  curl -X GET http://localhost:3000/api/device/summary \
    -H 'Authorization: Bearer <jwt>'
  ```
  
---

## Data shapes (reference)

- Device

  ```ts
  type DeviceStatus = 'online' | 'offline';
  interface Device {
    deviceId: string;
    name: string;
    status: DeviceStatus;
    userId: string;
  }
  ```

- TaskStack

  ```ts
  type TaskStackStatus = 'pending' | 'in_progress' | 'completed' | 'failed';
  type Coordinates = { x: number; y: number; z: number };
  type PickTask   = { type: 'pick'; from: Coordinates };
  type PlaceTask  = { type: 'place'; to: Coordinates };
  type Task       = PickTask | PlaceTask;

  interface TaskStack {
    stackId: string;
    deviceId: string;
    tasks: Task[]; // persisted as JSON
    status: TaskStackStatus;
    createdAt: string; // ISO date (in DB it's DateTime)
  }
  ```

- User

  ```ts
  interface User {
    id: string;
    email: string;
    name?: string;
    password: string; // hashed
    createdAt: string; // ISO
    updatedAt: string; // ISO
  }
  ```

## Notes

- Authentication uses JWT Bearer tokens. See `src/lib/auth.ts` and the auth endpoints above.
- Validation is powered by Zod. Errors return a unified `{ error, details }` payload.
- Persistence is implemented with Prisma (Postgres). See `prisma/schema.prisma` and `.env.example`.
  - JWT config via `JWT_SECRET` and `JWT_EXPIRES_IN`.
