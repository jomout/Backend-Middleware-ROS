# API Reference

This document describes the available REST endpoints under `src/app/api`.

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

Use JWT Bearer tokens for authenticated requests. First **register** or **log in** to obtain a token, then include it in the `Authorization` header.

### Register a new user

- Endpoint: `POST /api/auth/register`
- Body:

  ```json
  { "email": "user1@example.com", "password": "user1_secure_password", "name": "user1" }
  ```

- Responses:
  - 201 Created: `{ "token": "<jwt>" }`
  - 400 Validation error
  - 409 Email already registered

- cURL (capture TOKEN):

  ```zsh
  TOKEN=$(curl -s -X POST http://localhost:3000/api/auth/register \
    -H 'Content-Type: application/json' \
    -d '{"email":"user1@example.com","password":"user1_secure_password","name":"user1"}' \
    | jq -r .token)
  echo $TOKEN
  ```

### Log in

- Endpoint: `POST /api/auth/login`
- Body:

  ```json
  { "email": "user1@example.com", "password": "user1_secure_password" }
  ```

- Responses:
  - 200 OK: `{ "token": "<jwt>" }`
  - 400 Validation error
  - 401 Invalid credentials

- cURL (capture TOKEN):

  ```zsh
  TOKEN=$(curl -s -X POST http://localhost:3000/api/auth/login \
    -H 'Content-Type: application/json' \
    -d '{"email":"user1@example.com","password":"user1_secure_password"}' \
    | jq -r .token)
  echo $TOKEN
  ```

---

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

- Validation **(Zod)**:
  - `deviceId` optional string. Required if the user owns multiple devices.
  - `tasks` is a non-empty array of discriminated union on `type`:
    - `pick`: `{ type: 'pick', from: { x: number, y: number, z: number } }`
    - `place`: `{ type: 'place', to: { x: number, y: number, z: number } }`

- Responses:
  - **201 Created**

    ```json
    { "stackId": "stack_abc123", "status": "pending" }
    ```

  - **400 Bad Request** (validation or missing deviceId when needed)

    ```json
    { "error": "Validation error", "details": { "fieldErrors": { "tasks": ["..."] } } }
    ```

  - **401 Unauthorized**

    ```json
    { "error": "Unauthorized" }
    ```

  - **403 Forbidden** (device not owned by user)

    ```json
    { "error": "Forbidden: device not owned by user" }
    ```

  - **404 Not Found** (deviceId does not exist)

    ```json
    { "error": "Device not found" }
    ```

- cURL examples:
  - With explicit deviceId (uses TOKEN)

    ```zsh
    curl -X POST http://localhost:3000/api/device/stack \
      -H 'Content-Type: application/json' \
      -H "Authorization: Bearer $TOKEN" \
      -d '{
        "deviceId": "dev_1",
        "tasks": [
          { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } },
          { "type": "place", "to": { "x": 4, "y": 5, "z": 6 } }
        ]
      }'
    ```

  - Inferring deviceId when user owns exactly one device (uses TOKEN)

    ```zsh
    curl -X POST http://localhost:3000/api/device/stack \
      -H 'Content-Type: application/json' \
      -H "Authorization: Bearer $TOKEN" \
      -d '{
        "tasks": [ { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } } ]
      }'
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
  - **200 OK**: returns empty array `[]` if the user has no devices.
  - **401 Unauthorized**: `{ "error": "Unauthorized" }`

- cURL example (uses TOKEN):

  ```zsh
  curl -X GET http://localhost:3000/api/device/summary \
    -H "Authorization: Bearer $TOKEN"
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
    createdAt: string;
  }
  ```

---

## Notes

- Authentication uses JWT Bearer tokens. See `src/lib/auth.ts` and the auth endpoints above.
- Validation is powered by Zod. Errors return a unified `{ error, details }` payload.
- Persistence is implemented with Prisma (Postgres). See `prisma/schema.prisma` and `.env.example`.
  - JWT config via `JWT_SECRET` and `JWT_EXPIRES_IN`.
