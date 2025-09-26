# ROS API (Next.js)

This is a [Next.js](https://nextjs.org) project bootstrapped with [`create-next-app`](https://nextjs.org/docs/app/api-reference/create-next-app).

## Getting Started

First, run the development server:

```bash
npm run dev
# or
yarn dev
# or
pnpm dev
# or
bun dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

You can start editing the page by modifying `app/route.ts`. The page auto-updates as you edit the file.

## Learn More

To learn more about Next.js, take a look at the following resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js features and API.
- [Learn Next.js](https://nextjs.org/learn) - an interactive Next.js tutorial.

You can check out [the Next.js GitHub repository](https://github.com/vercel/next.js) - your feedback and contributions are welcome!

## Deploy on Vercel

The easiest way to deploy your Next.js app is to use the [Vercel Platform](https://vercel.com/new?utm_medium=default-template&filter=next.js&utm_source=create-next-app&utm_campaign=create-next-app-readme) from the creators of Next.js.

Check out our [Next.js deployment documentation](https://nextjs.org/docs/app/building-your-application/deploying) for more details.

## API Routes

This directory contains example API routes for the headless API app.

For more details, see [route.js file convention](https://nextjs.org/docs/app/api-reference/file-conventions/route).

## API Reference

For endpoint payloads, validation, responses, and curl examples, see the full API docs:

- [API Reference (src/app/api/README.md)](src/app/api/README.md)

## Database Collections and Entities

This project currently uses an in-memory store for demonstration. Replace with your real database (e.g., MongoDB, Postgres, Prisma) while keeping the shapes defined below.

### devices

- deviceId: string (primary key)
- name: string
- status: enum('online' | 'offline')
- userId: string (owner relationship)

Example document:

```json
{
  "deviceId": "dev_1",
  "name": "device1",
  "status": "online",
  "userId": "user_a"
}
```

### task_stacks

- stackId: string (primary key)
- deviceId: string (FK -> devices.deviceId)
- tasks: Array of tasks
  - pick task: `{ "type": "pick", "from": { "x": number, "y": number, "z": number } }`
  - place task: `{ "type": "place", "to": { "x": number, "y": number, "z": number } }`
- status: enum('pending' | 'in_progress' | 'completed' | 'failed')
- createdAt: number (epoch ms)

Example document:

```json
{
  "stackId": "stack_abc123",
  "deviceId": "dev_1",
  "tasks": [
    { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } },
    { "type": "place", "to": { "x": 4, "y": 5, "z": 6 } }
  ],
  "status": "pending",
  "createdAt": 1727212345678
}
```

## Implemented API Endpoints

Authentication: use JWT Bearer tokens in the `Authorization: Bearer <token>` header. Obtain a token via `/api/auth/register` or `/api/auth/login`. See API docs for details.

### POST /api/device/stack

Creates a new task stack for a device. Must be owned by the authenticated user.

Body:

```json
{
  "deviceId": "dev_1", // optional if user owns exactly one device
  "tasks": [
    { "type": "pick", "from": { "x": 1, "y": 2, "z": 3 } },
    { "type": "place", "to": { "x": 4, "y": 5, "z": 6 } }
  ]
}
```

Responses:

- 201: `{ "stackId": string, "status": "pending" }`
- 400: validation errors (e.g., empty tasks, bad coordinates)
- 401: missing/invalid auth
- 403: device not owned by user
- 404: device not found

### GET /api/device/summary

Returns all devices belonging to the authenticated user with counts of task stacks.

Response item shape:

```json
{
  "deviceId": "dev_1",
  "name": "device1",
  "status": "online",
  "pendingTasks": 2,
  "activeTasks": 1
}
```

Notes:

- `pendingTasks` counts stacks with status `pending`
- `activeTasks` counts stacks with status `in_progress`
