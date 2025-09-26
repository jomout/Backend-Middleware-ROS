import Redis from "ioredis";

export const redis = new Redis(process.env.REDIS_URL || "redis://localhost:6379");

export const TASK_STACK_STREAM = process.env.TASK_STACK_STREAM || 'task_stacks';
