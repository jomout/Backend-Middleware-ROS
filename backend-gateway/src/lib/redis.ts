import Redis from "ioredis";
import { logger } from "./logger";

export const redis = new Redis(process.env.REDIS_URL || "redis://localhost:6379");

redis.on('connect', () => logger.info('Redis connected', { url: process.env.REDIS_URL || 'redis://localhost:6379' }));
redis.on('ready', () => logger.debug('Redis ready'));
redis.on('error', (err) => logger.error('Redis error', { err: String(err) }));
redis.on('reconnecting', () => logger.warn('Redis reconnecting'));

export const TASK_STACK_STREAM = process.env.TASK_STACK_STREAM || 'task_stacks';
