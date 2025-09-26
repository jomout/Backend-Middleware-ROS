import { NextResponse } from 'next/server';
import { z } from 'zod';

/**
 * Schema for 3D coordinates
 */
export const coordinatesSchema = z.object({
  x: z.number(),
  y: z.number(),
  z: z.number(),
});

/**
 * Schemas for task definitions
 */
export const pickTaskSchema = z.object({
  type: z.literal('pick'),
  from: coordinatesSchema,
});

export const placeTaskSchema = z.object({
  type: z.literal('place'),
  to: coordinatesSchema,
});

export const taskSchema = z.discriminatedUnion('type', [pickTaskSchema, placeTaskSchema]);

/**
 * Schema for creating a task stack
 */
export const createTaskStackBodySchema = z.object({
  deviceId: z.string().min(1).optional(),
  tasks: z.array(taskSchema).min(1, 'tasks must be a non-empty array'),
});

export type CreateTaskStackBodyInput = z.infer<typeof createTaskStackBodySchema>;

/**
 * Helper function to create a standardized error response
 * @param message Error message to include in the response
 * @param status HTTP status code for the response
 * @param details Additional details about the error
 * @returns NextResponse object with the error details
 */
export function errorResponse(message: string, status = 400, details?: unknown) {
  const body: Record<string, unknown> = { error: message };
  if (details !== undefined) body.details = details;
  return NextResponse.json(body, { status });
}

/**
 * Schemas for user registration and login
 */
export const registerSchema = z.object({
  email: z.email(),
  password: z.string().min(8),
  name: z.string().min(1),
});

export const loginSchema = z.object({
  email: z.email(),
  password: z.string().min(1),
});

