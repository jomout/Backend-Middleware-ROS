import { NextRequest, NextResponse } from "next/server";
import { requireUserId } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { createTaskStackBodySchema } from "@/lib/validation";
import type { CreateTaskStackResponseDto } from "@/dtos/api";
import { errorResponse } from "@/lib/validation";
import { redis, TASK_STACK_STREAM } from "@/lib/redis";
import { logger } from "@/lib/logger";

/**
 * Create a new task stack for a device.
 * @param request - The incoming HTTP request.
 * @returns A JSON response with the created task stack information.
 */
export async function POST(request: NextRequest) {
  const log = logger.child({ route: 'POST /api/device/stack' });
  
  // User must be authenticated
  const userId = requireUserId(request);
  if (!userId) {
    log.warn('Unauthorized');
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  // Validate request body
  const parseResult = createTaskStackBodySchema.safeParse(await request.json().catch(() => ({})));
  if (!parseResult.success) {
    log.warn('Validation error', { details: parseResult.error.flatten() });
    return errorResponse('Validation error', 400, parseResult.error.flatten());
  }
  const { deviceId, tasks } = parseResult.data;

  // Infer deviceId if omitted: if user has exactly one device, use that; otherwise require deviceId
  let resolvedDeviceId = deviceId;
  if (!resolvedDeviceId) {
    const count = await prisma.device.count({ where: { userId } });
    if (count === 1) {
      const onlyDevice = await prisma.device.findFirst({ where: { userId } });
      resolvedDeviceId = onlyDevice!.deviceId;
    } else {
      log.warn('deviceId required for multi-device user');
      return errorResponse('deviceId is required when user owns multiple devices', 400);
    }
  }

  // Verify device exists and belongs to user
  const device = await prisma.device.findUnique({ where: { deviceId: resolvedDeviceId } });
  
  // If device not found or not owned by user, return error
  if (!device) {
    log.warn('Device not found', { deviceId: resolvedDeviceId });
    return errorResponse('Device not found', 404);
  }

  // If device not found or not owned by user, return error
  if (device.userId !== userId) {
    log.warn('Forbidden device ownership', { deviceId: resolvedDeviceId, userId });
    return errorResponse('Forbidden: device not owned by user', 403);
  }

  // Create the task stack in the database
  const created = await prisma.taskStack.create({
    data: {
      deviceId: resolvedDeviceId,
      tasks: tasks as any,
      status: 'pending',
    },
    select: { stackId: true, status: true },
  });
  log.info('Task stack created', { stackId: created.stackId, deviceId: resolvedDeviceId, userId });

  // Publish event to Redis stream for middleware consumers
  try {
    await redis.xadd(
      TASK_STACK_STREAM,
      "*",           
      "event", "task_stack.created",
      "stackId", created.stackId,
      "deviceId", resolvedDeviceId,
      "userId", userId,
    );
    log.info('Published to Redis stream', { stream: TASK_STACK_STREAM, stackId: created.stackId });
  } catch (e) {
    log.error('Failed to publish to Redis stream', { error: String(e) });
  }

  const response: CreateTaskStackResponseDto = { stackId: created.stackId, status: created.status as any };
  return NextResponse.json(response, { status: 201 });
}