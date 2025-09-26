import { NextRequest, NextResponse } from "next/server";
import { requireUserId } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { createTaskStackBodySchema } from "@/lib/validation";
import type { CreateTaskStackResponseDto } from "@/dtos/api";
import { errorResponse } from "@/lib/validation";

export async function POST(request: NextRequest) {
  const userId = requireUserId(request);
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const parseResult = createTaskStackBodySchema.safeParse(await request.json().catch(() => ({})));
  if (!parseResult.success) {
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
      return errorResponse('deviceId is required when user owns multiple devices', 400);
    }
  }

  const device = await prisma.device.findUnique({ where: { deviceId: resolvedDeviceId } });
  if (!device) return errorResponse('Device not found', 404);
  if (device.userId !== userId) return errorResponse('Forbidden: device not owned by user', 403);

  const created = await prisma.taskStack.create({
    data: {
      deviceId: resolvedDeviceId,
      tasks: tasks as any,
      status: 'pending',
    },
    select: { stackId: true, status: true },
  });

  const response: CreateTaskStackResponseDto = { stackId: created.stackId, status: created.status as any };
  return NextResponse.json(response, { status: 201 });
}