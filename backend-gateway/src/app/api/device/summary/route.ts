import { NextRequest, NextResponse } from "next/server";
import { requireUserId } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import type { GetDeviceSummaryResponseDto } from "@/dtos/api";
import { errorResponse } from "@/lib/validation";
import { logger } from "@/lib/logger";

/**
 * Get a summary of devices for the authenticated user.
 * @param request - The incoming HTTP request.
 * @returns A JSON response with the device summary information.
 */
export async function GET(request: NextRequest) {
  const log = logger.child({ route: 'GET /api/device/summary' });
  
  // User must be authenticated
  const userId = requireUserId(request);
  if (!userId) {
    log.warn('Unauthorized');
    return errorResponse('Unauthorized', 401);
  }

  // Fetch devices for the user
  const devices = await prisma.device.findMany({ where: { userId }, select: { deviceId: true, name: true, status: true } });
  if (devices.length === 0) {
    log.info('No devices for user', { userId });
    return NextResponse.json([]);
  }

  // Fetch task stack counts grouped by deviceId and status
  // for statuses 'pending' and 'in_progress'
  const deviceIds = devices.map((d: { deviceId: string }) => d.deviceId);
  const stacks = await prisma.taskStack.groupBy({
    by: ['deviceId', 'status'],
    where: { deviceId: { in: deviceIds }, status: { in: ['pending', 'in_progress'] } },
    _count: { _all: true },
  });

  // Map deviceId to counts of pending and in_progress tasks
  const counts = new Map<string, { pending: number; in_progress: number }>();
  for (const d of deviceIds) counts.set(d, { pending: 0, in_progress: 0 });
  for (const row of stacks) {
    const entry = counts.get(row.deviceId)!;
    if (row.status === 'pending') entry.pending = row._count._all;
    if (row.status === 'in_progress') entry.in_progress = row._count._all;
  }

  const response: GetDeviceSummaryResponseDto = devices.map((d: { deviceId: string; name: string; status: string }) => ({
    deviceId: d.deviceId,
    name: d.name,
    status: d.status as any,
    pendingTasks: counts.get(d.deviceId)!.pending,
    activeTasks: counts.get(d.deviceId)!.in_progress,
  }));

  log.info('Device summary', { userId, deviceCount: devices.length });
  return NextResponse.json(response);
}