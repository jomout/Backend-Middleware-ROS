import { NextRequest, NextResponse } from "next/server";
import { requireUserId } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import type { GetDeviceSummaryResponseDto } from "@/dtos/api";
import { errorResponse } from "@/lib/validation";


export async function GET(request: NextRequest) {
  const userId = requireUserId(request);
  if (!userId) return errorResponse('Unauthorized', 401);

  const devices = await prisma.device.findMany({ where: { userId }, select: { deviceId: true, name: true, status: true } });
  if (devices.length === 0) return NextResponse.json([]);

  const deviceIds = devices.map((d: { deviceId: string }) => d.deviceId);
  const stacks = await prisma.taskStack.groupBy({
    by: ['deviceId', 'status'],
    where: { deviceId: { in: deviceIds }, status: { in: ['pending', 'in_progress'] } },
    _count: { _all: true },
  });

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

  return NextResponse.json(response);
}