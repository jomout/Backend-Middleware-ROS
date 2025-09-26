import { NextRequest, NextResponse } from 'next/server';
import { requireUserId } from '@/lib/auth';
import { prisma } from '@/lib/prisma';
import { flattenError, z } from 'zod';
import { errorResponse } from '@/lib/validation';

const createDeviceSchema = z.object({
  name: z.string().min(1, 'name is required'),
  status: z.enum(['online', 'offline']).optional(),
});

export async function POST(request: NextRequest) {
  const userId = requireUserId(request);
  if (!userId) return errorResponse('Unauthorized', 401);

  const body = await request.json().catch(() => ({}));
  const parsed = createDeviceSchema.safeParse(body);
  if (!parsed.success) return errorResponse('Validation error', 400, flattenError(parsed.error));

  const { name, status = 'offline' } = parsed.data;

  const device = await prisma.device.create({
    data: { name, status, userId },
    select: { deviceId: true, name: true, status: true, userId: true },
  });

  return NextResponse.json(device, { status: 201 });
}
