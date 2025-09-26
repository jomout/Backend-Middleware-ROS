import { NextRequest, NextResponse } from 'next/server';
import { verifyJwt } from '@/lib/jwt';
import { errorResponse } from '@/lib/validation';

/**
 * Get the current user's information.
 * @param request - The incoming request
 * @returns The user's information or an error response
 */
export async function GET(request: NextRequest) {
  const auth = request.headers.get('authorization');
  const token = auth?.startsWith('Bearer ') ? auth.slice(7) : null;
  if (!token) return errorResponse('Unauthorized', 401);
  const payload = verifyJwt(token);
  if (!payload) return errorResponse('Unauthorized', 401);
  return NextResponse.json({ userId: payload.sub, email: payload.email });
}
