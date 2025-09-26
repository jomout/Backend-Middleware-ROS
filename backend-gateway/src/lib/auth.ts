import type { NextRequest } from 'next/server';
import { verifyJwt } from './jwt';

/**
 * Extracts and verifies the JWT from the Authorization header.
 * @param req - The incoming request
 * @returns The user ID or null if not found
 */
export function requireUserId(request: NextRequest): string | null {
  const auth = request.headers.get('authorization');

  if (!auth || !auth.startsWith('Bearer ')) {
    return null;
  }

  const token = auth.slice(7);
  const payload = verifyJwt(token);

  return payload?.sub ? (payload.sub as string) : null;
}
