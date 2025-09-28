import { NextRequest, NextResponse } from 'next/server';
import { prisma } from '@/lib/prisma';
import { loginSchema, errorResponse } from '@/lib/validation';
import { comparePassword, signJwt } from '@/lib/jwt';
import { flattenError } from 'zod';
import { logger } from '@/lib/logger';

/**
 * Login user. 
 * Validates the request body, checks user credentials, and returns a JWT if successful.
 * 
 * @param req NextRequest object containing the request details
 * @returns NextResponse with JWT token or error message
 */
export async function POST(request: NextRequest) {
  const log = logger.child({ route: 'POST /api/auth/login' });
  // Validate login request body
  const parse = loginSchema.safeParse(await request.json().catch(() => ({})));

  // Check if the request body is valid
  if (!parse.success) {
    log.warn('Validation error', { details: flattenError(parse.error) });
    return errorResponse('Validation error', 400, flattenError(parse.error));
  }
  
  const { email, password } = parse.data;

  // Find the user by email
  const user = await prisma.user.findUnique({ where: { email } });
  if (!user) {
    log.warn('Invalid credentials (email not found)', { email });
    return errorResponse('Invalid credentials', 401);
  }

  // Check if the password is correct
  const ok = await comparePassword(password, user.password);
  if (!ok) {
    log.warn('Invalid credentials (password mismatch)', { userId: user.id });
    return errorResponse('Invalid credentials', 401);
  }
  
  // If valid, sign and return a JWT
  const token = signJwt({ sub: user.id, email: user.email });
  log.info('User logged in', { userId: user.id });
  return NextResponse.json({ token });
}
