import { NextRequest, NextResponse } from 'next/server';
import { prisma } from '@/lib/prisma';
import { registerSchema, errorResponse } from '@/lib/validation';
import { hashPassword, signJwt } from '@/lib/jwt';
import { flattenError } from 'zod';

/**
 * User registration handler
 * @param req NextRequest object containing the request details
 * @returns NextResponse with JWT token or error message
 */
export async function POST(request: NextRequest) {
  // Validate registration request body
  const parse = registerSchema.safeParse(await request.json().catch(() => ({})));
  if (!parse.success) {
    return errorResponse('Validation error', 400, flattenError(parse.error));
  }

  const { email, password, name } = parse.data;

  // Check if the email is already registered
  const exists = await prisma.user.findUnique({ where: { email } });
  if (exists) {
    return errorResponse('Email already registered', 409);
  }

  // Create the user with hashed password
  const hashed = await hashPassword(password);
  const user = await prisma.user.create({ data: { email, password: hashed, name } });

  // Sign and return a JWT
  const token = signJwt({ sub: user.id, email: user.email });

  return NextResponse.json({ token }, { status: 201 });
}
