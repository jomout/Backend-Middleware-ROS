import jwt, { type Secret, type SignOptions } from 'jsonwebtoken';
import bcrypt from 'bcryptjs';

// JWT secret and expiration
const JWT_SECRET: Secret = (process.env.JWT_SECRET || 'dev-secret-change-me') as Secret;
const JWT_EXPIRES_IN: SignOptions['expiresIn'] = (process.env.JWT_EXPIRES_IN || '7d') as SignOptions['expiresIn'];

/**
 * Payload structure for JWT
 */
export type JwtPayload = { sub: string; email: string } & jwt.JwtPayload;

/**
 * Signs a JWT
 * @param payload - The payload to include in the JWT
 * @returns The signed JWT
 */
export function signJwt(payload: Omit<JwtPayload, 'iat' | 'exp'>) {
  const options: SignOptions = { expiresIn: JWT_EXPIRES_IN };
  return jwt.sign(payload, JWT_SECRET, options);
}

/**
 * Verifies a JWT and returns the decoded payload
 * @param token - The JWT to verify
 * @returns The decoded payload or null if verification fails
 */
export function verifyJwt<T extends JwtPayload = JwtPayload>(token: string): T | null {
  try {
    return jwt.verify(token, JWT_SECRET) as T;
  } catch {
    return null;
  }
}

/**
 * Hashes a plain text password
 * @param plain - The plain text password to hash
 * @returns The hashed password
 */
export async function hashPassword(plain: string) {
  const salt = await bcrypt.genSalt(10);
  return bcrypt.hash(plain, salt);
}

/**
 * Compares a plain text password with a hashed password
 * @param plain - The plain text password to compare
 * @param hash - The hashed password to compare against
 * @returns True if the passwords match, false otherwise
 */
export function comparePassword(plain: string, hash: string) {
  return bcrypt.compare(plain, hash);
}
