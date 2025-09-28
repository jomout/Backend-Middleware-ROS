import { PrismaClient } from '@prisma/client';
import { logger } from './logger';

declare global {
  // eslint-disable-next-line no-var
  var prisma: PrismaClient | undefined;
}

/**
 * Log levels configuration:
 * - In production, only log 'error' and 'warn' levels to reduce noise.
 * - In development, log 'query', 'error', 'warn', and 'info' for better visibility.
 */
const logLevels = process.env.NODE_ENV === 'production'
  ? [{ level: 'error', emit: 'event' }, { level: 'warn', emit: 'event' }] as const
  : [
      { level: 'query', emit: 'event' },
      { level: 'error', emit: 'event' },
      { level: 'warn', emit: 'event' },
      { level: 'info', emit: 'event' },
    ] as const;

export const prisma = global.prisma ?? new PrismaClient({
  log: logLevels as any,
});

// Hook logs to our logger
// @ts-expect-error prisma log typing
prisma.$on('error', (e: any) => logger.error('Prisma error', { target: e.target, message: e.message }));
// @ts-expect-error prisma log typing
prisma.$on('warn', (e: any) => logger.warn('Prisma warn', { message: e.message }));
// @ts-expect-error prisma log typing
prisma.$on('info', (e: any) => logger.info('Prisma info', { message: e.message }));
// @ts-expect-error prisma log typing
prisma.$on('query', (e: any) => logger.debug('Prisma query', { query: e.query, params: e.params, durationMs: e.duration }));
if (process.env.NODE_ENV !== 'production') global.prisma = prisma;
