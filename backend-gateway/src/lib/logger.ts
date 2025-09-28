type LogLevel = 'debug' | 'info' | 'warn' | 'error';

const LEVELS: Record<LogLevel, number> = {
  debug: 10,
  info: 20,
  warn: 30,
  error: 40,
};

const envLevel = (process.env.LOG_LEVEL as LogLevel) || (process.env.NODE_ENV === 'production' ? 'info' : 'debug');
const threshold = LEVELS[envLevel] ?? LEVELS.info;

function levelLabel(level: LogLevel) {
  return level.toUpperCase().padEnd(5, ' ');
}

function serializeValue(v: unknown): string {
  if (v == null) return 'null';
  if (typeof v === 'string') {
    // quote only if contains spaces or special chars
    return /\s|["'`]/.test(v) ? JSON.stringify(v) : v;
  }
  if (typeof v === 'number' || typeof v === 'boolean') return String(v);
  // compact JSON for objects/arrays
  try { return JSON.stringify(v); } catch { return '[Unserializable]'; }
}

function serializeContext(ctx?: Record<string, unknown>): string {
  if (!ctx) return '';
  const parts: string[] = [];
  for (const [k, v] of Object.entries(ctx)) {
    parts.push(`${k}=${serializeValue(v)}`);
  }
  return parts.length ? ' ' + parts.join(' ') : '';
}

/**
 * Logs a message with the specified log level.
 * @param level - The log level.
 * @param msg - The log message.
 * @param context - Additional context to include in the log.
 * @returns 
 */
function log(level: LogLevel, msg: string, context?: Record<string, unknown>) {
  if (LEVELS[level] < threshold) return;
  // Pretty format only: LEVEL msg key=value
  const lvl = levelLabel(level);
  const suffix = serializeContext(context);
  const line = `${lvl} ${msg}${suffix}`;
  if (level === 'error') console.error(line);
  else if (level === 'warn') console.warn(line);
  else console.log(line);
}

/**
 * A simple structured logger with log levels and child loggers.
 */
export const logger = {
  debug: (msg: string, ctx?: Record<string, unknown>) => log('debug', msg, ctx),
  info: (msg: string, ctx?: Record<string, unknown>) => log('info', msg, ctx),
  warn: (msg: string, ctx?: Record<string, unknown>) => log('warn', msg, ctx),
  error: (msg: string, ctx?: Record<string, unknown>) => log('error', msg, ctx),
  child: (ctx: Record<string, unknown>) => ({
    debug: (msg: string, more?: Record<string, unknown>) => log('debug', msg, { ...ctx, ...(more || {}) }),
    info: (msg: string, more?: Record<string, unknown>) => log('info', msg, { ...ctx, ...(more || {}) }),
    warn: (msg: string, more?: Record<string, unknown>) => log('warn', msg, { ...ctx, ...(more || {}) }),
    error: (msg: string, more?: Record<string, unknown>) => log('error', msg, { ...ctx, ...(more || {}) }),
  }),
};
