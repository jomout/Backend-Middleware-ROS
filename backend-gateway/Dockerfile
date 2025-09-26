FROM node:22-alpine AS base
ENV NEXT_TELEMETRY_DISABLED=1
WORKDIR /app

# Install dependencies
FROM base AS dependencies

COPY package.json package-lock.json ./

RUN npm ci


# Build the app
FROM base AS builder
COPY --from=dependencies /app/node_modules ./node_modules
COPY . .

# Generate Prisma client before building Next (required for imports during build)
RUN npm run prisma:generate

RUN npm run build


# Production image, copy all the files and run next
FROM base AS runner

ENV NODE_ENV=production

# Use built-in non-root user
USER node

# Copy standalone output only (no need for .next/static if no frontend)
COPY --from=builder /app/.next/standalone ./

# Include Prisma client and engine binaries for runtime
COPY --from=builder /app/node_modules/@prisma ./node_modules/@prisma
COPY --from=builder /app/node_modules/.prisma ./node_modules/.prisma
COPY --from=builder /app/node_modules/@prisma/client ./node_modules/@prisma/client

EXPOSE 3000
CMD ["node", "server.js"]