# Base image
FROM python:3.13-slim AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH="/root/.local/bin:${PATH}"

WORKDIR /app


# -----------------------------
# Dependencies stage
# -----------------------------
FROM base AS dependencies

# Build deps
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       curl \
       build-essential \
       libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Install uv
ADD https://astral.sh/uv/install.sh /uv-installer.sh
RUN sh /uv-installer.sh && rm /uv-installer.sh

# Copy dependency files
COPY pyproject.toml uv.lock ./

# Install dependencies into venv
RUN uv sync --locked --no-dev


# -----------------------------
# Builder stage
# -----------------------------
FROM base AS builder

ENV VIRTUAL_ENV=/opt/venv \
    PATH="/opt/venv/bin:${PATH}"

# Copy venv from dependencies
COPY --from=dependencies /app/.venv /opt/venv

# Copy project source
COPY . .


# -----------------------------
# Runtime stage
# -----------------------------
FROM base AS runtime

ENV VIRTUAL_ENV=/opt/venv \
    PATH="/opt/venv/bin:${PATH}"

# Runtime deps only (no compilers)
RUN apt-get update \
    && apt-get install -y --no-install-recommends libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy venv from dependencies
COPY --from=dependencies /app/.venv /opt/venv

# Copy source from builder
COPY --from=builder /app /app

# Expose FastAPI default port
EXPOSE 8000

# Start FastAPI with Uvicorn
CMD ["python", "-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
