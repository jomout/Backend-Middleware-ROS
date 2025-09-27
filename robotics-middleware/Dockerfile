# Base image with ROS 2 Jazzy (Ubuntu 24.04, Python 3.12)
FROM ros:jazzy-ros-core AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH="/root/.local/bin:${PATH}"

ENV ROS_DISTRO=jazzy

WORKDIR /app


# -----------------------------
# Dependencies stage
# -----------------------------
FROM base AS dependencies

# Build deps
RUN apt-get update \
     && apt-get install -y --no-install-recommends \
         curl \
         ca-certificates \
         build-essential \
         libpq-dev \
         python3-venv \
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

# Ensure ROS 2 environment is sourced before starting FastAPI so rclpy works
CMD ["/bin/bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && python -m uvicorn app.main:app --host 0.0.0.0 --port 8000"]
