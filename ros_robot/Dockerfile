# Base image with ROS 2 Jazzy (Ubuntu 24.04, Python 3.12)
FROM ros:jazzy-ros-core AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH="/root/.local/bin:${PATH}" \
    ROS_DISTRO=jazzy

WORKDIR /app

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

# Install dependencies into a venv
RUN uv sync --locked --no-dev

# Make uv’s venv the default
ENV VIRTUAL_ENV=/app/.venv \
    PATH="/app/.venv/bin:${PATH}"

# Copy project source
COPY . .

CMD ["/bin/bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec python3 /app/robot_node.py"]
