-- ================================================================
-- PostgreSQL schema for Devices, TaskStacks, and Users
-- ================================================================

-- ========================
-- ENUMS
-- ========================

-- Device status: tracks whether a device is online or offline
CREATE TYPE device_status AS ENUM ('online', 'offline');

-- TaskStack status: lifecycle of a stack of tasks
CREATE TYPE taskstack_status AS ENUM ('pending', 'in_progress', 'completed', 'failed');

-- ========================
-- USERS TABLE
-- ========================
CREATE TABLE users (
    id         TEXT PRIMARY KEY DEFAULT gen_random_uuid(), -- UUID
    email      TEXT NOT NULL UNIQUE,                       -- unique identifier for login
    name       TEXT NOT NULL,                              -- user name
    password   TEXT NOT NULL,                              -- hashed password
    created_at TIMESTAMPTZ NOT NULL DEFAULT now(),         -- timestamp of account creation
    updated_at TIMESTAMPTZ NOT NULL DEFAULT now()          -- updated via trigger or app logic
);

-- ========================
-- DEVICES TABLE
-- ========================
CREATE TABLE devices (
    device_id  TEXT PRIMARY KEY DEFAULT gen_random_uuid(),              -- unique device ID
    name       TEXT NOT NULL,                                           -- human-friendly name
    status     device_status NOT NULL,                                  -- enum: online/offline
    user_id    TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE    -- FK to User
);

-- Index for faster lookups of devices by user
CREATE INDEX idx_device_user_id ON devices(user_id);

-- ========================
-- TASKSTACKS TABLE
-- ========================
CREATE TABLE task_stacks (
    stack_id   TEXT PRIMARY KEY DEFAULT gen_random_uuid(),                      -- unique stack ID
    device_id  TEXT NOT NULL REFERENCES devices(device_id) ON DELETE CASCADE,  -- FK to Device
    tasks      JSONB NOT NULL,                                                  -- JSON array of tasks (pick/place)
    status     taskstack_status NOT NULL DEFAULT 'pending',                     -- lifecycle state
    created_at TIMESTAMPTZ NOT NULL DEFAULT now()                               -- creation timestamp
);

-- Indexes for performance
CREATE INDEX idx_taskstack_device_id ON task_stacks(device_id);
CREATE INDEX idx_taskstack_status ON task_stacks(status);
