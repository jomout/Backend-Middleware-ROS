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
-- TRIGGER FUNCTION
-- ========================
-- Updates updated_at on every row update
CREATE OR REPLACE FUNCTION set_updated_at()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = now();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- ========================
-- USERS TABLE
-- ========================
CREATE TABLE users (
    user_id    UUID PRIMARY KEY DEFAULT gen_random_uuid(), -- UUID
    email      TEXT NOT NULL UNIQUE,                       -- unique identifier for login
    name       TEXT NOT NULL,                              -- user name
    password   TEXT NOT NULL,                              -- hashed password
    created_at TIMESTAMPTZ NOT NULL DEFAULT now(),         -- timestamp of account creation
    updated_at TIMESTAMPTZ NOT NULL DEFAULT now()          -- updated via trigger
);

-- ========================
-- DEVICES TABLE
-- ========================
CREATE TABLE devices (
    device_id  UUID PRIMARY KEY DEFAULT gen_random_uuid(),                  -- UUID
    name       TEXT NOT NULL,                                               -- human-friendly name
    status     device_status NOT NULL,                                      -- enum: online/offline
    user_id    UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,   -- FK to User
    created_at TIMESTAMPTZ NOT NULL DEFAULT now(),                          -- timestamp of device registration
    updated_at TIMESTAMPTZ NOT NULL DEFAULT now()                           -- updated via trigger
);

-- Index for faster lookups of devices by user
CREATE INDEX idx_device_user_id ON devices(user_id);

-- ========================
-- TASKSTACKS TABLE
-- ========================
CREATE TABLE task_stacks (
    stack_id   UUID PRIMARY KEY DEFAULT gen_random_uuid(),                      -- UUID
    device_id  UUID NOT NULL REFERENCES devices(device_id) ON DELETE CASCADE,   -- FK to Device
    tasks      JSONB NOT NULL,                                                  -- JSON array of tasks (pick/place)
    status     taskstack_status NOT NULL DEFAULT 'pending',                     -- lifecycle state
    created_at TIMESTAMPTZ NOT NULL DEFAULT now(),                              -- timestamp of stack creation
    updated_at TIMESTAMPTZ NOT NULL DEFAULT now()                               -- updated via trigger
);

-- Indexes for performance
CREATE INDEX idx_taskstack_device_id ON task_stacks(device_id);
CREATE INDEX idx_taskstack_status ON task_stacks(status);

-- ========================
-- TRIGGERS
-- ========================
CREATE TRIGGER trg_users_set_updated_at
BEFORE UPDATE ON users
FOR EACH ROW
EXECUTE FUNCTION set_updated_at();

CREATE TRIGGER trg_devices_set_updated_at
BEFORE UPDATE ON devices
FOR EACH ROW
EXECUTE FUNCTION set_updated_at();

CREATE TRIGGER trg_task_stacks_set_updated_at
BEFORE UPDATE ON task_stacks
FOR EACH ROW
EXECUTE FUNCTION set_updated_at();
