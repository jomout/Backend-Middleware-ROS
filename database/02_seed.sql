-- ================================================================
-- Demo seed data for users, devices, and task stacks
-- This file expects that:
--  - enums and tables are already created by 01_init.sql
--  - pgcrypto extension is available (00_extensions.sql)
-- Passwords are hashed using bcrypt via pgcrypto's crypt() with gen_salt('bf').
-- ================================================================

-- Users -----------------------------------------------------------
INSERT INTO users (email, name, password)
VALUES
  ('user1@example.com', 'user1', crypt('user1_secure_password', gen_salt('bf'))),
  ('user2@example.com',   'user2',   crypt('user2_secure_password', gen_salt('bf')))
ON CONFLICT (email) DO NOTHING;

-- Devices ---------------------------------------------------------
-- Device for User1
INSERT INTO devices (name, status, user_id)
SELECT 'robot_1', 'online', u.user_id
FROM users u
WHERE u.email = 'user1@example.com'
  AND NOT EXISTS (
    SELECT 1 FROM devices d WHERE d.name = 'robot_1' AND d.user_id = u.user_id
  );

INSERT INTO devices (name, status, user_id)
SELECT 'robot_3', 'offline', u.user_id
FROM users u
WHERE u.email = 'user1@example.com'
  AND NOT EXISTS (
    SELECT 1 FROM devices d WHERE d.name = 'robot_3' AND d.user_id = u.user_id
  );

-- Device for User2
INSERT INTO devices (name, status, user_id)
SELECT 'robot_2', 'online', u.user_id
FROM users u
WHERE u.email = 'user2@example.com'
  AND NOT EXISTS (
    SELECT 1 FROM devices d WHERE d.name = 'robot_2' AND d.user_id = u.user_id
  );

-- Task stacks -----------------------------------------------------
-- Seed a couple of stacks for user1 robot_1
INSERT INTO task_stacks (device_id, tasks, status)
SELECT d.device_id,
       '[{"type":"pick","from":{"x":1,"y":2,"z":3}},{"type":"place","to":{"x":4,"y":5,"z":6}}]'::jsonb,
       'completed'
FROM devices d
JOIN users u ON u.user_id = d.user_id
WHERE u.email = 'user1@example.com' AND d.name = 'robot_1'
  AND NOT EXISTS (
    SELECT 1 FROM task_stacks s WHERE s.device_id = d.device_id AND s.status = 'completed'
  );

-- One stack for user2 robot_3
INSERT INTO task_stacks (device_id, tasks, status)
SELECT d.device_id,
       '[{"type":"place","to":{"x":7,"y":8,"z":9}}]'::jsonb,
       'completed'
FROM devices d
JOIN users u ON u.user_id = d.user_id
WHERE u.email = 'user2@example.com' AND d.name = 'robot_2'
  AND NOT EXISTS (
    SELECT 1 FROM task_stacks s WHERE s.device_id = d.device_id AND s.status = 'completed'
  );
