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
INSERT INTO devices (device_id, name, status, user_id)
SELECT '18f2d2a4-d391-4346-8dc2-2f195b05d52a', 'robot_1', 'online', u.user_id
FROM users u
WHERE u.email = 'user1@example.com';

INSERT INTO devices (device_id, name, status, user_id)
SELECT 'ab1e7619-b3d1-474d-9d95-90e84598ee7d', 'robot_3', 'offline', u.user_id
FROM users u
WHERE u.email = 'user1@example.com';

-- Device for User2
INSERT INTO devices (device_id, name, status, user_id)
SELECT 'a3369c60-c84d-49d5-85b8-972d007f8933', 'robot_2', 'online', u.user_id
FROM users u
WHERE u.email = 'user2@example.com';


-- Task stacks -----------------------------------------------------
-- Seed a couple of stacks for user1 robot_1
INSERT INTO task_stacks (device_id, tasks, status)
SELECT d.device_id,
       '[{"type":"pick","from":{"x":1,"y":2,"z":3}},{"type":"place","to":{"x":4,"y":5,"z":6}}]'::jsonb,
       'completed'
FROM devices d
JOIN users u ON u.user_id = d.user_id
WHERE u.email = 'user1@example.com' AND d.device_id = '18f2d2a4-d391-4346-8dc2-2f195b05d52a';

-- One stack for user2 robot_3
INSERT INTO task_stacks (device_id, tasks, status)
SELECT d.device_id,
       '[{"type":"place","to":{"x":7,"y":8,"z":9}}]'::jsonb,
       'completed'
FROM devices d
JOIN users u ON u.user_id = d.user_id
WHERE u.email = 'user2@example.com' AND d.device_id = 'a3369c60-c84d-49d5-85b8-972d007f8933';
