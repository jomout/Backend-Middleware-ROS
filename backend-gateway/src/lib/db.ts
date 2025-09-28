import { DeviceEntity, TaskStackEntity } from '@/dtos/entities';

// In-memory stores (for demo only). Replace with real DB in production.
const devices = new Map<string, DeviceEntity>();
const taskStacks = new Map<string, TaskStackEntity>();

// Simple id generator for demo
const genId = (prefix: string) => `${prefix}_${Math.random().toString(36).slice(2, 10)}`;

/**
 * In-memory DB operations
 */
export const db = {
  // ==================================
  // Device operations
  // ==================================

  /**
   * Creates a new device
   * @param input - The device details
   * @returns The created device
   */
  createDevice(input: Omit<DeviceEntity, 'deviceId'> & { deviceId?: string }) {
    const deviceId = input.deviceId ?? genId('dev');
    const entity: DeviceEntity = { deviceId, name: input.name, status: input.status, userId: input.userId };
    devices.set(deviceId, entity);
    return entity;
  },

  /**
   * Retrieves a device by its ID
   * @param deviceId - The ID of the device to retrieve
   * @returns The device or null if not found
   */
  getDevice(deviceId: string) {
    return devices.get(deviceId) ?? null;
  },

  /**
   * Lists all devices for a given user
   * @param userId - The ID of the user whose devices to list
   * @returns An array of devices belonging to the user
   */
  listDevicesByUser(userId: string) {
    return [...devices.values()].filter(d => d.userId === userId);
  },

  // ==================================
  // Task stack operations
  // ==================================

  /**
   * Creates a new task stack
   * @param input - The task stack details
   * @returns The created task stack
   */
  createTaskStack(input: Omit<TaskStackEntity, 'stackId' | 'createdAt'> & { stackId?: string }) {
    const stackId = input.stackId ?? genId('stack');
    const entity: TaskStackEntity = { ...input, stackId, createdAt: Date.now() };
    taskStacks.set(stackId, entity);
    return entity;
  },

  /**
   * Lists all task stacks for a given device
   * @param deviceId - The ID of the device whose task stacks to list
   * @returns An array of task stacks belonging to the device
   */
  listTaskStacksByDevice(deviceId: string) {
    return [...taskStacks.values()].filter(s => s.deviceId === deviceId);
  },
};
