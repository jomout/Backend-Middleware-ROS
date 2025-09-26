import type { DeviceStatus, Task, TaskStackStatus } from './entities';

// POST /api/device/stack
export interface CreateTaskStackBodyDto {
  // deviceId is optional; if omitted, the server should infer from auth context
  deviceId?: string;
  tasks: Task[];
}

export interface CreateTaskStackResponseDto {
  stackId: string;
  status: TaskStackStatus;
}

// GET /api/device/summary
export interface DeviceSummaryItemDto {
  deviceId: string;
  name: string;
  status: DeviceStatus;
  pendingTasks: number;
  activeTasks: number;
}

export type GetDeviceSummaryResponseDto = DeviceSummaryItemDto[];
