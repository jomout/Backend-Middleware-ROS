export type DeviceStatus = 'online' | 'offline';

export interface DeviceEntity {
  deviceId: string;
  name: string;
  status: DeviceStatus;
  userId: string;
}

export type TaskStackStatus = 'pending' | 'in_progress' | 'completed' | 'failed';

export type Coordinates = { x: number; y: number; z: number };

export type PickTask = { type: 'pick'; from: Coordinates };
export type PlaceTask = { type: 'place'; to: Coordinates };

export type Task = PickTask | PlaceTask;

export interface TaskStackEntity {
  stackId: string;
  deviceId: string;
  tasks: Task[];
  status: TaskStackStatus;
  createdAt: number;
}
