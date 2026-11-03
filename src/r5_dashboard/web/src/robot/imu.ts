import type { Quaternion } from '../ros/topics';

export interface Euler {
  rollDeg: number;
  pitchDeg: number;
  /** ROS yaw: anticlockwise from east (ENU). */
  yawDeg: number;
}

const DEG = 180 / Math.PI;

export function quaternionToEuler(q: Quaternion): Euler {
  const sinrCosp = 2 * (q.w * q.x + q.y * q.z);
  const cosrCosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const sinp = 2 * (q.w * q.y - q.z * q.x);
  const sinyCosp = 2 * (q.w * q.z + q.x * q.y);
  const cosyCosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return {
    rollDeg: Math.atan2(sinrCosp, cosrCosp) * DEG,
    pitchDeg: Math.asin(Math.max(-1, Math.min(1, sinp))) * DEG,
    yawDeg: Math.atan2(sinyCosp, cosyCosp) * DEG,
  };
}

/** Compass heading, clockwise from north, 0–360°, from ROS ENU yaw. */
export function compassHeading(yawDeg: number): number {
  return (((90 - yawDeg) % 360) + 360) % 360;
}

const POINTS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];

export function cardinal(headingDeg: number): string {
  return POINTS[Math.round(headingDeg / 45) % 8];
}

export function quaternionNorm(q: Quaternion): number {
  return Math.hypot(q.x, q.y, q.z, q.w);
}

export interface ImuCalibration {
  sys: number;
  gyro: number;
  accel: number;
  mag: number;
}

/** Parses the BNO055 driver's calib_status string. Each value is 0 (none) to 3 (fully calibrated). */
export function parseCalibration(data: string | undefined): ImuCalibration | null {
  if (!data) return null;
  try {
    const v = JSON.parse(data) as Partial<ImuCalibration>;
    const keys = ['sys', 'gyro', 'accel', 'mag'] as const;
    if (!keys.every((k) => typeof v[k] === 'number')) return null;
    return v as ImuCalibration;
  } catch {
    return null;
  }
}
