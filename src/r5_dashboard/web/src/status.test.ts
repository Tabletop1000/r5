import { describe, expect, it } from 'vitest';
import { batteryPercent, countAlerts, formatAge, isStale, reconnectDelayMs, STALE_AFTER_MS } from './status';
import type { DiagnosticStatus } from './ros/topics';

const status = (name: string, level: number): DiagnosticStatus => ({ name, level, message: '', hardware_id: '' });

describe('batteryPercent', () => {
  it('converts 0..1 to a whole percentage', () => {
    expect(batteryPercent({ percentage: 0.876, voltage: 25, power_supply_status: 0 })).toBe(88);
  });
  it('treats NaN, null and missing messages as unknown', () => {
    expect(batteryPercent(null)).toBeNull();
    expect(batteryPercent({ percentage: null, voltage: 25, power_supply_status: 0 })).toBeNull();
    expect(batteryPercent({ percentage: Number.NaN, voltage: 25, power_supply_status: 0 })).toBeNull();
  });
  it('clamps out-of-range values', () => {
    expect(batteryPercent({ percentage: 1.2, voltage: null, power_supply_status: 0 })).toBe(100);
  });
});

describe('countAlerts', () => {
  it('returns zero for no message', () => {
    expect(countAlerts(null)).toEqual({ warnings: 0, errors: 0 });
  });
  it('counts each fault once, ignoring the groups that repeat it', () => {
    const msg = {
      status: [
        status('/Sensors', 2),
        status('/Sensors/IMU', 2),
        status('/Sensors/GPS', 1),
        status('/Motors', 0),
        status('/Motors/Left', 0),
        status('/Computer', 3),
      ],
    };
    expect(countAlerts(msg)).toEqual({ errors: 1, warnings: 2 });
  });
  it('does not treat a name prefix as a group', () => {
    const msg = { status: [status('/Motor', 1), status('/MotorDriver', 1)] };
    expect(countAlerts(msg)).toEqual({ errors: 0, warnings: 2 });
  });
});

describe('isStale', () => {
  it('is stale with no data or after the limit', () => {
    expect(isStale(null, 1000)).toBe(true);
    expect(isStale(1000, 1000 + STALE_AFTER_MS)).toBe(false);
    expect(isStale(1000, 1001 + STALE_AFTER_MS)).toBe(true);
  });
});

describe('formatAge', () => {
  it('uses seconds, minutes, then hours', () => {
    expect(formatAge(4_900)).toBe('4 s');
    expect(formatAge(185_000)).toBe('3 min');
    expect(formatAge(2 * 3_600_000 + 5)).toBe('2 h');
    expect(formatAge(-50)).toBe('0 s');
  });
});

describe('reconnectDelayMs', () => {
  it('backs off to a 5 s ceiling', () => {
    expect([0, 1, 2, 3, 10].map(reconnectDelayMs)).toEqual([1000, 2000, 4000, 5000, 5000]);
  });
});
