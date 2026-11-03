import { describe, expect, it } from 'vitest';
import { fixed, formatHours, formatKm, formatSpeed, formatWhen, rosTimeToDate } from './format';

describe('format', () => {
  it('formats hours and km', () => {
    expect(formatHours(12.44)).toBe('12.4 h');
    expect(formatKm(3.456)).toBe('3.46 km');
    expect(formatKm(87.25)).toBe('87.3 km');
  });
  it('never shows minus zero', () => {
    expect(fixed(-0.00001, 1)).toBe('0.0');
    expect(fixed(-0.4, 0)).toBe('0');
    expect(fixed(-0.06, 2)).toBe('-0.06');
    expect(formatSpeed(-0.0000002)).toBe('0.000 m/s');
  });
  it('converts ROS time', () => {
    expect(rosTimeToDate({ sec: 1, nanosec: 500_000_000 }).getTime()).toBe(1500);
  });
  it('shows only the time for today', () => {
    const now = new Date(2026, 10, 4, 15, 0);
    expect(formatWhen(new Date(2026, 10, 4, 9, 5), now)).not.toMatch(/Nov/);
    expect(formatWhen(new Date(2026, 10, 1, 9, 5), now)).toMatch(/Nov/);
  });
});
