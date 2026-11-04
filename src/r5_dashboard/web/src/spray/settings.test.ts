import { describe, expect, it } from 'vitest';
import { checkForm, flowLMin, formatDuration, formFromSettings, maxSpeedForPump, settingsFromForm } from './settings';

const limits = { maxSpeedKmh: 1.08, pumpMaxLMin: 10 };
const form = { rate: '0.020', boom: '1.5', speed: '1', tank: '50', autoAngle: true, angle: '0' };

describe('spray settings', () => {
  it('computes flow as in spec AS-1', () => {
    expect(flowLMin(0.02, 1.5, 1)).toBeCloseTo(0.5);
    expect(maxSpeedForPump(0.02, 1.5, 0.5)).toBeCloseTo(1);
  });
  it('accepts sensible values', () => {
    const c = checkForm(form, limits);
    expect(c.errors).toEqual({});
    expect(c.flow).toBeCloseTo(0.5);
  });
  it('flags bad values and the robot top speed', () => {
    expect(checkForm({ ...form, rate: '' }, limits).errors.rate).toBeTruthy();
    expect(checkForm({ ...form, rate: '3' }, limits).errors.rate).toMatch('1 L/m²');
    expect(checkForm({ ...form, speed: '2' }, limits).errors.speed).toMatch('top speed');
    expect(checkForm({ ...form, autoAngle: false, angle: 'x' }, limits).errors.angle).toBeTruthy();
  });
  it('suggests a speed the pump can manage', () => {
    const c = checkForm({ ...form, rate: '0.2' }, { maxSpeedKmh: 1.08, pumpMaxLMin: 2.5 });
    expect(c.suggestedSpeed).toBe(0.5);
    expect(c.errors.speed).toMatch('0.50 km/h');
  });
  it('round-trips settings and normalises the row direction', () => {
    const s = settingsFromForm('f1', { ...form, autoAngle: false, angle: '200' });
    expect(s.row_angle_deg).toBe(20);
    expect(formFromSettings({ ...s, auto_angle: true }).rate).toBe('0.020');
  });
  it('formats durations', () => {
    expect(formatDuration(45 * 60)).toBe('45 min');
    expect(formatDuration(135 * 60)).toBe('2 h 15 min');
  });
});
