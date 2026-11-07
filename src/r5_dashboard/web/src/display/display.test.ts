import { describe, expect, it } from 'vitest';
import { checkForm, formFromSettings, settingsFromForm } from '../spray/settings';
import { changedSettings, checkSetting, formFromValues, type SettingDescription } from '../settings/settings';
import { formatWhen, resolveTimeZone } from './time';
import { formatArea, formatDistanceKm, formatSpeed, formatVolume, rate } from './units';

describe('units', () => {
  it('formats in either system', () => {
    expect(formatSpeed(1.609344, 'us')).toBe('1.00 mph');
    expect(formatArea(4046.8564224 * 2, 'us')).toBe('2.00 acres');
    expect(formatArea(20_000, 'metric')).toBe('2.00 ha');
    expect(formatVolume(3.785411784 * 4, 'us')).toBe('4.0 gal');
    expect(formatDistanceKm(16.09344, 'us')).toBe('10.0 mi');
  });
  it('converts a rate both ways', () => {
    expect(rate.toLPerM2(rate.fromLPerM2(0.02, 'us'), 'us')).toBeCloseTo(0.02, 12);
    expect(rate.fromLPerM2(0.02, 'us')).toBeCloseTo(21.38, 2);
  });
});

describe('the spray form in US units', () => {
  const s = { field_id: 'f', application_rate_l_per_m2: 0.02, boom_width_m: 1.5, speed_kmh: 1.0, tank_capacity_l: 50, auto_angle: true, row_angle_deg: 0 };
  it('shows US values and sends metric', () => {
    const f = formFromSettings(s, 'us');
    expect(f).toMatchObject({ rate: '21.4', boom: '4.92', speed: '0.62', tank: '13.2' });
    const back = settingsFromForm('f', f, 'us');
    expect(back.boom_width_m).toBeCloseTo(1.5, 2);
    expect(back.tank_capacity_l).toBeCloseTo(50, 0);
  });
  it('checks against the robot limits in US units', () => {
    const f = { ...formFromSettings(s, 'us'), speed: '0.8' };
    expect(checkForm(f, { maxSpeedKmh: 1.08, pumpMaxLMin: 10 }, 'us').errors.speed).toBe("The robot's top speed is 0.67 mph.");
  });
});

describe('speed settings in US units', () => {
  const max: SettingDescription = { key: 'max_speed_kmh', label: 'Maximum speed', group: 'Driving', kind: 'number', default: 1.08, help: '', unit: 'km/h', minimum: 0.1, maximum: 1.08, quantity: 'speed' };
  it('shows mph and stores km/h', () => {
    expect(formFromValues([max], { max_speed_kmh: 1.08 }, 'us')).toEqual({ max_speed_kmh: '0.67' });
    expect(checkSetting(max, '0.5', 'us')).toEqual({ value: 0.804672 });
    expect(checkSetting(max, '0.7', 'us')).toEqual({ error: 'Maximum speed can be at most 0.67 mph.' });
  });
  it("doesn't count display rounding as a change", () => {
    const values = { max_speed_kmh: 1.08 };
    expect(changedSettings([max], values, formFromValues([max], values, 'us'), 'us').changes).toEqual({});
  });
});

describe('time zones', () => {
  it('uses the device time zone for "device" or an unknown name', () => {
    expect(resolveTimeZone('device')).toBeUndefined();
    expect(resolveTimeZone('Not/AZone')).toBeUndefined();
    expect(resolveTimeZone('Pacific/Auckland')).toBe('Pacific/Auckland');
  });
  it('formats in the chosen zone', () => {
    const d = new Date('2026-11-06T23:30:00Z');
    expect(formatWhen(d, d, 'UTC')).toMatch(/23:30|11:30/);
    expect(formatWhen(d, new Date('2026-11-07T01:00:00Z'), 'UTC')).toMatch(/6 Nov|Nov 6/);
    expect(formatWhen(d, new Date('2026-11-07T01:00:00Z'), 'Pacific/Auckland')).toMatch(/12:30/);
  });
});
