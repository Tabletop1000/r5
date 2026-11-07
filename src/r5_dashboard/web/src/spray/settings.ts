// Spray settings maths and checks shown while typing (spec AS-1, AS-2). The robot checks again.
// The form holds what the user typed, in the units chosen in Settings; the robot gets metric.

import { formatFlow, formatSpeed, inputNumber, length, rate as rateUnits, speed as speedUnits, volume, type Units } from '../display/units';
import type { SpraySettings } from '../ros/topics';

/** Spec AS-1: flow (L/min) = rate (L/m²) × boom width (m) × speed (km/h) × 1000 / 60. */
export function flowLMin(rate: number, boomWidth: number, speedKmh: number): number {
  return (rate * boomWidth * speedKmh * 1000) / 60;
}

/** The fastest speed at which the pump can still deliver the rate. */
export function maxSpeedForPump(rate: number, boomWidth: number, pumpMaxLMin: number): number {
  return (pumpMaxLMin * 60) / (rate * boomWidth * 1000);
}

export interface SettingsForm {
  rate: string;
  boom: string;
  speed: string;
  tank: string;
  autoAngle: boolean;
  angle: string;
}

export interface Limits {
  maxSpeedKmh: number;
  pumpMaxLMin: number;
}

export function formFromSettings(s: SpraySettings, u: Units = 'metric'): SettingsForm {
  return {
    rate: u === 'us' ? rateUnits.fromLPerM2(s.application_rate_l_per_m2, u).toFixed(1) : s.application_rate_l_per_m2.toFixed(3),
    boom: inputNumber(length.fromM(s.boom_width_m, u), 2),
    speed: inputNumber(speedUnits.fromKmh(s.speed_kmh, u), 2),
    tank: inputNumber(volume.fromL(s.tank_capacity_l, u), 1),
    autoAngle: s.auto_angle,
    angle: String(Math.round(s.row_angle_deg)),
  };
}

export function settingsFromForm(fieldId: string, f: SettingsForm, u: Units = 'metric'): SpraySettings {
  return {
    field_id: fieldId,
    application_rate_l_per_m2: rateUnits.toLPerM2(Number(f.rate), u),
    boom_width_m: length.toM(Number(f.boom), u),
    speed_kmh: speedUnits.toKmh(Number(f.speed), u),
    tank_capacity_l: volume.toL(Number(f.tank), u),
    auto_angle: f.autoAngle,
    row_angle_deg: f.autoAngle ? 0 : ((Number(f.angle) % 180) + 180) % 180,
  };
}

export interface FormCheck {
  errors: Partial<Record<keyof SettingsForm, string>>;
  /** L/min. */
  flow: number | null;
  /** Set when the pump can't keep up: the highest speed that works, in the form's units. */
  suggestedSpeed: number | null;
}

export function checkForm(f: SettingsForm, limits: Limits | null, u: Units = 'metric'): FormCheck {
  const errors: FormCheck['errors'] = {};
  const num = (v: string) => (v.trim() === '' ? NaN : Number(v));
  const rate = rateUnits.toLPerM2(num(f.rate), u);
  const boom = length.toM(num(f.boom), u);
  const speed = speedUnits.toKmh(num(f.speed), u);
  const tank = num(f.tank);
  if (!(rate > 0)) errors.rate = 'Enter a rate above zero.';
  else if (rate > 1 + 1e-9) errors.rate = u === 'us' ? `At most ${Math.floor(rateUnits.fromLPerM2(1, u))} gal/acre.` : 'At most 1 L/m².';
  if (!(boom > 0) || boom > 10 + 1e-9) errors.boom = u === 'us' ? 'Between 0 and 32.8 ft.' : 'Between 0 and 10 m.';
  if (!(speed > 0)) errors.speed = 'Enter a speed above zero.';
  else if (limits && speed > limits.maxSpeedKmh + 1e-6) errors.speed = `The robot's top speed is ${formatSpeed(limits.maxSpeedKmh, u)}.`;
  if (!(tank > 0)) errors.tank = 'Enter the tank size.';
  if (!f.autoAngle && !Number.isFinite(num(f.angle))) errors.angle = 'Enter a direction in degrees.';

  let flow: number | null = null;
  let suggestedSpeed: number | null = null;
  if (!errors.rate && !errors.boom && !errors.speed) {
    flow = flowLMin(rate, boom, speed);
    if (limits && flow > limits.pumpMaxLMin + 1e-9) {
      const top = speedUnits.fromKmh(maxSpeedForPump(rate, boom, limits.pumpMaxLMin), u);
      suggestedSpeed = Math.floor(top * 100 + 1e-6) / 100;
      errors.speed = `The pump gives at most ${formatFlow(limits.pumpMaxLMin, u)}. Use ${suggestedSpeed.toFixed(2)} ${speedUnits.unit(u)} or less.`;
    }
  }
  return { errors, flow, suggestedSpeed };
}

export function formatDuration(seconds: number): string {
  const minutes = Math.max(0, Math.round(seconds / 60));
  return minutes >= 60 ? `${Math.floor(minutes / 60)} h ${String(minutes % 60).padStart(2, '0')} min` : `${minutes} min`;
}
