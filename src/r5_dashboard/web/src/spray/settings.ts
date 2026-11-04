// Spray settings maths and checks shown while typing (spec AS-1, AS-2). The robot checks again.

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

export function formFromSettings(s: SpraySettings): SettingsForm {
  return {
    rate: s.application_rate_l_per_m2.toFixed(3),
    boom: String(s.boom_width_m),
    speed: String(s.speed_kmh),
    tank: String(s.tank_capacity_l),
    autoAngle: s.auto_angle,
    angle: String(Math.round(s.row_angle_deg)),
  };
}

export function settingsFromForm(fieldId: string, f: SettingsForm): SpraySettings {
  return {
    field_id: fieldId,
    application_rate_l_per_m2: Number(f.rate),
    boom_width_m: Number(f.boom),
    speed_kmh: Number(f.speed),
    tank_capacity_l: Number(f.tank),
    auto_angle: f.autoAngle,
    row_angle_deg: f.autoAngle ? 0 : ((Number(f.angle) % 180) + 180) % 180,
  };
}

export interface FormCheck {
  errors: Partial<Record<keyof SettingsForm, string>>;
  flow: number | null;
  /** Set when the pump can't keep up: the highest speed that works. */
  suggestedSpeed: number | null;
}

export function checkForm(f: SettingsForm, limits: Limits | null): FormCheck {
  const errors: FormCheck['errors'] = {};
  const num = (v: string) => (v.trim() === '' ? NaN : Number(v));
  const rate = num(f.rate);
  const boom = num(f.boom);
  const speed = num(f.speed);
  const tank = num(f.tank);
  if (!(rate > 0)) errors.rate = 'Enter a rate above zero.';
  else if (rate > 1) errors.rate = 'At most 1 L/m².';
  if (!(boom > 0) || boom > 10) errors.boom = 'Between 0 and 10 m.';
  if (!(speed > 0)) errors.speed = 'Enter a speed above zero.';
  else if (limits && speed > limits.maxSpeedKmh + 1e-9) errors.speed = `The robot's top speed is ${limits.maxSpeedKmh.toFixed(2)} km/h.`;
  if (!(tank > 0)) errors.tank = 'Enter the tank size.';
  if (!f.autoAngle && !Number.isFinite(num(f.angle))) errors.angle = 'Enter a direction in degrees.';

  let flow: number | null = null;
  let suggestedSpeed: number | null = null;
  if (!errors.rate && !errors.boom && !errors.speed) {
    flow = flowLMin(rate, boom, speed);
    if (limits && flow > limits.pumpMaxLMin + 1e-9) {
      suggestedSpeed = Math.floor(maxSpeedForPump(rate, boom, limits.pumpMaxLMin) * 100 + 1e-6) / 100;
      errors.speed = `The pump gives at most ${limits.pumpMaxLMin.toFixed(1)} L/min. Use ${suggestedSpeed.toFixed(2)} km/h or less.`;
    }
  }
  return { errors, flow, suggestedSpeed };
}

export function formatDuration(seconds: number): string {
  const minutes = Math.max(0, Math.round(seconds / 60));
  return minutes >= 60 ? `${Math.floor(minutes / 60)} h ${String(minutes % 60).padStart(2, '0')} min` : `${minutes} min`;
}
