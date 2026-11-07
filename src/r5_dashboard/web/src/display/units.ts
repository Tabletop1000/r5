// Units for what the dashboard shows and what the user types (Settings: Units). The robot works in
// metric throughout, so values are converted at the edges: to show them and from inputs. Sensor
// readings on the Robot page stay in SI units. The robot's own messages use
// r5_dashboard_server/units.py.

export type Units = 'metric' | 'us';

export const KM_PER_MILE = 1.609344;
export const L_PER_US_GALLON = 3.785411784;
export const M2_PER_ACRE = 4046.8564224;
export const M_PER_FT = 0.3048;

export function unitsFrom(value: unknown): Units {
  return value === 'us' ? 'us' : 'metric';
}

// ---- Conversions between metric and the display units ----

export const speed = {
  unit: (u: Units) => (u === 'us' ? 'mph' : 'km/h'),
  fromKmh: (kmh: number, u: Units) => (u === 'us' ? kmh / KM_PER_MILE : kmh),
  toKmh: (v: number, u: Units) => (u === 'us' ? v * KM_PER_MILE : v),
};

export const volume = {
  unit: (u: Units) => (u === 'us' ? 'gal' : 'L'),
  fromL: (l: number, u: Units) => (u === 'us' ? l / L_PER_US_GALLON : l),
  toL: (v: number, u: Units) => (u === 'us' ? v * L_PER_US_GALLON : v),
};

export const length = {
  unit: (u: Units) => (u === 'us' ? 'ft' : 'm'),
  fromM: (m: number, u: Units) => (u === 'us' ? m / M_PER_FT : m),
  toM: (v: number, u: Units) => (u === 'us' ? v * M_PER_FT : v),
};

/** Application rate. Metric uses L/m², as the robot does; US uses gal/acre. */
export const rate = {
  unit: (u: Units) => (u === 'us' ? 'gal/acre' : 'L/m²'),
  fromLPerM2: (r: number, u: Units) => (u === 'us' ? (r * M2_PER_ACRE) / L_PER_US_GALLON : r),
  toLPerM2: (v: number, u: Units) => (u === 'us' ? (v * L_PER_US_GALLON) / M2_PER_ACRE : v),
};

// ---- Formatting ----

export function formatSpeed(kmh: number, u: Units): string {
  return `${speed.fromKmh(kmh, u).toFixed(2)} ${speed.unit(u)}`;
}

export function formatVolume(litres: number, u: Units, digits = 1): string {
  return `${volume.fromL(litres, u).toFixed(digits)} ${volume.unit(u)}`;
}

export function formatFlow(lMin: number, u: Units): string {
  return u === 'us' ? `${(lMin / L_PER_US_GALLON).toFixed(2)} gal/min` : `${lMin.toFixed(2)} L/min`;
}

/** Short lengths: headlands, booms, recorded tracks. */
export function formatLength(m: number, u: Units, digits = 1): string {
  return `${length.fromM(m, u).toFixed(digits)} ${length.unit(u)}`;
}

/** Long distances: km, or miles. */
export function formatDistanceKm(km: number, u: Units): string {
  const v = u === 'us' ? km / KM_PER_MILE : km;
  const unit = u === 'us' ? 'mi' : 'km';
  return `${Math.abs(v) < 10 ? v.toFixed(2) : v.toFixed(1)} ${unit}`;
}

export function formatArea(m2: number, u: Units = 'metric'): string {
  if (u === 'us') {
    const acres = m2 / M2_PER_ACRE;
    return acres >= 0.1 ? `${acres.toFixed(2)} acres` : `${Math.round(m2 / M_PER_FT ** 2).toLocaleString()} ft²`;
  }
  const ha = m2 / 10_000;
  return ha >= 0.1 ? `${ha.toFixed(2)} ha` : `${Math.round(m2).toLocaleString()} m²`;
}

export function formatRate(lPerM2: number, u: Units): string {
  return u === 'us' ? `${rate.fromLPerM2(lPerM2, u).toFixed(1)} gal/acre` : `${lPerM2.toFixed(3)} L/m²`;
}

/** A number for an input box, without trailing zeros. */
export function inputNumber(value: number, digits: number): string {
  return String(Number(value.toFixed(digits)));
}
