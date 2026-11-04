import type { RosTime } from '../ros/topics';

export function rosTimeToDate(t: RosTime): Date {
  return new Date(t.sec * 1000 + Math.floor(t.nanosec / 1e6));
}

/** "12.4 h" below 100 h, then whole hours with separators. */
export function formatHours(h: number): string {
  return h < 100 ? `${h.toFixed(1)} h` : `${Math.round(h).toLocaleString()} h`;
}

export function formatKm(km: number): string {
  return km < 10 ? `${km.toFixed(2)} km` : `${km.toFixed(1)} km`;
}

/** toFixed without "-0.0" for values that round to zero. */
export function fixed(value: number, digits: number): string {
  const text = value.toFixed(digits);
  return /^-0\.?0*$/.test(text) ? text.slice(1) : text;
}

export function formatSpeed(mps: number): string {
  return `${fixed(mps, 3)} m/s`;
}

/** Time of day if today, otherwise date and time. */
export function formatWhen(d: Date, now = new Date()): string {
  const sameDay = d.toDateString() === now.toDateString();
  return sameDay
    ? d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    : d.toLocaleString([], { day: 'numeric', month: 'short', hour: '2-digit', minute: '2-digit' });
}
