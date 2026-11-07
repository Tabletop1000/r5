// Dates and times in the time zone chosen in Settings, or the device's own.

import type { RosTime } from '../ros/topics';

/** An IANA time zone for Intl, or undefined for the device's own. Unknown names count as the device's. */
export function resolveTimeZone(setting: unknown): string | undefined {
  if (typeof setting !== 'string' || !setting || setting === 'device') return undefined;
  try {
    new Intl.DateTimeFormat(undefined, { timeZone: setting });
    return setting;
  } catch {
    return undefined;
  }
}

export function rosTimeToDate(t: RosTime): Date {
  return new Date(t.sec * 1000 + Math.floor(t.nanosec / 1e6));
}

/** "6 Nov 2026". */
export function formatDate(t: RosTime, timeZone?: string): string {
  if (!t.sec) return '—';
  return new Date(t.sec * 1000).toLocaleDateString([], { day: 'numeric', month: 'short', year: 'numeric', timeZone });
}

/** Time of day if today, otherwise date and time. */
export function formatWhen(d: Date, now = new Date(), timeZone?: string): string {
  const day = (x: Date) => x.toLocaleDateString('en-CA', { timeZone });
  return day(d) === day(now)
    ? d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', timeZone })
    : d.toLocaleString([], { day: 'numeric', month: 'short', hour: '2-digit', minute: '2-digit', timeZone });
}
