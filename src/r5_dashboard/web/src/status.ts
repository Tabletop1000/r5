// Pure helpers behind the status strip (spec NAV-3, NF-3). Kept free of React so they can be tested.

import type { BatteryState, DiagnosticArray, DiagnosticStatus } from './ros/topics';

/** Data older than this is shown as stale (spec NF-3). */
export const STALE_AFTER_MS = 5000;

export function isStale(receivedAt: number | null, now: number): boolean {
  return receivedAt === null || now - receivedAt > STALE_AFTER_MS;
}

/** Battery charge as a whole percentage, or null when the robot doesn't report it. */
export function batteryPercent(msg: BatteryState | null): number | null {
  const p = msg?.percentage;
  // BatteryState uses NaN for "unknown"; rosbridge sends NaN as null.
  if (p === null || p === undefined || !Number.isFinite(p)) return null;
  return Math.round(Math.min(1, Math.max(0, p)) * 100);
}

export const DIAGNOSTIC_LEVEL = { OK: 0, WARN: 1, ERROR: 2, STALE: 3 } as const;

export interface AlertCounts {
  warnings: number;
  errors: number;
}

/**
 * Counts problems in an aggregated diagnostics message.
 *
 * The aggregator publishes each group ("/Sensors") as well as its members ("/Sensors/IMU"), and a
 * group copies the worst level of its members. Only entries with no members are counted, so one
 * fault is counted once.
 */
export function countAlerts(msg: DiagnosticArray | null): AlertCounts {
  const counts: AlertCounts = { warnings: 0, errors: 0 };
  if (!msg) return counts;
  const names = msg.status.map((s) => s.name);
  const isGroup = (s: DiagnosticStatus) => names.some((n) => n !== s.name && n.startsWith(`${s.name}/`));
  for (const s of msg.status) {
    if (isGroup(s)) continue;
    if (s.level === DIAGNOSTIC_LEVEL.ERROR) counts.errors += 1;
    else if (s.level === DIAGNOSTIC_LEVEL.WARN || s.level === DIAGNOSTIC_LEVEL.STALE) counts.warnings += 1;
  }
  return counts;
}

/** "4 s", "3 min", "2 h" — for "last contact ... ago". */
export function formatAge(ms: number): string {
  const s = Math.max(0, Math.floor(ms / 1000));
  if (s < 60) return `${s} s`;
  const min = Math.floor(s / 60);
  if (min < 60) return `${min} min`;
  return `${Math.floor(min / 60)} h`;
}

/** Wait before reconnect attempt number `attempt` (0-based): 1 s, 2 s, 4 s, then every 5 s. */
export function reconnectDelayMs(attempt: number): number {
  return Math.min(5000, 1000 * 2 ** Math.max(0, attempt));
}
