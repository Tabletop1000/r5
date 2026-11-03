// Test GPS / Test IMU (spec RB-8): listen for a few seconds and check what arrived.

import type { Imu, NavSatFix } from '../ros/topics';
import { FIX_LABEL, summarizeGnss } from './gnss';
import { quaternionNorm } from './imu';

export interface Check {
  label: string;
  ok: boolean;
  /** Not a failure, but worth knowing (e.g. no RTK during development). */
  warning?: boolean;
  detail: string;
}

export interface TestReport {
  passed: boolean;
  checks: Check[];
}

/** Messages per second from arrival times (ms), over the listening window. */
export function measuredRate(count: number, windowMs: number): number {
  return windowMs > 0 ? count / (windowMs / 1000) : 0;
}

function rateCheck(count: number, windowMs: number, expectedHz: number): Check {
  const rate = measuredRate(count, windowMs);
  return {
    label: 'Data rate',
    ok: rate >= expectedHz * 0.8,
    detail: count === 0 ? 'No messages arrived.' : `${rate.toFixed(1)} Hz (expected ${expectedHz} Hz)`,
  };
}

function report(checks: Check[]): TestReport {
  return { passed: checks.every((c) => c.ok), checks };
}

export function evaluateGpsTest(samples: NavSatFix[], windowMs: number, expectedHz: number): TestReport {
  const checks = [rateCheck(samples.length, windowMs, expectedHz)];
  if (samples.length === 0) return report(checks);

  const last = samples[samples.length - 1];
  const summary = summarizeGnss(last, null);
  checks.push({
    label: 'Position fix',
    ok: summary.quality !== 'none',
    detail: FIX_LABEL[summary.quality],
  });
  const valid =
    Number.isFinite(last.latitude) &&
    Number.isFinite(last.longitude) &&
    Math.abs(last.latitude) <= 90 &&
    Math.abs(last.longitude) <= 180 &&
    !(last.latitude === 0 && last.longitude === 0);
  checks.push({
    label: 'Position values',
    ok: valid,
    detail: valid ? `${last.latitude.toFixed(6)}, ${last.longitude.toFixed(6)}` : 'Latitude/longitude out of range',
  });
  checks.push({
    label: 'Accuracy',
    ok: true,
    warning: summary.accuracyM === null || summary.accuracyM > 0.1,
    detail:
      summary.accuracyM === null
        ? 'Not reported'
        : `About ${summary.accuracyM < 1 ? `${(summary.accuracyM * 100).toFixed(0)} cm` : `${summary.accuracyM.toFixed(1)} m`}` +
          (summary.accuracyM > 0.1 ? ' — not RTK accuracy; fine for development only' : ''),
  });
  return report(checks);
}

const GRAVITY = 9.81;

export function evaluateImuTest(samples: Imu[], windowMs: number, expectedHz: number): TestReport {
  const checks = [rateCheck(samples.length, windowMs, expectedHz)];
  if (samples.length === 0) return report(checks);

  const badQuaternion = samples.filter((s) => Math.abs(quaternionNorm(s.orientation) - 1) > 0.05).length;
  checks.push({
    label: 'Orientation',
    ok: badQuaternion === 0,
    detail: badQuaternion === 0 ? 'Valid' : `${badQuaternion} of ${samples.length} readings invalid`,
  });

  const mean = (f: (s: Imu) => number) => samples.reduce((a, s) => a + f(s), 0) / samples.length;
  const ax = mean((s) => s.linear_acceleration.x);
  const ay = mean((s) => s.linear_acceleration.y);
  const az = mean((s) => s.linear_acceleration.z);
  const g = Math.hypot(ax, ay, az);
  checks.push({
    label: 'Gravity',
    ok: Math.abs(g - GRAVITY) < 1.5,
    detail: `${g.toFixed(2)} m/s² (expected about ${GRAVITY}) — run with the robot standing still`,
  });

  const rate = Math.max(
    ...samples.map((s) => Math.hypot(s.angular_velocity.x, s.angular_velocity.y, s.angular_velocity.z)),
  );
  checks.push({
    label: 'Gyro at rest',
    ok: rate < 0.2,
    detail: `Largest turn rate ${((rate * 180) / Math.PI).toFixed(1)} °/s`,
  });
  return report(checks);
}
