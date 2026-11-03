import type { JointState } from '../ros/topics';

export interface WheelConfig {
  wheelRadiusM: number;
  leftWheelJoints: readonly string[];
  rightWheelJoints: readonly string[];
}

/** Average ground speed of each side, m/s, or null if joint velocities aren't reported. */
export function sideSpeeds(js: JointState | null, cfg: WheelConfig): { left: number; right: number } | null {
  if (!js || !js.velocity || js.velocity.length === 0) return null;
  const speedOf = (names: readonly string[]) => {
    const values = names
      .map((n) => js.velocity[js.name.indexOf(n)])
      .filter((v): v is number => typeof v === 'number' && Number.isFinite(v));
    return values.length ? (values.reduce((a, b) => a + b, 0) / values.length) * cfg.wheelRadiusM : null;
  };
  const left = speedOf(cfg.leftWheelJoints);
  const right = speedOf(cfg.rightWheelJoints);
  return left === null || right === null ? null : { left, right };
}
