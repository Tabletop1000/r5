import { describe, expect, it } from 'vitest';
import type { Imu, NavSatFix } from '../ros/topics';
import { summarizeGnss } from './gnss';
import { cardinal, compassHeading, parseCalibration, quaternionToEuler } from './imu';
import { evaluateGpsTest, evaluateImuTest } from './sensorTests';
import { sideSpeeds } from './wheels';
import { ROBOT_CONFIG } from './robotConfig';

const fix = (status: number, cov = 2.25, lat = -36.85, lon = 174.76): NavSatFix => ({
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'gps_link' },
  status: { status, service: 1 },
  latitude: lat,
  longitude: lon,
  altitude: 10,
  position_covariance: [cov, 0, 0, 0, cov, 0, 0, 0, cov * 2],
  position_covariance_type: 2,
});

describe('summarizeGnss', () => {
  it('reads NavSatFix alone', () => {
    const g = summarizeGnss(fix(0), null);
    expect(g.quality).toBe('gps');
    expect(g.accuracyM).toBeCloseTo(1.5);
    expect(g.satellites).toBeNull();
    expect(g.latitude).toBe(-36.85);
  });
  it('has no position without a fix', () => {
    const g = summarizeGnss(fix(-1), null);
    expect(g.quality).toBe('none');
    expect(g.latitude).toBeNull();
  });
  it('uses NavPVT for RTK state, satellites and accuracy', () => {
    const fixed = summarizeGnss(fix(2), { fix_type: 3, flags: 0b10000001, num_sv: 21, h_acc: 14 });
    expect(fixed).toMatchObject({ quality: 'rtk-fixed', satellites: 21, accuracyM: 0.014 });
    expect(summarizeGnss(fix(2), { fix_type: 3, flags: 0b01000001, num_sv: 18, h_acc: 300 }).quality).toBe('rtk-float');
  });
});

describe('imu helpers', () => {
  it('converts quaternions', () => {
    const s = Math.SQRT1_2;
    expect(quaternionToEuler({ x: 0, y: 0, z: s, w: s }).yawDeg).toBeCloseTo(90);
    expect(quaternionToEuler({ x: s, y: 0, z: 0, w: s }).rollDeg).toBeCloseTo(90);
  });
  it('turns ENU yaw into a compass heading', () => {
    expect(compassHeading(90)).toBe(0); // facing north
    expect(compassHeading(0)).toBe(90); // facing east
    expect(compassHeading(-90)).toBe(180);
    expect(cardinal(44)).toBe('NE');
    expect(cardinal(350)).toBe('N');
  });
  it('parses calibration status', () => {
    expect(parseCalibration('{"sys": 3, "gyro": 3, "accel": 1, "mag": 0}')).toEqual({ sys: 3, gyro: 3, accel: 1, mag: 0 });
    expect(parseCalibration('nonsense')).toBeNull();
    expect(parseCalibration(undefined)).toBeNull();
  });
});

describe('sideSpeeds', () => {
  it('averages each side and converts to m/s', () => {
    const js = {
      name: ['front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint'],
      position: [],
      velocity: [1, 0.6, -0.4, -0.4],
      effort: [],
    };
    const s = sideSpeeds(js, ROBOT_CONFIG)!;
    expect(s.left).toBeCloseTo(0.08);
    expect(s.right).toBeCloseTo(-0.04);
  });
  it('is null without velocities', () => {
    expect(sideSpeeds({ name: ['a'], position: [0], velocity: [], effort: [] }, ROBOT_CONFIG)).toBeNull();
  });
});

describe('sensor tests', () => {
  it('GPS passes with a fix at the right rate, warning about accuracy', () => {
    const r = evaluateGpsTest([fix(0), fix(0), fix(0), fix(0), fix(0), fix(0)], 6000, 1);
    expect(r.passed).toBe(true);
    expect(r.checks.find((c) => c.label === 'Accuracy')?.warning).toBe(true);
  });
  it('GPS fails with no data or no fix', () => {
    expect(evaluateGpsTest([], 6000, 1).passed).toBe(false);
    expect(evaluateGpsTest(Array(6).fill(fix(-1)), 6000, 1).passed).toBe(false);
    expect(evaluateGpsTest(Array(6).fill(fix(0, 2.25, 0, 0)), 6000, 1).passed).toBe(false);
  });

  const imu = (az = 9.81, wz = 0, q = { x: 0, y: 0, z: 0, w: 1 }): Imu => ({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'imu_link' },
    orientation: q,
    angular_velocity: { x: 0, y: 0, z: wz },
    linear_acceleration: { x: 0, y: 0, z: az },
  });

  it('IMU passes at rest at the right rate', () => {
    expect(evaluateImuTest(Array(300).fill(imu()), 3000, 100).passed).toBe(true);
  });
  it('IMU fails on low rate, bad gravity or bad quaternion', () => {
    expect(evaluateImuTest(Array(30).fill(imu()), 3000, 100).passed).toBe(false);
    expect(evaluateImuTest(Array(300).fill(imu(2)), 3000, 100).passed).toBe(false);
    expect(evaluateImuTest(Array(300).fill(imu(9.81, 0, { x: 0, y: 0, z: 0, w: 0 })), 3000, 100).passed).toBe(false);
  });
});
