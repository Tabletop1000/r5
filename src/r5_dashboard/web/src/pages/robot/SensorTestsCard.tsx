import { useCallback, useEffect, useRef, useState } from 'react';
import { Card } from '../../components/Card';
import { useRobot } from '../../ros/RobotContext';
import { TOPICS, type Imu, type NavSatFix, type TopicSpec } from '../../ros/topics';
import { ROBOT_CONFIG } from '../../robot/robotConfig';
import { evaluateGpsTest, evaluateImuTest, type TestReport } from '../../robot/sensorTests';

const GPS_WINDOW_MS = 6000;
const IMU_WINDOW_MS = 3000;

/** Test GPS and Test IMU (spec RB-8). Each listens for a few seconds and checks what arrived. */
export function SensorTestsCard() {
  const gps = useSensorTest<NavSatFix>(TOPICS.fix, GPS_WINDOW_MS, (s, ms) => evaluateGpsTest(s, ms, ROBOT_CONFIG.gpsRateHz));
  const imu = useSensorTest<Imu>(TOPICS.imu, IMU_WINDOW_MS, (s, ms) => evaluateImuTest(s, ms, ROBOT_CONFIG.imuRateHz));
  return (
    <Card title="Sensor tests">
      <div className="test-grid">
        <SensorTest name="GPS" seconds={GPS_WINDOW_MS / 1000} test={gps} />
        <SensorTest name="IMU" seconds={IMU_WINDOW_MS / 1000} test={imu} />
      </div>
    </Card>
  );
}

interface TestState {
  running: boolean;
  report: TestReport | null;
  run: () => void;
}

function SensorTest({ name, seconds, test }: { name: string; seconds: number; test: TestState }) {
  return (
    <div className="sensor-test">
      <button type="button" className="secondary" disabled={test.running} onClick={test.run}>
        {test.running ? `Testing ${name}… (${seconds} s)` : `Test ${name}`}
      </button>
      {test.report && !test.running && (
        <div className={`test-report ${test.report.passed ? 'pass' : 'fail'}`} role="status">
          <strong>{test.report.passed ? `${name} passed` : `${name} failed`}</strong>
          <ul>
            {test.report.checks.map((c) => (
              <li key={c.label} className={!c.ok ? 'check-fail' : c.warning ? 'check-warn' : 'check-ok'}>
                <span aria-hidden="true">{!c.ok ? '✕' : c.warning ? '!' : '✓'}</span> {c.label}: {c.detail}
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}

function useSensorTest<T>(spec: TopicSpec, windowMs: number, evaluate: (samples: T[], windowMs: number) => TestReport): TestState {
  const robot = useRobot();
  const [running, setRunning] = useState(false);
  const [report, setReport] = useState<TestReport | null>(null);
  const cleanup = useRef<(() => void) | null>(null);

  useEffect(() => () => cleanup.current?.(), []);

  const run = useCallback(() => {
    if (running) return;
    setRunning(true);
    setReport(null);
    // Unthrottled, so the real data rate can be measured.
    const topic = robot.topic<T>(spec.name, spec.type, 0);
    const samples: T[] = [];
    const onMessage = (m: T) => samples.push(m);
    topic.subscribe(onMessage);
    const timer = setTimeout(() => {
      topic.unsubscribe(onMessage);
      cleanup.current = null;
      setReport(evaluate(samples, windowMs));
      setRunning(false);
    }, windowMs);
    cleanup.current = () => {
      clearTimeout(timer);
      topic.unsubscribe(onMessage);
    };
  }, [robot, spec.name, spec.type, windowMs, evaluate, running]);

  return { running, report, run };
}
