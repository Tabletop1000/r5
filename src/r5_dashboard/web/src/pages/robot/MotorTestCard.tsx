import { useEffect, useRef, useState } from 'react';
import type { Action } from 'roslib';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { useLinkState, useRobot } from '../../ros/RobotContext';
import { ACTIONS, type MotorTestFeedback, type MotorTestGoal, type MotorTestResult } from '../../ros/topics';
import { formatSpeed } from '../../robot/format';

const SIDES = [
  { value: 0, label: 'Left' },
  { value: 1, label: 'Right' },
  { value: 2, label: 'Both' },
];

type Outcome = { kind: 'result'; result: MotorTestResult } | { kind: 'error'; message: string };

/**
 * Spin one side, or both, slowly for a short time and compare the measured wheel speed with the
 * command (spec RB-9). Speed and duration are fixed on the robot (spec RB-10).
 */
export function MotorTestCard() {
  const robot = useRobot();
  const link = useLinkState();
  const [side, setSide] = useState(2);
  const [forward, setForward] = useState(true);
  const [confirming, setConfirming] = useState(false);
  const [running, setRunning] = useState(false);
  const [feedback, setFeedback] = useState<MotorTestFeedback | null>(null);
  const [outcome, setOutcome] = useState<Outcome | null>(null);
  const goal = useRef<{ action: Action<MotorTestGoal, MotorTestFeedback, MotorTestResult>; id: string } | null>(null);

  // Leaving the page stops a running test.
  useEffect(() => () => {
    if (goal.current) goal.current.action.cancelGoal(goal.current.id);
  }, []);

  function start() {
    setConfirming(false);
    setOutcome(null);
    setFeedback(null);
    setRunning(true);
    const action = robot.action<MotorTestGoal, MotorTestFeedback, MotorTestResult>(ACTIONS.motorTest);
    const finish = (o: Outcome) => {
      goal.current = null;
      setRunning(false);
      setOutcome(o);
    };
    const id = action.sendGoal(
      { side, forward },
      (result) => finish({ kind: 'result', result }),
      (fb) => setFeedback(fb),
      (error) =>
        finish({
          kind: 'error',
          // roslib reports a cancelled goal as "Action was canceled: …".
          message: /canceled|canceling/i.test(error) ? 'Stopped.' : `The test didn't run: ${error}`,
        }),
    );
    if (id) goal.current = { action, id };
  }

  function stop() {
    if (goal.current) goal.current.action.cancelGoal(goal.current.id);
  }

  const progress = feedback && feedback.duration_s > 0 ? Math.min(1, feedback.elapsed_s / feedback.duration_s) : 0;

  return (
    <Card title="Motor test">
      <p className="muted">Runs the chosen wheels slowly for about 2 seconds and checks they turn as commanded.</p>
      <div className="control-row">
        <Segmented label="Wheels" options={SIDES} value={side} onChange={setSide} disabled={running} />
        <Segmented
          label="Direction"
          options={[
            { value: 1, label: 'Forward' },
            { value: 0, label: 'Reverse' },
          ]}
          value={forward ? 1 : 0}
          onChange={(v) => setForward(v === 1)}
          disabled={running}
        />
      </div>
      <div className="control-row">
        {running ? (
          <button type="button" className="danger" onClick={stop}>
            Stop
          </button>
        ) : (
          <button type="button" className="primary" disabled={link.status !== 'connected'} onClick={() => setConfirming(true)}>
            Run motor test
          </button>
        )}
      </div>

      {running && (
        <div className="motor-live" role="status">
          <div className="meter">
            <div className="meter-fill" style={{ width: `${progress * 100}%` }} />
          </div>
          <p>
            Left {feedback?.speeds_valid ? formatSpeed(feedback.left_speed) : '—'} · Right{' '}
            {feedback?.speeds_valid ? formatSpeed(feedback.right_speed) : '—'}
          </p>
        </div>
      )}

      {outcome && (
        <div className={`test-report ${outcome.kind === 'result' && outcome.result.success ? 'pass' : 'fail'}`} role="status">
          {outcome.kind === 'error' ? (
            <strong>{outcome.message}</strong>
          ) : (
            <>
              <strong>{outcome.result.success ? 'Passed' : 'Failed'}</strong> — {outcome.result.message}
              {outcome.result.measured_valid && (
                <table className="speed-table">
                  <thead>
                    <tr>
                      <th />
                      <th>Commanded</th>
                      <th>Measured</th>
                    </tr>
                  </thead>
                  <tbody>
                    <tr>
                      <th>Left</th>
                      <td>{formatSpeed(outcome.result.commanded_left)}</td>
                      <td>{formatSpeed(outcome.result.measured_left)}</td>
                    </tr>
                    <tr>
                      <th>Right</th>
                      <td>{formatSpeed(outcome.result.commanded_right)}</td>
                      <td>{formatSpeed(outcome.result.measured_right)}</td>
                    </tr>
                  </tbody>
                </table>
              )}
            </>
          )}
        </div>
      )}

      <ConfirmDialog
        open={confirming}
        title="Robot may move"
        confirmLabel="Run test"
        danger
        onConfirm={start}
        onCancel={() => setConfirming(false)}
      >
        <p>
          <strong>Lift the wheels or clear the area</strong> before continuing.{' '}
          {side === 2 ? 'All four wheels' : `The ${SIDES[side].label.toLowerCase()}-side wheels`} will turn{' '}
          {forward ? 'forwards' : 'in reverse'} slowly for about 2 seconds.
        </p>
      </ConfirmDialog>
    </Card>
  );
}

function Segmented({
  label,
  options,
  value,
  onChange,
  disabled,
}: {
  label: string;
  options: { value: number; label: string }[];
  value: number;
  onChange: (v: number) => void;
  disabled?: boolean;
}) {
  return (
    <fieldset className="segmented" disabled={disabled}>
      <legend>{label}</legend>
      {options.map((o) => (
        <label key={o.value} className={o.value === value ? 'active' : undefined}>
          <input type="radio" name={label} checked={o.value === value} onChange={() => onChange(o.value)} />
          {o.label}
        </label>
      ))}
    </fieldset>
  );
}
