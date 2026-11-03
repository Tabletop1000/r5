import { useState } from 'react';
import { useRobot } from '../../ros/RobotContext';
import { RECORDING_STATE, SERVICES, type LatLon, type RecordingStatus, type TriggerResponse } from '../../ros/topics';

interface ControlsProps {
  status: RecordingStatus;
  /** What is being recorded, for the text: "field boundary", "keep-out zone". */
  purpose: string;
  useLabel: string;
  onUse: (points: LatLon[]) => void;
  onDiscarded?: () => void;
}

/**
 * Live state of a boundary recording on the robot, with Stop, Use and Discard (spec FM-4).
 * The robot does the recording, so this reflects it even after a reconnect or if it was
 * started from the controller.
 */
export function RecordingControls({ status, purpose, useLabel, onUse, onDiscarded }: ControlsProps) {
  const robot = useRobot();
  const [error, setError] = useState<string | null>(null);

  async function call(service: { name: string; type: string }) {
    setError(null);
    try {
      const resp = await robot.callService<object, TriggerResponse>(service, {});
      if (!resp.success) setError(resp.message);
      return resp.success;
    } catch (err) {
      setError((err as Error).message);
      return false;
    }
  }

  const fixOk = status.current_fix === status.required_fix || status.required_fix === '3D fix' || (status.required_fix === 'RTK float' && status.current_fix === 'RTK fixed');

  return (
    <div className="recording" role="status">
      {status.state === RECORDING_STATE.RECORDING && (
        <>
          <p className="recording-live">
            <span className="rec-dot" aria-hidden="true" /> Recording {purpose}. {status.message}
          </p>
          <ul className="recording-stats">
            <li>
              {status.points.length} positions · {status.length_m.toFixed(1)} m driven
            </li>
            <li className={fixOk ? undefined : 'warn-text'}>
              GPS: {status.current_fix} (needs {status.required_fix} or better)
            </li>
            {status.skipped_points > 0 && <li className="warn-text">{status.skipped_points} positions skipped because the fix wasn't good enough</li>}
          </ul>
          <button type="button" className="primary" onClick={() => void call(SERVICES.stopRecording)}>
            Stop recording
          </button>
        </>
      )}
      {status.state === RECORDING_STATE.FINISHED && (
        <>
          <p>{status.message}</p>
          <div className="button-row">
            {status.points.length >= 3 && (
              <button type="button" className="primary" onClick={() => onUse(status.points)}>
                {useLabel}
              </button>
            )}
            <button
              type="button"
              className="secondary"
              onClick={() => void call(SERVICES.clearRecording).then((ok) => ok && onDiscarded?.())}
            >
              Discard recording
            </button>
          </div>
        </>
      )}
      {status.state === RECORDING_STATE.IDLE && <p className="muted">Starting…</p>}
      {error && <p className="error">{error}</p>}
    </div>
  );
}
