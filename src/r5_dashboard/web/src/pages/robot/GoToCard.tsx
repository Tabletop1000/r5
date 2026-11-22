import { useMemo, useState } from 'react';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { useDisplay } from '../../display/DisplayContext';
import { formatLength } from '../../display/units';
import { draftFromMsg } from '../../fields/geo';
import { useFields } from '../../fields/useFields';
import { distanceM, GoToLayer } from '../../map/GoToLayer';
import { FieldsLayer, FitTo, HomeMarker, RobotMarker } from '../../map/layers';
import { MapView } from '../../map/MapView';
import { useLinkState, useRobot, useTopic } from '../../ros/RobotContext';
import { JOB_ACTIVE, SERVICES, TOPICS, type JobStatus, type LatLon, type NavSatFix, type TriggerResponse } from '../../ros/topics';

/** Matches job_manager's MAX_GOTO_M. */
const MAX_GOTO_M = 300;
/** job_manager's current action while a go-to is under way starts with this. */
const GOING = 'Going to a point';

/** Click the map to send the robot to a point, in a straight line (Robot page). */
export function GoToCard() {
  const robot = useRobot();
  const link = useLinkState();
  const { units } = useDisplay();
  const { fields, home } = useFields();
  const fix = useTopic<NavSatFix>(TOPICS.fix).message;
  const job = useTopic<JobStatus>(TOPICS.jobStatus).message;
  const [pending, setPending] = useState<LatLon | null>(null);
  const [sent, setSent] = useState<LatLon | null>(null);
  const [message, setMessage] = useState<{ text: string; error: boolean } | null>(null);

  const position = fix && fix.status.status >= 0 ? { latitude: fix.latitude, longitude: fix.longitude } : null;
  const drafts = useMemo(() => (fields ?? []).map(draftFromMsg), [fields]);
  const jobRunning = job !== null && JOB_ACTIVE.has(job.state);
  const going = job !== null && job.current_action.startsWith(GOING);
  const offline = link.status !== 'connected';
  const enabled = !offline && !jobRunning && position !== null;
  const pendingDistance = pending && position ? distanceM(position, pending) : null;

  async function send(target: LatLon) {
    setPending(null);
    setMessage(null);
    try {
      const r = await robot.callService<LatLon, TriggerResponse>(SERVICES.goTo, target, 10000);
      setMessage({ text: r.message, error: !r.success });
      if (r.success) setSent(target);
    } catch (err) {
      setMessage({ text: `Couldn't send the robot: ${(err as Error).message}`, error: true });
    }
  }

  let note: string;
  if (offline) note = 'Not connected to the robot.';
  else if (jobRunning) note = 'A spray job is running. Go-to is available when it ends.';
  else if (!position) note = "Waiting for the robot's GPS position.";
  else note = "Click the map to send the robot there. It drives in a straight line at the transit speed and doesn't steer round fields or keep-out zones. STOP at the top halts it.";

  return (
    <Card title="Go to" className="card-wide">
      <p className="muted small">{note}</p>
      {going && job && <p className="note-plain">{job.current_action}…</p>}
      {message && <p className={message.error ? 'error' : 'note-plain'}>{message.text}</p>}
      <MapView className="goto-map">
        <FitTo points={position ? [position] : home ? [home] : []} fitKey={position ? 'robot' : home ? 'home' : 'none'} />
        <FieldsLayer fields={drafts} />
        <HomeMarker position={home ?? null} />
        <GoToLayer robot={position} enabled={enabled} target={pending ?? (going ? sent : null)} units={units} onPick={setPending} />
        <RobotMarker />
      </MapView>
      <ConfirmDialog
        open={pending !== null}
        title="Send the robot here?"
        confirmLabel="Go"
        onConfirm={() => pending && pendingDistance !== null && pendingDistance <= MAX_GOTO_M && void send(pending)}
        onCancel={() => setPending(null)}
      >
        {pendingDistance !== null && pendingDistance > MAX_GOTO_M ? (
          <p className="error">
            That's {formatLength(pendingDistance, units, 0)} away. Go-to is limited to {formatLength(MAX_GOTO_M, units, 0)}. Cancel and pick a closer point.
          </p>
        ) : (
          <p>
            The robot will drive {pendingDistance !== null ? formatLength(pendingDistance, units) : ''} in a straight line to the point. It doesn't
            steer round fields or keep-out zones, so check the way is clear.
          </p>
        )}
      </ConfirmDialog>
    </Card>
  );
}
