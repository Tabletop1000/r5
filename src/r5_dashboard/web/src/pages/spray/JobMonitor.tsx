import { useMemo, useState } from 'react';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { useDisplay } from '../../display/DisplayContext';
import { formatArea, formatVolume, volume } from '../../display/units';
import { draftFromMsg } from '../../fields/geo';
import { linkLossText } from '../../settings/settings';
import { useSettings } from '../../settings/useSettings';
import { useFields } from '../../fields/useFields';
import { FieldsLayer, FitTo, HomeMarker, RobotMarker } from '../../map/layers';
import { MapView } from '../../map/MapView';
import { PlanLayer } from '../../map/PlanLayer';
import { useNow } from '../../hooks/useNow';
import { useLinkState, useRobot, useTopic } from '../../ros/RobotContext';
import { formatAge } from '../../status';
import { JOB_STATE, SERVICES, TOPICS, type CoveragePlan, type JobStatus, type TriggerResponse } from '../../ros/topics';
import { formatDuration } from '../../spray/settings';

const STATE_LABEL: Record<number, string> = {
  [JOB_STATE.GOING_TO_START]: 'Going to start',
  [JOB_STATE.SPRAYING]: 'Spraying',
  [JOB_STATE.TURNING]: 'Turning',
  [JOB_STATE.PAUSED]: 'Paused',
  [JOB_STATE.RETURNING_HOME]: 'Returning home to refill',
  [JOB_STATE.WAITING_FOR_REFILL]: 'Waiting for refill',
  [JOB_STATE.RETURNING_TO_WORK]: 'Returning to the field',
  [JOB_STATE.FINISHED]: 'Finished',
  [JOB_STATE.FAILED]: 'Failed',
  [JOB_STATE.STOPPED]: 'Stopped',
};

/** A running (or just ended) job: progress, liquid, time left, pause/resume/stop (spec AS-6, AS-9). */
export function JobMonitor({ status, onDone }: { status: JobStatus; onDone: () => void }) {
  const { units } = useDisplay();
  const settings = useSettings();
  const robot = useRobot();
  const link = useLinkState();
  const now = useNow();
  const { fields, home } = useFields();
  const plan = useTopic<CoveragePlan>(TOPICS.jobPlan).message;
  const [error, setError] = useState<string | null>(null);
  const [confirmRefill, setConfirmRefill] = useState(false);
  const [refilled, setRefilled] = useState(false);

  const field = fields?.find((f) => f.id === status.field_id);
  const draft = useMemo(() => (field ? draftFromMsg(field) : null), [field]);
  const ended = status.state === JOB_STATE.FINISHED || status.state === JOB_STATE.FAILED || status.state === JOB_STATE.STOPPED;
  const tankFraction = status.tank_capacity_l > 0 ? status.tank_remaining_l / status.tank_capacity_l : 0;

  const offline = link.status !== 'connected';

  async function call(service: { name: string; type: string }, request: object = {}) {
    setError(null);
    try {
      const r = await robot.callService<object, TriggerResponse>(service, request);
      if (!r.success) setError(r.message);
    } catch (err) {
      setError((err as Error).message);
    }
  }

  return (
    <div className="fields-layout">
      <div className="fields-side">
        <Card title={status.field_name ? `Spraying ${status.field_name}` : 'Spray job'}>
          <p className={`job-state state-${status.state}`} role="status">
            {status.sprayer_on && <span className="rec-dot" aria-hidden="true" />} <strong>{STATE_LABEL[status.state] ?? '—'}</strong>
            {status.sprayer_on && ' · sprayer on'}
          </p>
          {status.message && <p className={status.state === JOB_STATE.FAILED ? 'error' : 'note-plain'}>{status.message}</p>}
          {link.status !== 'connected' && !ended && (
            <p className="link-lost" role="alert">
              Link to the robot lost{link.lastContact ? ` ${formatAge(now - link.lastContact)} ago` : ''}: {linkLossText(settings?.values)}.
              These figures are the robot's last report and update again when the link returns.
            </p>
          )}
          <div className="meter" role="meter" aria-label="Progress" aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(status.progress * 100)}>
            <div className="meter-fill" style={{ width: `${status.progress * 100}%` }} />
            <span className="meter-caption">{Math.round(status.progress * 100)}% sprayed</span>
          </div>
          <dl className="readings">
            <div className="reading"><dt>Rows</dt><dd>{status.swaths_done} of {status.swath_count}</dd></div>
            <div className="reading"><dt>Area done</dt><dd>{formatArea(status.area_done_m2, units)}</dd></div>
            <div className="reading"><dt>Liquid used</dt><dd>{formatVolume(status.liquid_used_l, units)}</dd></div>
            <div className="reading">
              <dt>Tank (estimated)</dt>
              <dd className={tankFraction < 0.15 ? 'tone-text-warn' : undefined}>
                {volume.fromL(status.tank_remaining_l, units).toFixed(1)} of {formatVolume(status.tank_capacity_l, units, 0)}
              </dd>
            </div>
            {!ended && <div className="reading"><dt>Time left</dt><dd>about {formatDuration(status.remaining_s)}</dd></div>}
          </dl>
          <div className="button-row job-actions">
            {status.state === JOB_STATE.WAITING_FOR_REFILL ? (
              <button type="button" className="primary" disabled={offline} onClick={() => { setRefilled(false); setConfirmRefill(true); }}>
                Tank refilled — resume
              </button>
            ) : status.state === JOB_STATE.PAUSED ? (
              <button type="button" className="primary" disabled={offline} onClick={() => void call(SERVICES.resumeJob, { tank_refilled: false })}>
                Resume
              </button>
            ) : (
              !ended && (
                <button type="button" className="secondary" disabled={offline} onClick={() => void call(SERVICES.pauseJob)}>
                  Pause
                </button>
              )
            )}
            {!ended && (
              <button type="button" className="danger" disabled={offline} onClick={() => void call(SERVICES.stop)}>
                Stop job
              </button>
            )}
            {ended && (
              <button type="button" className="primary" onClick={onDone}>
                Set up another job
              </button>
            )}
          </div>
          {error && <p className="error">{error}</p>}
          <p className="muted small">If the link to the robot drops, the robot carries on with the job by itself.</p>
        </Card>
      </div>

      <MapView className="fields-map">
        {draft && <FitTo points={draft.boundary} fitKey={draft.id} />}
        {draft && <FieldsLayer fields={[draft]} selectedId={draft.id} />}
        {plan && plan.plan_id === status.plan_id && <PlanLayer sections={plan.sections} doneBefore={ended && status.state === JOB_STATE.FINISHED ? plan.sections.length : status.section_index} />}
        <HomeMarker position={home ?? null} />
        <RobotMarker />
      </MapView>

      <ConfirmDialog
        open={confirmRefill}
        title="Carry on spraying?"
        confirmLabel="Resume"
        onConfirm={() => {
          if (!refilled) return;
          setConfirmRefill(false);
          void call(SERVICES.resumeJob, { tank_refilled: true });
        }}
        onCancel={() => setConfirmRefill(false)}
      >
        <p>The robot will drive back to where it stopped and carry on.</p>
        <label className="inline confirm-check">
          <input type="checkbox" checked={refilled} onChange={(e) => setRefilled(e.target.checked)} /> The tank has been refilled ({formatVolume(status.tank_capacity_l, units, 0)})
        </label>
      </ConfirmDialog>
    </div>
  );
}
