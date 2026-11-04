import { useState } from 'react';
import { useTopic } from '../../ros/RobotContext';
import { JOB_ACTIVE, TOPICS, type JobStatus } from '../../ros/topics';
import { JobMonitor } from './JobMonitor';
import { JobSetup } from './JobSetup';

type Mode = 'area' | 'spot';

/** Spray page with Area Spray and Spot Spray (spec NAV-5, 4.2). */
export function SprayPage() {
  const [mode, setMode] = useState<Mode>('area');
  return (
    <section className="page wide">
      <h1>Spray</h1>
      <div className="tabs" role="tablist" aria-label="Spray mode">
        <Tab id="area" label="Area Spray" mode={mode} onSelect={setMode} />
        <Tab id="spot" label="Spot Spray" mode={mode} onSelect={setMode} />
      </div>
      {mode === 'area' ? (
        <AreaSpray />
      ) : (
        <div className="placeholder" role="tabpanel">
          <p>Coming soon. Spot spraying is not available yet.</p>
        </div>
      )}
    </section>
  );
}

/** Shows the running job if there is one (also after a reconnect, spec AS-7), otherwise setup. */
function AreaSpray() {
  const status = useTopic<JobStatus>(TOPICS.jobStatus).message;
  // A finished, stopped or failed job stays on screen until the user moves on.
  const [dismissedPlan, setDismissedPlan] = useState<number | null>(null);
  const showMonitor =
    status !== null && status.state !== 0 && (JOB_ACTIVE.has(status.state) || status.plan_id !== dismissedPlan);
  return showMonitor ? <JobMonitor status={status} onDone={() => setDismissedPlan(status.plan_id)} /> : <JobSetup />;
}

function Tab({ id, label, mode, onSelect }: { id: Mode; label: string; mode: Mode; onSelect: (m: Mode) => void }) {
  return (
    <button type="button" role="tab" aria-selected={mode === id} className={mode === id ? 'tab active' : 'tab'} onClick={() => onSelect(id)}>
      {label}
    </button>
  );
}
