import { useEffect, useState, type ReactNode } from 'react';
import { useNow } from '../hooks/useNow';
import { useLinkState, useRobot, useTopic } from '../ros/RobotContext';
import { JOB_ACTIVE, SERVICES, TOPICS, type AlertArray, type BatteryState, type JobStatus, type Odometry, type TriggerResponse } from '../ros/topics';
import { linkLossText } from '../settings/settings';
import { useRobotName, useSettings } from '../settings/useSettings';
import { batteryPercent, countAlerts, formatAge, isStale } from '../status';

const NOT_AVAILABLE = 'Not available';

/**
 * Always-visible strip with the robot's nickname, link state, battery, current action and alerts (spec NAV-3).
 * Values older than 5 s are greyed out as stale (spec NF-3).
 */
export function StatusStrip() {
  const now = useNow();
  const link = useLinkState();
  const name = useRobotName();
  // The last settings received stay in state while disconnected.
  const settings = useSettings();
  useEffect(() => {
    document.title = `${name} Dashboard`;
  }, [name]);
  const battery = useTopic<BatteryState>(TOPICS.battery);
  const alertList = useTopic<AlertArray>(TOPICS.alerts);

  const jobTopic = useTopic<JobStatus>(TOPICS.jobStatus);
  const job = jobTopic.message;
  const odom = useTopic<Odometry>(TOPICS.odometryLocal);
  const moving =
    odom.message !== null &&
    !isStale(odom.receivedAt, now) &&
    (Math.abs(odom.message.twist.twist.linear.x) > 0.03 || Math.abs(odom.message.twist.twist.angular.z) > 0.05);
  const showStop = link.status === 'connected' && ((job !== null && JOB_ACTIVE.has(job.state)) || moving);
  const percent = batteryPercent(battery.message);
  const alerts = countAlerts(alertList.message);
  const alertTotal = alerts.errors + alerts.warnings;

  let linkText: string;
  if (link.status === 'connected') linkText = 'Connected';
  else if (link.status === 'connecting') linkText = 'Connecting…';
  else {
    const age = link.lastContact ? ` · last contact ${formatAge(now - link.lastContact)} ago` : '';
    linkText = `Disconnected — ${linkLossText(settings?.values)}${age}`;
  }

  return (
    <header className="status-strip" aria-label="Robot status">
      <div className="robot-name" title="Change the nickname on the Settings page">
        {name}
      </div>
      <StatusItem label="Link" tone={link.status === 'connected' ? 'good' : link.status === 'connecting' ? 'neutral' : 'bad'}>
        <span role="status">{linkText}</span>
      </StatusItem>

      <StatusItem label="Battery" stale={percent !== null && isStale(battery.receivedAt, now)}>
        {percent === null ? NOT_AVAILABLE : `${percent}%`}
      </StatusItem>

      <StatusItem label="Action" tone={job && JOB_ACTIVE.has(job.state) ? 'good' : 'neutral'} stale={job !== null && isStale(jobTopic.receivedAt, now)}>
        {job ? job.current_action || 'Idle' : NOT_AVAILABLE}
      </StatusItem>

      <StatusItem
        label="Alerts"
        tone={alerts.errors ? 'bad' : alerts.warnings ? 'warn' : 'neutral'}
        stale={alertList.message !== null && isStale(alertList.receivedAt, now)}
      >
        {alertList.message === null
          ? NOT_AVAILABLE
          : alertTotal === 0
            ? 'None'
            : [
                alerts.errors ? `${alerts.errors} error${alerts.errors > 1 ? 's' : ''}` : '',
                alerts.warnings ? `${alerts.warnings} warning${alerts.warnings > 1 ? 's' : ''}` : '',
              ]
                .filter(Boolean)
                .join(', ')}
      </StatusItem>
      {showStop && <StopButton />}
    </header>
  );
}

/** Always shown while a job runs or the robot moves (spec NAV-4). No confirmation: it must be quick. */
function StopButton() {
  const robot = useRobot();
  const [state, setState] = useState<'idle' | 'sending' | 'failed'>('idle');
  async function stop() {
    setState('sending');
    try {
      const resp = await robot.callService<object, TriggerResponse>(SERVICES.stop, {}, 3000);
      setState(resp.success ? 'idle' : 'failed');
    } catch {
      setState('failed');
    }
  }
  return (
    <button type="button" className="stop-button" onClick={() => void stop()} disabled={state === 'sending'}>
      {state === 'sending' ? 'Stopping…' : state === 'failed' ? 'Stop failed — try again' : 'STOP'}
    </button>
  );
}

interface ItemProps {
  label: string;
  tone?: 'good' | 'warn' | 'bad' | 'neutral';
  stale?: boolean;
  children: ReactNode;
}

function StatusItem({ label, tone = 'neutral', stale = false, children }: ItemProps) {
  return (
    <div className={`status-item tone-${tone}${stale ? ' stale' : ''}`} title={stale ? 'No update for over 5 seconds' : undefined}>
      <span className="status-label">{label}</span>
      <span className="status-value">
        <span className="dot" aria-hidden="true" />
        {children}
        {stale && <span className="visually-hidden"> (out of date)</span>}
      </span>
    </div>
  );
}
