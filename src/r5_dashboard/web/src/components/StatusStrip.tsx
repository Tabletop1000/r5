import type { ReactNode } from 'react';
import { useNow } from '../hooks/useNow';
import { useLinkState, useTopic } from '../ros/RobotContext';
import { TOPICS, type BatteryState, type DiagnosticArray } from '../ros/topics';
import { batteryPercent, countAlerts, formatAge, isStale } from '../status';

const NOT_AVAILABLE = 'Not available';

/**
 * Always-visible strip with link state, battery, current action and alerts (spec NAV-3).
 * Values older than 5 s are greyed out as stale (spec NF-3).
 */
export function StatusStrip() {
  const now = useNow();
  const link = useLinkState();
  const battery = useTopic<BatteryState>(TOPICS.battery);
  const diagnostics = useTopic<DiagnosticArray>(TOPICS.diagnostics);

  const percent = batteryPercent(battery.message);
  const alerts = countAlerts(diagnostics.message);
  const alertTotal = alerts.errors + alerts.warnings;

  let linkText: string;
  if (link.status === 'connected') linkText = 'Connected';
  else if (link.status === 'connecting') linkText = 'Connecting…';
  else {
    const age = link.lastContact ? ` · last contact ${formatAge(now - link.lastContact)} ago` : '';
    linkText = `Disconnected — robot continues on its own${age}`;
  }

  return (
    <header className="status-strip" aria-label="Robot status">
      <StatusItem label="Link" tone={link.status === 'connected' ? 'good' : link.status === 'connecting' ? 'neutral' : 'bad'}>
        <span role="status">{linkText}</span>
      </StatusItem>

      <StatusItem label="Battery" stale={percent !== null && isStale(battery.receivedAt, now)}>
        {percent === null ? NOT_AVAILABLE : `${percent}%`}
      </StatusItem>

      {/* The job manager that reports the current action doesn't exist yet (spec 5.2). */}
      <StatusItem label="Action">{NOT_AVAILABLE}</StatusItem>

      <StatusItem
        label="Alerts"
        tone={alerts.errors ? 'bad' : alerts.warnings ? 'warn' : 'neutral'}
        stale={diagnostics.message !== null && isStale(diagnostics.receivedAt, now)}
      >
        {diagnostics.message === null
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
    </header>
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
