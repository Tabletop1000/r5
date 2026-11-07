import { useState } from 'react';
import { Card } from '../../components/Card';
import { useRobot, useTopic } from '../../ros/RobotContext';
import { ALERT_SEVERITY, SERVICES, TOPICS, type Alert, type AlertArray, type TriggerResponse } from '../../ros/topics';
import { useDisplay } from '../../display/DisplayContext';
import { formatWhen, rosTimeToDate } from '../../display/time';

const SEVERITY_LABEL: Record<number, string> = { 0: 'Info', 1: 'Warning', 2: 'Error' };
const SEVERITY_TONE: Record<number, string> = { 0: 'neutral', 1: 'warn', 2: 'bad' };

/** Active alerts with severity, message and start time; each can be acknowledged (spec RB-2). */
export function AlertsCard() {
  const { timeZone } = useDisplay();
  const robot = useRobot();
  const msg = useTopic<AlertArray>(TOPICS.alerts).message;
  const [error, setError] = useState<string | null>(null);

  async function acknowledge(alert: Alert) {
    setError(null);
    try {
      const resp = await robot.callService<{ id: string }, TriggerResponse>(SERVICES.acknowledgeAlert, { id: alert.id });
      if (!resp.success) setError(resp.message);
    } catch (err) {
      setError((err as Error).message);
    }
  }

  return (
    <Card title="Alerts">
      {msg === null ? (
        <p className="muted">Not available — the robot_status node isn't reporting.</p>
      ) : msg.alerts.length === 0 ? (
        <p className="muted">No active alerts.</p>
      ) : (
        <ul className="alert-list">
          {msg.alerts.map((a) => (
            <li key={a.id} className={`alert tone-${SEVERITY_TONE[a.severity] ?? 'neutral'}${a.acknowledged ? ' acknowledged' : ''}`}>
              <span className="badge">{SEVERITY_LABEL[a.severity] ?? 'Alert'}</span>
              <div className="alert-text">
                <strong>{a.source}</strong> {a.message}
                <div className="muted small">Since {formatWhen(rosTimeToDate(a.since), new Date(), timeZone)}</div>
              </div>
              {a.severity > ALERT_SEVERITY.INFO &&
                (a.acknowledged ? (
                  <span className="muted small">Acknowledged</span>
                ) : (
                  <button type="button" className="secondary small" onClick={() => void acknowledge(a)}>
                    Acknowledge
                  </button>
                ))}
            </li>
          ))}
        </ul>
      )}
      {error && <p className="error">{error}</p>}
    </Card>
  );
}
