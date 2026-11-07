import { useState } from 'react';
import { Card, Reading, Readings } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { useRobot, useTopic } from '../../ros/RobotContext';
import { SERVICES, TOPICS, type RobotStats, type TriggerResponse } from '../../ros/topics';
import { useDisplay } from '../../display/DisplayContext';
import { formatWhen, rosTimeToDate } from '../../display/time';
import { formatDistanceKm } from '../../display/units';
import { formatHours } from '../../robot/format';

/** Operating time, distance and the service schedule (spec RB-4). */
export function UsageCard() {
  const { units, timeZone } = useDisplay();
  const robot = useRobot();
  const stats = useTopic<RobotStats>(TOPICS.stats).message;
  const [confirming, setConfirming] = useState(false);
  const [note, setNote] = useState<string | null>(null);

  async function recordService() {
    setConfirming(false);
    try {
      const resp = await robot.callService<object, TriggerResponse>(SERVICES.serviceDone, {});
      setNote(resp.message);
    } catch (err) {
      setNote(`Couldn't record the service: ${(err as Error).message}`);
    }
  }

  const used = stats ? Math.min(1, Math.max(0, 1 - stats.km_to_service / stats.service_interval_km)) : 0;
  const serviceTone = !stats ? undefined : stats.service_overdue ? 'bad' : stats.service_due_soon ? 'warn' : 'good';

  return (
    <Card
      title="Usage and service"
      actions={
        <button type="button" className="secondary small" disabled={!stats} onClick={() => setConfirming(true)}>
          Service done
        </button>
      }
    >
      <Readings>
        <Reading label="Operating time" value={stats ? formatHours(stats.operating_hours) : null} />
        <Reading label="Distance covered" value={stats ? formatDistanceKm(stats.distance_km, units) : null} />
        <Reading
          label="Next service"
          tone={serviceTone}
          value={
            stats
              ? stats.service_overdue
                ? `Overdue by ${formatDistanceKm(-stats.km_to_service, units)}`
                : `In ${formatDistanceKm(stats.km_to_service, units)}`
              : null
          }
        />
        <Reading
          label="Last service"
          value={
            stats
              ? stats.has_last_service
                ? `${formatWhen(rosTimeToDate(stats.last_service_time), new Date(), timeZone)} at ${formatDistanceKm(stats.last_service_distance_km, units)}`
                : 'None recorded'
              : null
          }
        />
      </Readings>
      {stats && (
        <div className={`meter tone-${serviceTone}`} role="meter" aria-label="Distance since last service" aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(used * 100)}>
          <div className="meter-fill" style={{ width: `${used * 100}%` }} />
          <span className="meter-caption">Service every {formatDistanceKm(stats.service_interval_km, units)}</span>
        </div>
      )}
      {note && <p className="note" role="status">{note}</p>}
      <ConfirmDialog
        open={confirming}
        title="Record a service?"
        confirmLabel="Record service"
        onConfirm={() => void recordService()}
        onCancel={() => setConfirming(false)}
      >
        <p>This resets the countdown to the next service and logs today's date and the distance.</p>
      </ConfirmDialog>
    </Card>
  );
}
