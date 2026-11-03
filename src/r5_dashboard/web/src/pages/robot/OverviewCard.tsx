import { Card, Reading, Readings } from '../../components/Card';
import { useRoundTrip } from '../../hooks/useRoundTrip';
import { useTopic } from '../../ros/RobotContext';
import { TOPICS, type BatteryState, type DiagnosticStatus, type Float32, type Temperature } from '../../ros/topics';
import { ROBOT_CONFIG } from '../../robot/robotConfig';
import { batteryPercent } from '../../status';
import { RobotIllustration } from './RobotIllustration';

// sensor_msgs/BatteryState power_supply_status
const SUPPLY_STATUS: Record<number, string> = { 1: 'Charging', 2: 'Discharging', 3: 'Not charging', 4: 'Full' };

/** Battery, temperature, link, current action and versions (spec 4.4 status table). */
export function OverviewCard() {
  const battery = useTopic<BatteryState>(TOPICS.battery).message;
  const temp = useTopic<Temperature>(TOPICS.imuTemperature).message;
  const signal = useTopic<Float32>(TOPICS.wifiSignal).message;
  const versions = useTopic<DiagnosticStatus>(TOPICS.versions).message;
  const rtt = useRoundTrip();

  const percent = batteryPercent(battery);
  const supply = battery ? SUPPLY_STATUS[battery.power_supply_status] : undefined;
  const tempC = temp && Number.isFinite(temp.temperature) ? temp.temperature : null;

  return (
    <Card title="Overview" className="overview">
      <RobotIllustration />
      <Readings>
        <Reading
          label="Battery"
          value={percent === null ? null : `${percent}%${battery?.voltage ? ` · ${battery.voltage.toFixed(1)} V` : ''}${supply ? ` · ${supply}` : ''}`}
          tone={percent === null ? undefined : percent < 20 ? 'bad' : percent < 40 ? 'warn' : 'good'}
        />
        <Reading
          label="Temperature (IMU)"
          value={tempC === null ? null : `${tempC.toFixed(0)} °C`}
          tone={tempC !== null && tempC > ROBOT_CONFIG.temperatureWarnC ? 'warn' : undefined}
        />
        <Reading label="WiFi signal" value={signal ? `${signal.data.toFixed(0)} dBm` : null} tone={signal ? signalTone(signal.data) : undefined} />
        <Reading label="Round trip" value={rtt === null ? null : `${rtt} ms`} tone={rtt !== null && rtt > 500 ? 'warn' : undefined} />
        {/* Reported by the job manager, which isn't built yet (spec 5.2). */}
        <Reading label="Current action" value={null} />
        <Reading
          label="Versions"
          value={versions && versions.values.length ? versions.values.map((kv) => `${kv.key} ${kv.value}`).join(' · ') : null}
        />
      </Readings>
    </Card>
  );
}

function signalTone(dbm: number) {
  if (dbm >= -67) return 'good' as const;
  if (dbm >= -80) return 'warn' as const;
  return 'bad' as const;
}
