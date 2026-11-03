import { Card, Reading, Readings } from '../../components/Card';
import { useTopic, useTopicWithRate } from '../../ros/RobotContext';
import { TOPICS, type Imu, type JointState, type NavPvt, type NavSatFix, type StringMsg, type Temperature } from '../../ros/topics';
import { fixed, formatSpeed } from '../../robot/format';
import { FIX_LABEL, summarizeGnss } from '../../robot/gnss';
import { cardinal, compassHeading, parseCalibration, quaternionToEuler } from '../../robot/imu';
import { ROBOT_CONFIG } from '../../robot/robotConfig';
import { sideSpeeds } from '../../robot/wheels';

/** GNSS readings (spec RB-5). */
export function GnssCard() {
  const fix = useTopicWithRate<NavSatFix>(TOPICS.fix);
  const pvt = useTopic<NavPvt>(TOPICS.navPvt).message;
  const g = summarizeGnss(fix.message, pvt);
  const qualityTone = !fix.message ? undefined : g.quality === 'rtk-fixed' ? 'good' : g.quality === 'none' ? 'bad' : 'warn';
  return (
    <Card title="GPS">
      <Readings>
        <Reading label="Fix" value={fix.message ? FIX_LABEL[g.quality] : null} tone={qualityTone} />
        <Reading
          label="Accuracy"
          value={g.accuracyM === null ? null : g.accuracyM < 1 ? `±${(g.accuracyM * 100).toFixed(0)} cm` : `±${g.accuracyM.toFixed(1)} m`}
        />
        <Reading label="Satellites" value={g.satellites} />
        <Reading
          label="Position"
          value={g.latitude === null || g.longitude === null ? null : `${g.latitude.toFixed(7)}, ${g.longitude.toFixed(7)}`}
        />
        <Reading label="Update rate" value={fix.rateHz === null ? null : `${fix.rateHz.toFixed(1)} Hz`} />
        {/* Needs the RTK base station link reporting on the robot (TODO_ROBOT.md). */}
        <Reading label="Correction age" value={null} />
      </Readings>
    </Card>
  );
}

const CAL_LABEL = ['None', 'Low', 'Medium', 'Full'];

/** IMU readings (spec RB-6). */
export function ImuCard() {
  const imu = useTopic<Imu>(TOPICS.imu).message;
  const cal = parseCalibration(useTopic<StringMsg>(TOPICS.imuCalibration).message?.data);
  const temp = useTopic<Temperature>(TOPICS.imuTemperature).message;
  const e = imu ? quaternionToEuler(imu.orientation) : null;
  const heading = e ? compassHeading(e.yawDeg) : null;
  return (
    <Card title="IMU">
      <Readings>
        <Reading label="Heading" value={heading === null ? null : `${Math.round(heading) % 360}° ${cardinal(heading)}`} />
        <Reading label="Roll" value={e ? `${fixed(e.rollDeg, 1)}°` : null} />
        <Reading label="Pitch" value={e ? `${fixed(e.pitchDeg, 1)}°` : null} />
        <Reading label="Turn rate" value={imu ? `${fixed((imu.angular_velocity.z * 180) / Math.PI, 1)} °/s` : null} />
        <Reading
          label="Calibration"
          value={cal ? `System ${CAL_LABEL[cal.sys]} · Gyro ${CAL_LABEL[cal.gyro]} · Accel ${CAL_LABEL[cal.accel]} · Mag ${CAL_LABEL[cal.mag]}` : null}
          tone={cal ? (cal.sys === 3 ? 'good' : 'warn') : undefined}
        />
        <Reading label="Temperature" value={temp ? `${temp.temperature.toFixed(0)} °C` : null} />
      </Readings>
    </Card>
  );
}

/** Wheel speeds (spec RB-7). */
export function WheelsCard() {
  const js = useTopic<JointState>(TOPICS.jointStates).message;
  const speeds = sideSpeeds(js, ROBOT_CONFIG);
  return (
    <Card title="Wheels">
      <Readings>
        <Reading label="Left side speed" value={speeds ? formatSpeed(speeds.left) : null} />
        <Reading label="Right side speed" value={speeds ? formatSpeed(speeds.right) : null} />
        {/* The motor node doesn't report current yet (TODO_ROBOT.md). */}
        <Reading label="Motor current" value={null} />
      </Readings>
    </Card>
  );
}
