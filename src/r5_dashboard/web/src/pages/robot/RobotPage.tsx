import { AlertsCard } from './AlertsCard';
import { GoToCard } from './GoToCard';
import { MotorTestCard } from './MotorTestCard';
import { OverviewCard } from './OverviewCard';
import { PasswordCard } from './PasswordCard';
import { GnssCard, ImuCard, WheelsCard } from './SensorsCards';
import { SensorTestsCard } from './SensorTestsCard';
import { UsageCard } from './UsageCard';

/** Robot status, sensors, tests and settings (spec 4.4). */
export function RobotPage() {
  return (
    <section className="page wide">
      <h1>Robot</h1>
      <div className="card-grid">
        <OverviewCard />
        <AlertsCard />
        <GoToCard />
        <UsageCard />
        <GnssCard />
        <ImuCard />
        <WheelsCard />
        <SensorTestsCard />
        <MotorTestCard />
        <PasswordCard />
      </div>
    </section>
  );
}
