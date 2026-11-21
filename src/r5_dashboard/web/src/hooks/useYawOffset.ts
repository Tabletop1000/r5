import { useEffect, useState } from 'react';
import { useLinkState, useRobot } from '../ros/RobotContext';
import { SERVICES } from '../ros/topics';

interface GetParametersResponse {
  values: { type: number; double_value: number }[];
}

const PARAMETER_DOUBLE = 3;
const RETRY_MS = 5000;

/**
 * Degrees to add to the BNO055's yaw to get true ENU yaw: navsat_transform's yaw_offset plus
 * magnetic_declination_radians, the same correction it uses to place GPS in the map. Read from the
 * node itself so ekf.yaml stays the only place it's set. 0 until navsat_transform answers.
 */
export function useYawOffsetDeg(): number {
  const robot = useRobot();
  const link = useLinkState();
  const [offsetDeg, setOffsetDeg] = useState(0);
  useEffect(() => {
    if (link.status !== 'connected') return;
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const load = async () => {
      try {
        const r = await robot.callService<{ names: string[] }, GetParametersResponse>(
          SERVICES.navsatParameters,
          { names: ['yaw_offset', 'magnetic_declination_radians'] },
          3000,
        );
        const rad = r.values.reduce((sum, v) => sum + (v.type === PARAMETER_DOUBLE ? v.double_value : 0), 0);
        if (!cancelled) setOffsetDeg((rad * 180) / Math.PI);
      } catch {
        // navsat_transform may start after the dashboard. Keep asking.
        if (!cancelled) timer = setTimeout(() => void load(), RETRY_MS);
      }
    };
    void load();
    return () => {
      cancelled = true;
      clearTimeout(timer);
    };
  }, [robot, link.status]);
  return offsetDeg;
}
