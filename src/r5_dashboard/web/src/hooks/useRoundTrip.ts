import { useEffect, useState } from 'react';
import { useLinkState, useRobot } from '../ros/RobotContext';
import { SERVICES } from '../ros/topics';

/** Round-trip time to the robot in ms, measured every 2 s with a tiny service call. */
export function useRoundTrip(): number | null {
  const robot = useRobot();
  const link = useLinkState();
  const [rtt, setRtt] = useState<number | null>(null);
  useEffect(() => {
    if (link.status !== 'connected') {
      setRtt(null);
      return;
    }
    let cancelled = false;
    const ping = async () => {
      const start = performance.now();
      try {
        await robot.callService(SERVICES.ping, {}, 3000);
        if (!cancelled) setRtt(Math.round(performance.now() - start));
      } catch {
        if (!cancelled) setRtt(null);
      }
    };
    void ping();
    const id = setInterval(() => void ping(), 2000);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [robot, link.status]);
  return rtt;
}
