import { createContext, useContext, useEffect, useState, useSyncExternalStore } from 'react';
import type { RobotConnection, LinkState } from './connection';

export const RobotContext = createContext<RobotConnection | null>(null);

export function useRobot(): RobotConnection {
  const robot = useContext(RobotContext);
  if (!robot) throw new Error('useRobot must be used inside <RobotContext.Provider>.');
  return robot;
}

export function useLinkState(): LinkState {
  const robot = useRobot();
  return useSyncExternalStore(robot.subscribe, robot.getState);
}

export interface TopicValue<T> {
  message: T | null;
  /** When the latest message arrived (ms since epoch), or null if none has. */
  receivedAt: number | null;
}

/** Latest message on a topic. Subscribes while the component is mounted. */
export function useTopic<T>(topic: { name: string; type: string; throttleMs?: number }): TopicValue<T> {
  const robot = useRobot();
  const [value, setValue] = useState<TopicValue<T>>({ message: null, receivedAt: null });
  const { name, type, throttleMs } = topic;
  useEffect(() => {
    const t = robot.topic<T>(name, type, throttleMs);
    const onMessage = (message: T) => setValue({ message, receivedAt: Date.now() });
    t.subscribe(onMessage);
    return () => t.unsubscribe(onMessage);
  }, [robot, name, type, throttleMs]);
  return value;
}

/** Like useTopic, plus the measured message rate (Hz) over the last few seconds. */
export function useTopicWithRate<T>(topic: { name: string; type: string; throttleMs?: number }): TopicValue<T> & { rateHz: number | null } {
  const robot = useRobot();
  const [value, setValue] = useState<TopicValue<T> & { rateHz: number | null }>({ message: null, receivedAt: null, rateHz: null });
  const { name, type, throttleMs } = topic;
  useEffect(() => {
    const t = robot.topic<T>(name, type, throttleMs);
    const arrivals: number[] = [];
    const onMessage = (message: T) => {
      const now = Date.now();
      arrivals.push(now);
      while (arrivals.length > 0 && now - arrivals[0] > 5000) arrivals.shift();
      const span = arrivals.length > 1 ? (arrivals[arrivals.length - 1] - arrivals[0]) / 1000 : 0;
      setValue({ message, receivedAt: now, rateHz: span > 0 ? (arrivals.length - 1) / span : null });
    };
    t.subscribe(onMessage);
    return () => t.unsubscribe(onMessage);
  }, [robot, name, type, throttleMs]);
  return value;
}
