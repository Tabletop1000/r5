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
