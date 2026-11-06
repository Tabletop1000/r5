import { useMemo } from 'react';
import { useTopic } from '../ros/RobotContext';
import { TOPICS, type RobotSettingsMsg } from '../ros/topics';
import { DEFAULT_ROBOT_NAME, parseSettings, type RobotSettings } from './settings';

/** The robot's settings, or null until they arrive. Updates when any dashboard changes them. */
export function useSettings(): RobotSettings | null {
  const msg = useTopic<RobotSettingsMsg>(TOPICS.settings).message;
  return useMemo(() => parseSettings(msg), [msg]);
}

export function useRobotName(): string {
  const name = useSettings()?.values.robot_name;
  return typeof name === 'string' && name ? name : DEFAULT_ROBOT_NAME;
}
