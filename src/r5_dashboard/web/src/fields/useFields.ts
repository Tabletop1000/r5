import { useCallback, useEffect, useState } from 'react';
import { useLinkState, useRobot, useTopic } from '../ros/RobotContext';
import { SERVICES, TOPICS, type FieldMsg, type GetHomeResponse, type LatLon, type UInt32 } from '../ros/topics';

export interface FieldsState {
  fields: FieldMsg[] | null;
  /** undefined while loading, null when not set. */
  home: LatLon | null | undefined;
  error: string | null;
  reload: () => Promise<void>;
}

/** Fields and home point from the robot. Reloads when any dashboard changes them. */
export function useFields(): FieldsState {
  const robot = useRobot();
  const link = useLinkState();
  const revision = useTopic<UInt32>(TOPICS.fieldsRevision).message?.data;
  const [fields, setFields] = useState<FieldMsg[] | null>(null);
  const [home, setHome] = useState<LatLon | null | undefined>(undefined);
  const [error, setError] = useState<string | null>(null);

  const reload = useCallback(async () => {
    try {
      const [list, h] = await Promise.all([
        robot.callService<object, { fields: FieldMsg[] }>(SERVICES.listFields, {}),
        robot.callService<object, GetHomeResponse>(SERVICES.getHome, {}),
      ]);
      setFields(list.fields);
      setHome(h.is_set ? h.position : null);
      setError(null);
    } catch (err) {
      setError(`Couldn't load fields from the robot: ${(err as Error).message}`);
    }
  }, [robot]);

  useEffect(() => {
    if (link.status === 'connected') void reload();
  }, [reload, link.status, revision]);

  return { fields, home, error, reload };
}
