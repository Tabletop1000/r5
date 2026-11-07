import { createContext, useContext, useMemo, type ReactNode } from 'react';
import { useSettings } from '../settings/useSettings';
import { resolveTimeZone } from './time';
import { unitsFrom, type Units } from './units';

export interface Display {
  units: Units;
  /** undefined: the device's own time zone. */
  timeZone: string | undefined;
}

const DisplayContext = createContext<Display>({ units: 'metric', timeZone: undefined });

/** Units and time zone from the robot's settings, for everything below it. */
export function DisplayProvider({ children }: { children: ReactNode }) {
  const values = useSettings()?.values;
  const units = unitsFrom(values?.units);
  const timeZone = resolveTimeZone(values?.time_zone);
  const display = useMemo(() => ({ units, timeZone }), [units, timeZone]);
  return <DisplayContext.Provider value={display}>{children}</DisplayContext.Provider>;
}

export function useDisplay(): Display {
  return useContext(DisplayContext);
}
