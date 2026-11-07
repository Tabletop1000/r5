// Robot settings. The robot describes each setting (r5_dashboard_server/settings.py) and the
// Settings page builds its form from the descriptions, so a new setting needs no change here.

import { inputNumber, speed, type Units } from '../display/units';
import type { RobotSettingsMsg } from '../ros/topics';

export const DEFAULT_ROBOT_NAME = 'R5';

export interface SettingDescription {
  key: string;
  label: string;
  group: string;
  kind: 'number' | 'text' | 'choice';
  default: number | string;
  help: string;
  unit: string;
  minimum?: number;
  maximum?: number;
  step?: number;
  max_length?: number;
  choices?: { value: string; label: string }[];
  /** Only shown while another setting has this value. */
  shown_when?: { key: string; value: string };
  /** "speed": stored in km/h, shown in the chosen units. */
  quantity?: string;
}

/** The unit to show a setting in. */
export function displayUnit(s: SettingDescription, u: Units): string {
  return s.quantity === 'speed' ? speed.unit(u) : s.unit;
}

function toDisplay(s: SettingDescription, value: number, u: Units): number {
  return s.quantity === 'speed' ? speed.fromKmh(value, u) : value;
}

function fromDisplay(s: SettingDescription, value: number, u: Units): number {
  return s.quantity === 'speed' ? speed.toKmh(value, u) : value;
}

export type SettingValues = Record<string, number | string>;

export interface RobotSettings {
  schema: SettingDescription[];
  values: SettingValues;
}

export function parseSettings(msg: RobotSettingsMsg | null): RobotSettings | null {
  if (!msg) return null;
  try {
    const schema = JSON.parse(msg.schema_json) as SettingDescription[];
    const values = JSON.parse(msg.values_json) as SettingValues;
    if (!Array.isArray(schema) || typeof values !== 'object' || values === null) return null;
    return { schema, values };
  } catch {
    return null;
  }
}

/** Settings grouped for display, in the robot's order. */
export function groupSettings(schema: SettingDescription[]): { group: string; settings: SettingDescription[] }[] {
  const groups: { group: string; settings: SettingDescription[] }[] = [];
  for (const s of schema) {
    let g = groups.find((x) => x.group === s.group);
    if (!g) groups.push((g = { group: s.group, settings: [] }));
    g.settings.push(s);
  }
  return groups;
}

/** What the status strip says the robot does while the dashboard is disconnected. */
export function linkLossText(values: SettingValues | undefined): string {
  const timeout = Number(values?.comms_loss_timeout_s);
  const after = Number.isFinite(timeout) ? ` after ${timeout} s` : '';
  switch (values?.comms_loss_action) {
    case 'return_home':
      return `a running job returns home${after}`;
    case 'delayed_return_home':
      return `a running job carries on for ${values.comms_loss_delay_min} min, then returns home`;
    case 'stop':
      return `a running job stops${after}`;
    default:
      return 'robot continues on its own';
  }
}

/** Form text, as the inputs hold it. */
export type SettingsForm = Record<string, string>;

export function formFromValues(schema: SettingDescription[], values: SettingValues, u: Units = 'metric'): SettingsForm {
  return Object.fromEntries(
    schema.map((s) => {
      const v = values[s.key] ?? s.default;
      return [s.key, s.quantity && typeof v === 'number' ? inputNumber(toDisplay(s, v, u), 2) : String(v)];
    }),
  );
}

export function isShown(s: SettingDescription, form: SettingsForm): boolean {
  return !s.shown_when || form[s.shown_when.key] === s.shown_when.value;
}

/** The value to send, or a message for the user. The robot checks again when saving. */
export function checkSetting(s: SettingDescription, text: string, u: Units = 'metric'): { value: number | string } | { error: string } {
  if (s.kind === 'number') {
    const typed = text.trim() === '' ? NaN : Number(text);
    const n = fromDisplay(s, typed, u);
    const unitText = displayUnit(s, u);
    const unit = unitText ? ` ${unitText}` : '';
    const shown = (limit: number, round: (x: number) => number) => round(toDisplay(s, limit, u) * 100) / 100;
    if (!Number.isFinite(n)) return { error: `${s.label} must be a number.` };
    if (s.minimum !== undefined && n < s.minimum - 1e-6) return { error: `${s.label} must be at least ${shown(s.minimum, Math.ceil)}${unit}.` };
    if (s.maximum !== undefined && n > s.maximum + 1e-6) return { error: `${s.label} can be at most ${shown(s.maximum, Math.floor)}${unit}.` };
    return { value: n };
  }
  if (s.kind === 'text') {
    const t = text.split(/\s+/).filter(Boolean).join(' ');
    if (!t) return { error: `${s.label} can't be empty.` };
    if (s.max_length !== undefined && t.length > s.max_length) return { error: `${s.label} can be at most ${s.max_length} characters.` };
    return { value: t };
  }
  if (!s.choices?.some((c) => c.value === text)) return { error: `Choose a value for ${s.label}.` };
  return { value: text };
}

/** The settings the form changes, and any errors by key. Hidden settings are left as they are. */
export function changedSettings(
  schema: SettingDescription[],
  values: SettingValues,
  form: SettingsForm,
  u: Units = 'metric',
): { changes: SettingValues; errors: Record<string, string> } {
  const changes: SettingValues = {};
  const errors: Record<string, string> = {};
  // Compared as shown, so a value rounded for display (1.08 km/h is 0.67 mph) isn't a change.
  const shown = formFromValues(schema, values, u);
  for (const s of schema) {
    if (!isShown(s, form)) continue;
    const text = form[s.key] ?? '';
    const checked = checkSetting(s, text, u);
    if ('error' in checked) errors[s.key] = checked.error;
    else if (text !== shown[s.key] && checked.value !== values[s.key]) changes[s.key] = checked.value;
  }
  return { changes, errors };
}
