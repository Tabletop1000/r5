import { describe, expect, it } from 'vitest';
import { changedSettings, checkSetting, linkLossText, formFromValues, groupSettings, isShown, parseSettings, type SettingDescription } from './settings';

const schema: SettingDescription[] = [
  { key: 'robot_name', label: 'Nickname', group: 'Robot', kind: 'text', default: 'R5', help: '', unit: '', max_length: 20 },
  { key: 'max_speed_kmh', label: 'Maximum speed', group: 'Driving', kind: 'number', default: 1.08, help: '', unit: 'km/h', minimum: 0.1, maximum: 1.08 },
  {
    key: 'comms_loss_action',
    label: 'If the link is lost',
    group: 'Link loss',
    kind: 'choice',
    default: 'continue',
    help: '',
    unit: '',
    choices: [
      { value: 'continue', label: 'Keep working' },
      { value: 'delayed_return_home', label: 'Keep working, then return home' },
    ],
  },
  {
    key: 'comms_loss_delay_min',
    label: 'Return home after',
    group: 'Link loss',
    kind: 'number',
    default: 5,
    help: '',
    unit: 'min',
    minimum: 1,
    shown_when: { key: 'comms_loss_action', value: 'delayed_return_home' },
  },
];
const values = { robot_name: 'R5', max_speed_kmh: 1.08, comms_loss_action: 'continue', comms_loss_delay_min: 5 };

describe('parseSettings', () => {
  it('reads the message', () => {
    const parsed = parseSettings({ schema_json: JSON.stringify(schema), values_json: JSON.stringify(values) });
    expect(parsed?.values.robot_name).toBe('R5');
    expect(parsed?.schema).toHaveLength(4);
  });
  it('returns null for missing or damaged messages', () => {
    expect(parseSettings(null)).toBeNull();
    expect(parseSettings({ schema_json: '{', values_json: '{}' })).toBeNull();
  });
});

describe('the settings form', () => {
  it('groups settings in order', () => {
    expect(groupSettings(schema).map((g) => [g.group, g.settings.length])).toEqual([
      ['Robot', 1],
      ['Driving', 1],
      ['Link loss', 2],
    ]);
  });

  it('shows a dependent setting only for its choice', () => {
    const form = formFromValues(schema, values);
    expect(isShown(schema[3], form)).toBe(false);
    expect(isShown(schema[3], { ...form, comms_loss_action: 'delayed_return_home' })).toBe(true);
  });

  it('sends only what changed', () => {
    const form = { ...formFromValues(schema, values), robot_name: '  Big   Red ', max_speed_kmh: '1.08' };
    expect(changedSettings(schema, values, form)).toEqual({ changes: { robot_name: 'Big Red' }, errors: {} });
  });

  it('reports bad values and ignores hidden ones', () => {
    const form = { ...formFromValues(schema, values), max_speed_kmh: '2', comms_loss_delay_min: '0' };
    const { changes, errors } = changedSettings(schema, values, form);
    expect(changes).toEqual({});
    expect(Object.keys(errors)).toEqual(['max_speed_kmh']);
  });

  it('checks each kind', () => {
    expect(checkSetting(schema[1], '')).toEqual({ error: 'Maximum speed must be a number.' });
    expect(checkSetting(schema[1], '0.05')).toEqual({ error: 'Maximum speed must be at least 0.1 km/h.' });
    expect(checkSetting(schema[1], '0.5')).toEqual({ value: 0.5 });
    expect(checkSetting(schema[0], ' ')).toEqual({ error: "Nickname can't be empty." });
    expect(checkSetting(schema[0], 'x'.repeat(21))).toEqual({ error: 'Nickname can be at most 20 characters.' });
    expect(checkSetting(schema[2], 'panic')).toEqual({ error: 'Choose a value for If the link is lost.' });
  });
});

describe('linkLossText', () => {
  it('describes each action', () => {
    expect(linkLossText(undefined)).toBe('robot continues on its own');
    expect(linkLossText({ comms_loss_action: 'stop', comms_loss_timeout_s: 30 })).toBe('a running job stops after 30 s');
    expect(linkLossText({ comms_loss_action: 'delayed_return_home', comms_loss_delay_min: 5 })).toBe(
      'a running job carries on for 5 min, then returns home',
    );
  });
});
