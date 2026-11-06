import { useEffect, useRef, useState } from 'react';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { useFields } from '../../fields/useFields';
import { useLinkState, useRobot, useTopic } from '../../ros/RobotContext';
import { SERVICES, TOPICS, type LatLon, type NavSatFix, type TriggerResponse } from '../../ros/topics';
import {
  changedSettings,
  formFromValues,
  groupSettings,
  isShown,
  type RobotSettings,
  type SettingDescription,
  type SettingsForm,
} from '../../settings/settings';
import { useSettings } from '../../settings/useSettings';

/** Robot settings. The form comes from the robot's descriptions of its settings. */
export function SettingsPage() {
  const settings = useSettings();
  const link = useLinkState();
  return (
    <section className="page">
      <h1>Settings</h1>
      {settings ? (
        <SettingsForm settings={settings} />
      ) : (
        <p className="muted">{link.status === 'connected' ? 'Loading settings…' : 'Settings load once the robot is connected.'}</p>
      )}
      <GarageCard />
    </section>
  );
}

function SettingsForm({ settings }: { settings: RobotSettings }) {
  const robot = useRobot();
  const link = useLinkState();
  const { schema, values } = settings;
  const [form, setForm] = useState<SettingsForm>(() => formFromValues(schema, values));
  const [edited, setEdited] = useState(false);
  const [saving, setSaving] = useState(false);
  const [note, setNote] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Show changes made from another dashboard, unless this one has unsaved edits. Only when the
  // values change, so saving doesn't flash the old values before the robot sends the new ones.
  const editedRef = useRef(edited);
  editedRef.current = edited;
  useEffect(() => {
    if (!editedRef.current) setForm(formFromValues(schema, values));
  }, [schema, values]);

  const { changes, errors } = changedSettings(schema, values, form);
  const changeCount = Object.keys(changes).length;
  const hasErrors = Object.keys(errors).length > 0;

  function set(key: string, text: string) {
    setForm((f) => ({ ...f, [key]: text }));
    setEdited(true);
    setNote(null);
  }

  async function save() {
    setSaving(true);
    setError(null);
    try {
      const resp = await robot.callService<{ values_json: string }, TriggerResponse>(SERVICES.setSettings, {
        values_json: JSON.stringify(changes),
      });
      if (resp.success) {
        setEdited(false);
        setNote('Settings saved.');
      } else {
        setError(resp.message);
      }
    } catch (err) {
      setError((err as Error).message);
    } finally {
      setSaving(false);
    }
  }

  function discard() {
    setForm(formFromValues(schema, values));
    setEdited(false);
    setError(null);
  }

  return (
    <form
      className="settings-form"
      onSubmit={(e) => {
        e.preventDefault();
        if (changeCount && !hasErrors) void save();
      }}
    >
      {groupSettings(schema).map(({ group, settings: items }) => (
        <Card key={group} title={group}>
          <div className="form">
            {items.filter((s) => isShown(s, form)).map((s) => (
              <SettingInput key={s.key} setting={s} text={form[s.key] ?? ''} error={errors[s.key]} onChange={(t) => set(s.key, t)} />
            ))}
          </div>
        </Card>
      ))}
      <div className="button-row settings-actions">
        <button type="submit" className="primary" disabled={!changeCount || hasErrors || saving || link.status !== 'connected'}>
          {saving ? 'Saving…' : 'Save'}
        </button>
        {edited && (
          <button type="button" className="secondary" onClick={discard} disabled={saving}>
            Discard changes
          </button>
        )}
      </div>
      {note && <p className="note" role="status">{note}</p>}
      {error && <p className="error">{error}</p>}
    </form>
  );
}

function SettingInput({ setting: s, text, error, onChange }: { setting: SettingDescription; text: string; error?: string; onChange: (text: string) => void }) {
  const helpId = `setting-${s.key}-help`;
  const common = { 'aria-invalid': !!error, 'aria-describedby': helpId };
  let input;
  if (s.kind === 'choice') {
    input = (
      <select value={text} onChange={(e) => onChange(e.target.value)} {...common}>
        {s.choices?.map((c) => (
          <option key={c.value} value={c.value}>
            {c.label}
          </option>
        ))}
      </select>
    );
  } else if (s.kind === 'number') {
    input = (
      <span className="input-with-unit">
        <input type="number" inputMode="decimal" min={s.minimum} max={s.maximum} step={s.step ?? 'any'} value={text} onChange={(e) => onChange(e.target.value)} {...common} />
        {s.unit && <span className="unit">{s.unit}</span>}
      </span>
    );
  } else {
    input = <input type="text" maxLength={s.max_length} value={text} onChange={(e) => onChange(e.target.value)} {...common} />;
  }
  return (
    <label>
      {s.label}
      {input}
      <span id={helpId} className={error ? 'error small' : 'hint'}>
        {error ?? s.help}
      </span>
    </label>
  );
}

/** The home point, where the robot goes for refills and when the link is lost. */
function GarageCard() {
  const robot = useRobot();
  const link = useLinkState();
  const { home } = useFields();
  const fix = useTopic<NavSatFix>(TOPICS.fix).message;
  const position: LatLon | null = fix && fix.status.status >= 0 ? { latitude: fix.latitude, longitude: fix.longitude } : null;
  const [confirming, setConfirming] = useState(false);
  const [note, setNote] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  async function setHome() {
    setConfirming(false);
    setError(null);
    setNote(null);
    if (!position) return;
    try {
      const resp = await robot.callService<{ position: LatLon }, TriggerResponse>(SERVICES.setHome, { position });
      if (resp.success) setNote('Garage location set to where the robot is now.');
      else setError(resp.message);
    } catch (err) {
      setError((err as Error).message);
    }
  }

  return (
    <Card title="Garage location (home)">
      <p className="muted small">Where the robot goes to have its tank refilled, and when the link is lost. A spray job can't start until it's set.</p>
      <p>
        {home === undefined
          ? link.status === 'connected'
            ? 'Loading…'
            : 'Not available'
          : home === null
            ? 'Not set.'
            : `Set at ${home.latitude.toFixed(6)}, ${home.longitude.toFixed(6)}.`}
      </p>
      <div className="button-row">
        <button type="button" className="secondary" disabled={!position || link.status !== 'connected'} onClick={() => (home ? setConfirming(true) : void setHome())}>
          Set to current location
        </button>
      </div>
      {!position && link.status === 'connected' && <p className="hint">Needs a GPS position from the robot.</p>}
      {note && <p className="note" role="status">{note}</p>}
      {error && <p className="error">{error}</p>}
      <ConfirmDialog
        open={confirming}
        title="Move the garage location?"
        confirmLabel="Set to current location"
        onConfirm={() => void setHome()}
        onCancel={() => setConfirming(false)}
      >
        <p>The garage location will move to where the robot is now. Jobs drive home to the new location.</p>
      </ConfirmDialog>
    </Card>
  );
}
