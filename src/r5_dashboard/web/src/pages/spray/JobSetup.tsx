import { useEffect, useMemo, useRef, useState } from 'react';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { draftFromMsg, formatArea } from '../../fields/geo';
import { useFields } from '../../fields/useFields';
import { FieldsLayer, FitTo, HomeMarker, RobotMarker } from '../../map/layers';
import { MapView } from '../../map/MapView';
import { PlanLayer } from '../../map/PlanLayer';
import { useLinkState, useRobot } from '../../ros/RobotContext';
import {
  CHECK_LEVEL,
  SERVICES,
  type Check,
  type JobDefaultsResponse,
  type PlanJobResponse,
  type StartJobResponse,
} from '../../ros/topics';
import { checkForm, formatDuration, formFromSettings, settingsFromForm, type Limits, type SettingsForm } from '../../spray/settings';

const PLAN_DELAY_MS = 600;

function fieldFromHash(): string {
  const query = window.location.hash.split('?')[1] ?? '';
  return new URLSearchParams(query).get('field') ?? '';
}

/** Area Spray setup: settings, plan preview, summary, checks and start (spec AS-1 to AS-5). */
export function JobSetup() {
  const robot = useRobot();
  const link = useLinkState();
  const { fields, home } = useFields();
  const [fieldId, setFieldId] = useState(fieldFromHash);
  const [form, setForm] = useState<SettingsForm | null>(null);
  const [limits, setLimits] = useState<Limits | null>(null);
  const [planResult, setPlanResult] = useState<PlanJobResponse | null>(null);
  const [planning, setPlanning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [confirming, setConfirming] = useState(false);
  const [tankFull, setTankFull] = useState(false);
  const [starting, setStarting] = useState(false);
  const requestNo = useRef(0);

  const field = fields?.find((f) => f.id === fieldId) ?? null;
  const draft = useMemo(() => (field ? draftFromMsg(field) : null), [field]);
  const formCheck = form ? checkForm(form, limits) : null;
  const formOk = formCheck !== null && Object.keys(formCheck.errors).length === 0;

  // Starting values: the field's last-used settings, or the robot's defaults (spec AS-2).
  useEffect(() => {
    setPlanResult(null);
    if (!fieldId || link.status !== 'connected') return;
    let cancelled = false;
    robot
      .callService<{ field_id: string }, JobDefaultsResponse>(SERVICES.jobDefaults, { field_id: fieldId })
      .then((d) => {
        if (cancelled) return;
        setForm(formFromSettings(d.settings));
        setLimits({ maxSpeedKmh: d.max_speed_kmh, pumpMaxLMin: d.pump_max_flow_l_min });
      })
      .catch((err: Error) => !cancelled && setError(`Couldn't load settings: ${err.message}`));
    return () => {
      cancelled = true;
    };
  }, [robot, fieldId, link.status]);

  // Plan preview whenever the field or settings change (spec AS-3).
  useEffect(() => {
    if (!fieldId || !form || !formOk || link.status !== 'connected') return;
    const no = ++requestNo.current;
    const timer = setTimeout(() => {
      setPlanning(true);
      robot
        .callService<{ settings: unknown }, PlanJobResponse>(SERVICES.planJob, { settings: settingsFromForm(fieldId, form) }, 15000)
        .then((r) => {
          if (no !== requestNo.current) return;
          setPlanResult(r);
          setError(null);
        })
        .catch((err: Error) => no === requestNo.current && setError(`Couldn't plan: ${err.message}`))
        .finally(() => no === requestNo.current && setPlanning(false));
    }, PLAN_DELAY_MS);
    return () => clearTimeout(timer);
  }, [robot, fieldId, form, formOk, link.status]);

  async function start() {
    if (!form) return;
    setStarting(true);
    setError(null);
    try {
      const r = await robot.callService<{ settings: unknown; tank_full_confirmed: boolean }, StartJobResponse>(
        SERVICES.startJob,
        { settings: settingsFromForm(fieldId, form), tank_full_confirmed: tankFull },
        15000,
      );
      if (!r.success) {
        setError(r.message);
        if (r.checks.length && planResult) setPlanResult({ ...planResult, checks: r.checks, ready: false });
      }
      setConfirming(false);
    } catch (err) {
      setError(`Couldn't start: ${(err as Error).message}`);
    } finally {
      setStarting(false);
    }
  }

  const plan = planResult?.success ? planResult.plan : null;
  const checks: Check[] = [
    { level: link.status === 'connected' ? CHECK_LEVEL.OK : CHECK_LEVEL.FAIL, label: 'Link to robot', detail: link.status === 'connected' ? 'Connected' : 'Not connected' },
    ...(planResult?.checks ?? []),
  ];
  const ready = !!plan && !!planResult?.ready && formOk && link.status === 'connected' && !planning;
  const set = (patch: Partial<SettingsForm>) => form && setForm({ ...form, ...patch });
  const err = formCheck?.errors ?? {};
  const rateNum = form ? Number(form.rate) : NaN;

  return (
    <div className="fields-layout">
      <div className="fields-side">
        <Card title="Area spray">
          <form className="form" onSubmit={(e) => e.preventDefault()}>
            <label>
              Field
              <select value={fieldId} onChange={(e) => setFieldId(e.target.value)}>
                <option value="">Choose a field…</option>
                {(fields ?? []).map((f) => (
                  <option key={f.id} value={f.id}>
                    {f.name} ({formatArea(f.area_m2)})
                  </option>
                ))}
              </select>
            </label>
            {fields && fields.length === 0 && <p className="muted small">No fields yet. Add one on the Manage Fields page.</p>}
            {form && field && (
              <>
                <NumberInput label="Application rate (L/m²)" value={form.rate} step="0.001" error={err.rate} onChange={(v) => set({ rate: v })}
                  hint={rateNum > 0 ? `= ${Math.round(rateNum * 10_000).toLocaleString()} L/ha` : undefined} />
                <NumberInput label="Boom width (m)" value={form.boom} step="0.1" error={err.boom} onChange={(v) => set({ boom: v })} hint="Also the spacing between rows." />
                <NumberInput label="Travel speed (km/h)" value={form.speed} step="0.05" error={err.speed} onChange={(v) => set({ speed: v })}
                  hint={limits ? `Top speed ${limits.maxSpeedKmh.toFixed(2)} km/h` : undefined} />
                {formCheck?.suggestedSpeed !== null && formCheck?.suggestedSpeed !== undefined && (
                  <button type="button" className="secondary small" onClick={() => set({ speed: String(formCheck.suggestedSpeed) })}>
                    Use {formCheck.suggestedSpeed.toFixed(2)} km/h
                  </button>
                )}
                <NumberInput label="Tank capacity (L)" value={form.tank} step="1" error={err.tank} onChange={(v) => set({ tank: v })} />
                <div>
                  <span className="label-text">Row direction</span>
                  <label className="inline">
                    <input type="checkbox" checked={form.autoAngle} onChange={(e) => set({ autoAngle: e.target.checked })} /> Auto (fewest turns)
                  </label>
                  {!form.autoAngle && (
                    <NumberInput label="Degrees clockwise from north" value={form.angle} step="1" error={err.angle} onChange={(v) => set({ angle: v })} />
                  )}
                </div>
                <div className="reading-row">
                  <span>Flow rate</span>
                  <strong>{formCheck?.flow !== null && formCheck?.flow !== undefined ? `${formCheck.flow.toFixed(2)} L/min` : '—'}</strong>
                  {limits && <span className="muted small">pump max {limits.pumpMaxLMin.toFixed(1)} L/min</span>}
                </div>
              </>
            )}
          </form>
        </Card>

        {field && (
          <Card title="Job summary">
            {planning && <p className="muted">Planning…</p>}
            {planResult && !planResult.success && <p className="error">{planResult.message}</p>}
            {plan && (
              <>
                <dl className="readings">
                  <Row label="Area sprayed" value={formatArea(plan.area_sprayed_m2)} />
                  <Row label="Rows" value={`${plan.swath_count}, ${Math.round(plan.row_angle_deg)}° from north`} />
                  <Row label="Path length" value={`${(plan.path_length_m / 1000).toFixed(2)} km`} />
                  <Row label="Estimated time" value={formatDuration(plan.duration_s)} />
                  <Row label="Liquid needed" value={`${plan.liquid_needed_l.toFixed(1)} L`} />
                  <Row label="Refills" value={plan.refills === 0 ? 'None' : `${plan.refills} (the robot returns home each time; included in the time)`} />
                </dl>
                <p className="muted small">The headland isn't sprayed.</p>
                {plan.warnings.length > 0 && (
                  <ul className="problems warnings">
                    {plan.warnings.map((w) => (
                      <li key={w}>{w}</li>
                    ))}
                  </ul>
                )}
              </>
            )}
          </Card>
        )}

        {field && planResult && (
          <Card title="Before starting">
            <ChecksList checks={checks} />
            {error && <p className="error">{error}</p>}
            <button type="button" className="primary" disabled={!ready} onClick={() => { setTankFull(false); setConfirming(true); }}>
              Start job
            </button>
          </Card>
        )}
        {error && !planResult && <p className="error">{error}</p>}
      </div>

      <MapView className="fields-map">
        {draft && <FitTo points={draft.boundary} fitKey={draft.id} />}
        {draft && <FieldsLayer fields={[draft]} selectedId={draft.id} />}
        {plan && <PlanLayer sections={plan.sections} />}
        <HomeMarker position={home ?? null} />
        <RobotMarker />
      </MapView>

      <ConfirmDialog
        open={confirming}
        title={`Start spraying ${field?.name ?? ''}?`}
        confirmLabel={starting ? 'Starting…' : 'Start job'}
        danger
        onConfirm={() => tankFull && !starting && void start()}
        onCancel={() => setConfirming(false)}
      >
        {plan && form && (
          <>
            <p>
              The robot will drive to the start of the field and spray {formatArea(plan.area_sprayed_m2)} at {Number(form.rate).toFixed(3)} L/m²,
              using about {plan.liquid_needed_l.toFixed(1)} L over {formatDuration(plan.duration_s)}
              {plan.refills > 0 && `, coming home ${plan.refills} time${plan.refills > 1 ? 's' : ''} to refill`}.
            </p>
            <p><strong>Make sure the area is clear of people and animals.</strong></p>
            <label className="inline confirm-check">
              <input type="checkbox" checked={tankFull} onChange={(e) => setTankFull(e.target.checked)} /> The tank is full ({Number(form.tank)} L)
            </label>
            {!tankFull && <p className="muted small">Tick to confirm. There is no tank sensor yet, so the robot counts from full.</p>}
          </>
        )}
      </ConfirmDialog>
    </div>
  );
}

function NumberInput({ label, value, step, error, hint, onChange }: { label: string; value: string; step: string; error?: string; hint?: string; onChange: (v: string) => void }) {
  return (
    <label>
      {label}
      <input type="number" inputMode="decimal" min={0} step={step} value={value} onChange={(e) => onChange(e.target.value)} aria-invalid={!!error} />
      {error ? <span className="error small">{error}</span> : hint && <span className="muted small">{hint}</span>}
    </label>
  );
}

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="reading">
      <dt>{label}</dt>
      <dd>{value}</dd>
    </div>
  );
}

export function ChecksList({ checks }: { checks: Check[] }) {
  const ICON = ['✓', '!', '✕'];
  const CLS = ['check-ok', 'check-warn', 'check-fail'];
  return (
    <ul className="checks">
      {checks.map((c) => (
        <li key={c.label} className={CLS[c.level]}>
          <span aria-hidden="true">{ICON[c.level]}</span> <strong>{c.label}:</strong> {c.detail}
        </li>
      ))}
    </ul>
  );
}
