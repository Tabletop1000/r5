import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { useDisplay } from '../../display/DisplayContext';
import { formatArea, formatLength, inputNumber, length } from '../../display/units';
import { checkField, LocalProjection, MAX_HEADLAND_M, msgFromDraft, draftFromMsg, type FieldDraft } from '../../fields/geo';
import { DrawLayer, type DrawApi, type DrawShapes, type Role } from '../../map/DrawLayer';
import { GatePicker } from '../../map/GatePicker';
import { CrossingsLayer, FieldsLayer, FitTo, HomeMarker, RobotMarker, TrackLayer } from '../../map/layers';
import { MapView } from '../../map/MapView';
import { useRobot, useTopic } from '../../ros/RobotContext';
import { RECORDING_STATE, SERVICES, TOPICS, type LatLon, type RecordingStatus, type SaveFieldResponse } from '../../ros/topics';
import { RecordingControls } from './RecordingPanel';

interface Props {
  initial: FieldDraft;
  otherNames: string[];
  /** Where to centre the map when there is no boundary yet. */
  startAt: LatLon[];
  home: LatLon | null | undefined;
  /** The draft came from a finished recording on the robot, which is cleared once saved. */
  fromRecording?: boolean;
  /** Things to point out about an imported shape. */
  notices?: string[];
  onSaved: (field: FieldDraft) => void;
  onCancel: () => void;
}

/** Width of a gate in metres. */
function gateWidthM(gate: LatLon[]): number {
  if (gate.length !== 2) return 0;
  const proj = LocalProjection.around(gate);
  const [a, b] = gate.map((p) => proj.toXY(p));
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

/**
 * Draw or edit a field: boundary, keep-out zones, gate, headland width and name (spec FM-3, FM-5,
 * FM-6). The gate is where the robot drives into and out of the field.
 */
export function FieldEditor({ initial, otherNames, startAt, home, fromRecording, notices, onSaved, onCancel }: Props) {
  const robot = useRobot();
  const drawApi = useRef<DrawApi | null>(null);
  const [draft, setDraft] = useState<FieldDraft>(initial);
  const [drawing, setDrawing] = useState<Role | null>(null);
  const [selected, setSelected] = useState<Role | null>(null);
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [confirmCancel, setConfirmCancel] = useState(false);
  const [recordingKeepOut, setRecordingKeepOut] = useState(false);
  const [loadProblem, setLoadProblem] = useState<string | null>(null);
  // Placing the gate: null when not, else the first post once it's placed.
  const [gatePicking, setGatePicking] = useState<{ first: LatLon | null } | null>(null);
  const recording = useTopic<RecordingStatus>(TOPICS.recordingStatus).message;

  const initialShapes = useMemo<DrawShapes>(() => ({ boundary: initial.boundary, keepOut: initial.keepOut }), [initial]);
  const onShapes = useCallback((s: DrawShapes) => setDraft((d) => ({ ...d, boundary: s.boundary, keepOut: s.keepOut })), []);
  const { units } = useDisplay();
  const check = checkField(draft, otherNames, units);
  const crossingShapes = useMemo(() => [draft.boundary, ...draft.keepOut], [draft.boundary, draft.keepOut]);
  // The headland input holds what was typed, in the display units; the draft keeps metres.
  const headlandText = (m: number) => (Number.isFinite(m) ? inputNumber(length.fromM(m, units), 2) : '');
  const [headland, setHeadland] = useState(() => headlandText(draft.headlandWidthM));
  const headlandUnits = useRef(units);
  if (headlandUnits.current !== units) {
    headlandUnits.current = units;
    setHeadland(headlandText(draft.headlandWidthM));
  }
  const isNew = !draft.id;
  const changed = JSON.stringify(draft) !== JSON.stringify(initial) || (isNew && initial.source === 'imported');

  async function save() {
    setSaving(true);
    setSaveError(null);
    try {
      const resp = await robot.callService<{ field: unknown }, SaveFieldResponse>(SERVICES.saveField, { field: msgFromDraft(draft) });
      if (resp.success) {
        if (fromRecording) await robot.callService(SERVICES.clearRecording, {}).catch(() => undefined);
        onSaved(draftFromMsg(resp.field));
      } else setSaveError(resp.message);
    } catch (err) {
      setSaveError(`Couldn't save: ${(err as Error).message}`);
    } finally {
      setSaving(false);
    }
  }

  function cancel() {
    if (changed) setConfirmCancel(true);
    else onCancel();
  }

  async function useRecordingAsKeepOut(points: LatLon[]) {
    drawApi.current?.addKeepOut(points);
    setRecordingKeepOut(false);
    try {
      await robot.callService(SERVICES.clearRecording, {});
    } catch {
      // The zone is in the draft already; a leftover recording is harmless.
    }
  }

  function startGate() {
    drawApi.current?.setPaused(true);
    setGatePicking({ first: null });
  }

  const stopGate = useCallback(() => {
    drawApi.current?.setPaused(false);
    setGatePicking(null);
  }, []);

  function pickGatePost(post: LatLon) {
    const first = gatePicking?.first;
    if (!first) {
      setGatePicking({ first: post });
      return;
    }
    setDraft((d) => ({ ...d, gate: [first, post] }));
    stopGate();
  }

  useEffect(() => {
    if (!gatePicking) return;
    const key = (e: KeyboardEvent) => {
      if (e.key === 'Escape') stopGate();
    };
    window.addEventListener('keydown', key);
    return () => window.removeEventListener('keydown', key);
  }, [gatePicking, stopGate]);

  const recordingActive = recording?.state === RECORDING_STATE.RECORDING;
  const recordingDone = recording?.state === RECORDING_STATE.FINISHED;

  return (
    <div className="fields-layout">
      <div className="fields-side">
        <section className="card">
          <div className="card-head">
            <h2>{isNew ? (initial.source === 'imported' ? 'Imported field' : 'New field') : `Edit ${initial.name}`}</h2>
          </div>
          <form
            className="form"
            onSubmit={(e) => {
              e.preventDefault();
              void save();
            }}
          >
            <label>
              Name
              <input value={draft.name} maxLength={60} onChange={(e) => setDraft({ ...draft, name: e.target.value })} autoFocus={!isNew} />
            </label>
            <label>
              Headland width ({length.unit(units)})
              <input
                type="number"
                min={0}
                max={Number(length.fromM(MAX_HEADLAND_M, units).toFixed(1))}
                step={0.1}
                value={headland}
                onChange={(e) => {
                  setHeadland(e.target.value);
                  setDraft({ ...draft, headlandWidthM: e.target.value === '' ? NaN : length.toM(Number(e.target.value), units) });
                }}
              />
              <span className="muted small">The strip inside the boundary where the robot turns. Shown in amber; it isn't sprayed.</span>
            </label>
            <label>
              Notes <span className="muted small">(optional)</span>
              <input value={draft.notes} onChange={(e) => setDraft({ ...draft, notes: e.target.value })} />
            </label>

            <div className="draw-tools">
              <strong>Shape</strong>
              {gatePicking ? (
                <>
                  <p className="hint">
                    Move along the boundary and click to place the {gatePicking.first ? 'second' : 'first'} gate post. The robot drives into
                    and out of the field between the two posts.
                  </p>
                  <button type="button" className="secondary small" onClick={stopGate}>
                    Cancel gate
                  </button>
                </>
              ) : drawing ? (
                <>
                  <p className="hint">
                    {drawing === 'boundary' ? 'Click on the map to place the boundary corners.' : 'Click on the map to place the keep-out zone corners.'}{' '}
                    Click the first corner again to finish.
                  </p>
                  {(drawing === 'keep_out' || draft.boundary.length >= 3) && (
                    <button type="button" className="secondary small" onClick={() => drawApi.current?.cancelDrawing()}>
                      Cancel drawing
                    </button>
                  )}
                </>
              ) : (
                <>
                  <p className="hint">Drag a corner to move it. Drag a small midpoint to add a corner. Right-click a corner to delete it. Click a shape to select it.</p>
                  <div className="button-row">
                    <button type="button" className="secondary small" disabled={draft.boundary.length < 3} onClick={() => drawApi.current?.draw('keep_out')}>
                      Draw keep-out zone
                    </button>
                    <button
                      type="button"
                      className="secondary small"
                      disabled={draft.boundary.length < 3 || recordingActive || recordingDone}
                      title={recordingDone && !recordingKeepOut ? 'Save this field first; a finished recording is waiting.' : undefined}
                      onClick={() => {
                        setRecordingKeepOut(true);
                        void robot.callService(SERVICES.startRecording, {}).catch((err: Error) => setSaveError(err.message));
                      }}
                    >
                      Record keep-out zone
                    </button>
                    <button type="button" className="secondary small" disabled={draft.boundary.length < 3} onClick={startGate}>
                      {draft.gate.length ? 'Move gate' : 'Add gate'}
                    </button>
                    {draft.gate.length > 0 && (
                      <button type="button" className="secondary small" onClick={() => setDraft((d) => ({ ...d, gate: [] }))}>
                        Remove gate
                      </button>
                    )}
                    {selected === 'keep_out' && (
                      <button type="button" className="secondary small" onClick={() => drawApi.current?.removeSelected()}>
                        Remove selected zone
                      </button>
                    )}
                    <button type="button" className="secondary small" onClick={() => drawApi.current?.draw('boundary')}>
                      Redraw boundary
                    </button>
                  </div>
                </>
              )}
            </div>

            {recording && (recordingActive || (recordingKeepOut && recordingDone)) && (
              <RecordingControls
                status={recording}
                purpose="keep-out zone"
                onUse={(points) => void useRecordingAsKeepOut(points)}
                useLabel="Add as keep-out zone"
                onDiscarded={() => setRecordingKeepOut(false)}
              />
            )}

            <div className="field-summary">
              <span>Area: {check.areaM2 ? formatArea(check.areaM2, units) : '—'}</span>
              <span>Keep-out zones: {draft.keepOut.length}</span>
              <span>Gate: {draft.gate.length === 2 ? formatLength(gateWidthM(draft.gate), units, 1) + ' wide' : 'none'}</span>
            </div>
            {notices?.map((n) => (
              <p key={n} className="warn-text small">
                {n}
              </p>
            ))}
            {loadProblem && <p className="error">{loadProblem}</p>}
            {check.errors.length > 0 && (
              <ul className="problems errors">
                {check.errors.map((e) => (
                  <li key={e}>{e}</li>
                ))}
              </ul>
            )}
            {check.warnings.length > 0 && (
              <ul className="problems warnings">
                {check.warnings.map((w) => (
                  <li key={w}>{w}</li>
                ))}
              </ul>
            )}
            {saveError && <p className="error">{saveError}</p>}
            <div className="button-row">
              <button type="submit" className="primary" disabled={check.errors.length > 0 || saving || drawing !== null || gatePicking !== null}>
                {saving ? 'Saving…' : 'Save field'}
              </button>
              <button type="button" className="secondary" onClick={cancel}>
                Cancel
              </button>
            </div>
          </form>
        </section>
      </div>

      <MapView className="fields-map">
        <FitTo points={initial.boundary.length ? initial.boundary : startAt} fitKey={`${initial.id || 'new'}:${initial.boundary.length > 0 || startAt.length > 0}`} />
        <FieldsLayer id="draft" fields={[{ ...draft, keepOut: [] }]} />
        <HomeMarker position={home ?? null} />
        <RobotMarker />
        {recording && recording.state !== RECORDING_STATE.IDLE && (
          <TrackLayer points={recording.points} closed={recording.state === RECORDING_STATE.FINISHED} />
        )}
        <DrawLayer initial={initialShapes} apiRef={drawApi} onChange={onShapes} onSelect={setSelected} onModeChange={setDrawing} onLoadProblem={setLoadProblem} />
        <CrossingsLayer shapes={crossingShapes} />
        {gatePicking && <GatePicker boundary={draft.boundary} firstPost={gatePicking.first} onPick={pickGatePost} />}
      </MapView>

      <ConfirmDialog
        open={confirmCancel}
        title="Discard changes?"
        confirmLabel="Discard"
        danger
        onConfirm={onCancel}
        onCancel={() => setConfirmCancel(false)}
      >
        <p>
          Your changes will be lost.
          {fromRecording && ' The recording stays on the robot, so you can start again from it.'}
        </p>
      </ConfirmDialog>
    </div>
  );
}
