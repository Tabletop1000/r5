import { useCallback, useMemo, useRef, useState } from 'react';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { checkField, formatArea, MAX_HEADLAND_M, msgFromDraft, draftFromMsg, type FieldDraft } from '../../fields/geo';
import { DrawLayer, type DrawApi, type DrawShapes, type Role } from '../../map/DrawLayer';
import { FieldsLayer, FitTo, HomeMarker, RobotMarker, TrackLayer } from '../../map/layers';
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
  onSaved: (field: FieldDraft) => void;
  onCancel: () => void;
}

/** Draw or edit a field: boundary, keep-out zones, headland width and name (spec FM-3, FM-5, FM-6). */
export function FieldEditor({ initial, otherNames, startAt, home, fromRecording, onSaved, onCancel }: Props) {
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
  const recording = useTopic<RecordingStatus>(TOPICS.recordingStatus).message;

  const initialShapes = useMemo<DrawShapes>(() => ({ boundary: initial.boundary, keepOut: initial.keepOut }), [initial]);
  const onShapes = useCallback((s: DrawShapes) => setDraft((d) => ({ ...d, boundary: s.boundary, keepOut: s.keepOut })), []);
  const check = checkField(draft, otherNames);
  const isNew = !draft.id;
  const changed = JSON.stringify(draft) !== JSON.stringify(initial);

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

  const recordingActive = recording?.state === RECORDING_STATE.RECORDING;
  const recordingDone = recording?.state === RECORDING_STATE.FINISHED;

  return (
    <div className="fields-layout">
      <div className="fields-side">
        <section className="card">
          <div className="card-head">
            <h2>{isNew ? 'New field' : `Edit ${initial.name}`}</h2>
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
              Headland width (m)
              <input
                type="number"
                min={0}
                max={MAX_HEADLAND_M}
                step={0.1}
                value={Number.isFinite(draft.headlandWidthM) ? draft.headlandWidthM : ''}
                onChange={(e) => setDraft({ ...draft, headlandWidthM: e.target.value === '' ? NaN : Number(e.target.value) })}
              />
              <span className="muted small">The strip inside the boundary where the robot turns. Shown in amber; it isn't sprayed.</span>
            </label>
            <label>
              Notes <span className="muted small">(optional)</span>
              <input value={draft.notes} onChange={(e) => setDraft({ ...draft, notes: e.target.value })} />
            </label>

            <div className="draw-tools">
              <strong>Shape</strong>
              {drawing ? (
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
              <span>Area: {check.areaM2 ? formatArea(check.areaM2) : '—'}</span>
              <span>Keep-out zones: {draft.keepOut.length}</span>
            </div>
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
              <button type="submit" className="primary" disabled={check.errors.length > 0 || saving || drawing !== null}>
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
