import { useCallback, useEffect, useMemo, useState, useSyncExternalStore } from 'react';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { FieldThumb } from '../../fields/FieldThumb';
import { DEFAULT_HEADLAND_M, draftFromMsg, formatArea, msgFromDraft, type FieldDraft } from '../../fields/geo';
import { formatDate } from '../../fields/time';
import { useFields } from '../../fields/useFields';
import { MapClickPicker } from '../../map/HomePlacer';
import { FieldsLayer, FitTo, HomeMarker, RobotMarker, TrackLayer } from '../../map/layers';
import { MapView } from '../../map/MapView';
import { useLinkState, useRobot, useTopic } from '../../ros/RobotContext';
import {
  RECORDING_STATE,
  SERVICES,
  TOPICS,
  type LatLon,
  type NavSatFix,
  type RecordingStatus,
  type SaveFieldResponse,
  type TriggerResponse,
} from '../../ros/topics';
import { FieldEditor } from './FieldEditor';
import { RecordingControls } from './RecordingPanel';

type View =
  | { kind: 'list'; selectedId: string | null }
  | { kind: 'edit'; draft: FieldDraft; fromRecording: boolean; returnTo: string | null };

const newDraft = (source: FieldDraft['source'], boundary: LatLon[] = []): FieldDraft => ({
  id: '',
  name: '',
  boundary,
  keepOut: [],
  headlandWidthM: DEFAULT_HEADLAND_M,
  source,
  notes: '',
});

/** Manage Fields (spec 4.1): list, view, add (draw or record), edit, rename, delete, home point. */
export function FieldsPage() {
  const link = useLinkState();
  const { fields, home, error, reload } = useFields();
  const recording = useTopic<RecordingStatus>(TOPICS.recordingStatus).message;
  const fix = useTopic<NavSatFix>(TOPICS.fix).message;
  const [view, setView] = useState<View>({ kind: 'list', selectedId: null });
  const [message, setMessage] = useState<string | null>(null);

  const drafts = useMemo(() => (fields ?? []).map(draftFromMsg), [fields]);
  const robotPosition: LatLon | null = fix && fix.status.status >= 0 ? { latitude: fix.latitude, longitude: fix.longitude } : null;
  const allPoints = useMemo(() => drafts.flatMap((d) => d.boundary), [drafts]);
  const startAt = robotPosition ? [robotPosition] : home ? [home] : allPoints;

  const openEditor = useCallback(
    (draft: FieldDraft, fromRecording = false) =>
      setView((v) => ({ kind: 'edit', draft, fromRecording, returnTo: v.kind === 'list' ? v.selectedId : null })),
    [],
  );

  if (view.kind === 'edit') {
    return (
      <section className="page wide">
        <h1>Manage Fields</h1>
        <FieldEditor
          initial={view.draft}
          otherNames={drafts.filter((d) => d.id !== view.draft.id).map((d) => d.name)}
          startAt={startAt}
          home={home}
          fromRecording={view.fromRecording}
          onSaved={(saved) => {
            setMessage(`Saved “${saved.name}”.`);
            setView({ kind: 'list', selectedId: saved.id });
            void reload();
          }}
          onCancel={() => setView({ kind: 'list', selectedId: view.returnTo })}
        />
      </section>
    );
  }

  const selected = drafts.find((d) => d.id === view.selectedId) ?? null;
  const select = (id: string | null) => {
    setMessage(null);
    setView({ kind: 'list', selectedId: id });
  };

  // The recording stays on the robot until the field is saved, so cancelling doesn't lose it.
  const useRecording = (points: LatLon[]) => openEditor(newDraft('recorded', points), true);

  const recordingVisible = recording && recording.state !== RECORDING_STATE.IDLE;

  return (
    <section className="page wide">
      <h1>Manage Fields</h1>
      {error && <p className="error">{error}</p>}
      {message && <p className="note" role="status">{message}</p>}

      <div className="field-gallery" role="list">
        <button type="button" className={`field-card new${view.selectedId === null ? ' active' : ''}`} onClick={() => select(null)} role="listitem">
          <span className="new-icon" aria-hidden="true">+</span>
          <span className="field-name">New field</span>
        </button>
        {fields === null && link.status === 'connected' && <p className="muted">Loading fields…</p>}
        {drafts.map((d, i) => (
          <button
            type="button"
            key={d.id}
            role="listitem"
            className={`field-card${d.id === view.selectedId ? ' active' : ''}`}
            aria-pressed={d.id === view.selectedId}
            onClick={() => select(d.id)}
          >
            <FieldThumb field={d} />
            <span className="field-name">{d.name}</span>
            <span className="muted small">{formatArea(fields![i].area_m2)}</span>
          </button>
        ))}
      </div>

      <div className="fields-layout">
        <div className="fields-side">
          {recordingVisible && (
            <Card title="Recording a boundary">
              <RecordingControls status={recording} purpose="field boundary" useLabel="Create field from this boundary" onUse={useRecording} />
            </Card>
          )}
          {selected ? (
            <FieldDetails
              field={selected}
              areaM2={fields!.find((f) => f.id === selected.id)!.area_m2}
              created={fields!.find((f) => f.id === selected.id)!.created}
              modified={fields!.find((f) => f.id === selected.id)!.modified}
              otherNames={drafts.filter((d) => d.id !== selected.id).map((d) => d.name)}
              onEdit={() => openEditor(selected)}
              onDeleted={() => {
                setMessage(`Deleted “${selected.name}”.`);
                setView({ kind: 'list', selectedId: null });
                void reload();
              }}
              onRenamed={() => void reload()}
            />
          ) : (
            !recordingVisible && (
              <AddFieldCard
                canRecord={link.status === 'connected'}
                onDraw={() => openEditor(newDraft('drawn'))}
                onRecordStarted={() => setMessage(null)}
              />
            )
          )}
          <HomeCard home={home} robotPosition={robotPosition} />
        </div>

        <MapView className="fields-map">
          <FitTo points={selected ? selected.boundary : allPoints.length ? allPoints : startAt} fitKey={`${view.selectedId}:${fields === null ? 'loading' : drafts.length}`} />
          <FieldsLayer fields={drafts} selectedId={view.selectedId} onSelect={select} />
          <HomeMarker position={home ?? null} />
          <RobotMarker />
          {recordingVisible && <TrackLayer points={recording.points} closed={recording.state === RECORDING_STATE.FINISHED} />}
          <HomePlacing />
        </MapView>
      </div>
    </section>
  );
}

// Home placing is triggered from HomeCard and handled on the map; a tiny shared store links them.
const placing = { active: false, listeners: new Set<() => void>(), onPick: null as ((p: LatLon) => void) | null };
function setPlacing(active: boolean, onPick: ((p: LatLon) => void) | null = null) {
  placing.active = active;
  placing.onPick = onPick;
  placing.listeners.forEach((l) => l());
}
const subscribePlacing = (l: () => void) => {
  placing.listeners.add(l);
  return () => placing.listeners.delete(l);
};
function usePlacing() {
  return useSyncExternalStore(subscribePlacing, () => placing.active);
}
function HomePlacing() {
  const active = usePlacing();
  const pick = useCallback((p: LatLon) => {
    const cb = placing.onPick;
    setPlacing(false);
    cb?.(p);
  }, []);
  return <MapClickPicker active={active} onPick={pick} />;
}

function AddFieldCard({ canRecord, onDraw, onRecordStarted }: { canRecord: boolean; onDraw: () => void; onRecordStarted: () => void }) {
  const robot = useRobot();
  const [error, setError] = useState<string | null>(null);
  async function startRecording() {
    setError(null);
    try {
      const resp = await robot.callService<object, TriggerResponse>(SERVICES.startRecording, {});
      if (!resp.success) setError(resp.message);
      else onRecordStarted();
    } catch (err) {
      setError((err as Error).message);
    }
  }
  return (
    <Card title="Add new field">
      <div className="add-options">
        <div>
          <button type="button" className="primary" disabled={!canRecord} onClick={() => void startRecording()}>
            Start recording
          </button>
          <p className="muted small">
            Drive the robot around the edge of the field with the controller while it records its GPS position. You can also start
            and stop recording by holding the controller's record button for a second.
          </p>
        </div>
        <div>
          <button type="button" className="secondary" onClick={onDraw}>
            Draw on map
          </button>
          <p className="muted small">Click the corners of the field on the map.</p>
        </div>
      </div>
      {error && <p className="error">{error}</p>}
    </Card>
  );
}

function FieldDetails({
  field,
  areaM2,
  created,
  modified,
  otherNames,
  onEdit,
  onDeleted,
  onRenamed,
}: {
  field: FieldDraft;
  areaM2: number;
  created: { sec: number; nanosec: number };
  modified: { sec: number; nanosec: number };
  otherNames: string[];
  onEdit: () => void;
  onDeleted: () => void;
  onRenamed: () => void;
}) {
  const robot = useRobot();
  const [confirmDelete, setConfirmDelete] = useState(false);
  const [renaming, setRenaming] = useState(false);
  const [newName, setNewName] = useState(field.name);
  const [error, setError] = useState<string | null>(null);

  async function remove() {
    setConfirmDelete(false);
    try {
      const resp = await robot.callService<{ id: string }, TriggerResponse>(SERVICES.deleteField, { id: field.id });
      if (resp.success) onDeleted();
      else setError(resp.message);
    } catch (err) {
      setError((err as Error).message);
    }
  }

  async function rename() {
    const name = newName.trim();
    if (!name) return setError('Give the field a name.');
    if (otherNames.some((n) => n.toLowerCase() === name.toLowerCase())) return setError(`There is already a field called “${name}”.`);
    try {
      const resp = await robot.callService<{ field: unknown }, SaveFieldResponse>(SERVICES.saveField, {
        field: msgFromDraft({ ...field, name }),
      });
      if (resp.success) {
        setRenaming(false);
        setError(null);
        onRenamed();
      } else setError(resp.message);
    } catch (err) {
      setError((err as Error).message);
    }
  }

  const SOURCE = { drawn: 'Drawn on map', recorded: 'Recorded by driving', imported: 'Imported' };
  return (
    <Card title={field.name}>
      <dl className="readings">
        <div className="reading">
          <dt>Area</dt>
          <dd>{formatArea(areaM2)}</dd>
        </div>
        <div className="reading">
          <dt>Headland</dt>
          <dd>{field.headlandWidthM.toFixed(1)} m</dd>
        </div>
        <div className="reading">
          <dt>Keep-out zones</dt>
          <dd>{field.keepOut.length}</dd>
        </div>
        <div className="reading">
          <dt>Made by</dt>
          <dd>{SOURCE[field.source]}</dd>
        </div>
        <div className="reading">
          <dt>Created</dt>
          <dd>{formatDate(created)}</dd>
        </div>
        <div className="reading">
          <dt>Changed</dt>
          <dd>{formatDate(modified)}</dd>
        </div>
        {field.notes && (
          <div className="reading">
            <dt>Notes</dt>
            <dd>{field.notes}</dd>
          </div>
        )}
      </dl>
      {renaming ? (
        <form
          className="rename-row"
          onSubmit={(e) => {
            e.preventDefault();
            void rename();
          }}
        >
          <input aria-label="New name" value={newName} maxLength={60} onChange={(e) => setNewName(e.target.value)} autoFocus />
          <button type="submit" className="primary small">
            Save
          </button>
          <button type="button" className="secondary small" onClick={() => setRenaming(false)}>
            Cancel
          </button>
        </form>
      ) : (
        <div className="button-row details-actions">
          <a className="button primary" href={`#/spray?field=${encodeURIComponent(field.id)}`}>
            Spray this field
          </a>
          <button type="button" className="secondary" onClick={onEdit}>
            Edit
          </button>
          <button
            type="button"
            className="secondary"
            onClick={() => {
              setNewName(field.name);
              setRenaming(true);
            }}
          >
            Rename
          </button>
          <button type="button" className="secondary danger-text" onClick={() => setConfirmDelete(true)}>
            Delete
          </button>
        </div>
      )}
      {error && <p className="error">{error}</p>}
      <ConfirmDialog
        open={confirmDelete}
        title={`Delete “${field.name}”?`}
        confirmLabel="Delete field"
        danger
        onConfirm={() => void remove()}
        onCancel={() => setConfirmDelete(false)}
      >
        <p>The field's boundary, keep-out zones and settings will be removed from the robot. This can't be undone.</p>
      </ConfirmDialog>
    </Card>
  );
}

function HomeCard({ home, robotPosition }: { home: LatLon | null | undefined; robotPosition: LatLon | null }) {
  const robot = useRobot();
  const active = usePlacing();
  const [error, setError] = useState<string | null>(null);
  const [note, setNote] = useState<string | null>(null);
  useEffect(() => () => setPlacing(false), []);

  async function setHome(position: LatLon) {
    setError(null);
    try {
      const resp = await robot.callService<{ position: LatLon }, TriggerResponse>(SERVICES.setHome, { position });
      if (resp.success) setNote('Home point set.');
      else setError(resp.message);
    } catch (err) {
      setError((err as Error).message);
    }
  }

  return (
    <Card title="Home point">
      <p className="muted small">Where the robot goes to have its tank refilled. A spray job can't start until it is set.</p>
      <p>
        {home === undefined
          ? 'Loading…'
          : home === null
            ? 'Not set.'
            : `Set at ${home.latitude.toFixed(6)}, ${home.longitude.toFixed(6)}.`}
      </p>
      <div className="button-row">
        <button type="button" className="secondary small" disabled={!robotPosition} onClick={() => robotPosition && void setHome(robotPosition)}>
          Use robot's position
        </button>
        {active ? (
          <button type="button" className="secondary small" onClick={() => setPlacing(false)}>
            Cancel placing
          </button>
        ) : (
          <button type="button" className="secondary small" onClick={() => setPlacing(true, (p) => void setHome(p))}>
            Place on map
          </button>
        )}
      </div>
      {active && <p className="hint">Click the map where home should be.</p>}
      {note && <p className="note">{note}</p>}
      {error && <p className="error">{error}</p>}
    </Card>
  );
}
