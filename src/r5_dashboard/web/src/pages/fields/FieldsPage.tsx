import { useCallback, useMemo, useState } from 'react';
import { Card } from '../../components/Card';
import { ConfirmDialog } from '../../components/ConfirmDialog';
import { FieldThumb } from '../../fields/FieldThumb';
import { DEFAULT_HEADLAND_M, draftFromMsg, formatArea, msgFromDraft, type FieldDraft } from '../../fields/geo';
import { formatDate } from '../../fields/time';
import { exportFileName, fieldToGeoJson, ImportError, parseFieldFile } from '../../fields/importExport';
import { useFields } from '../../fields/useFields';
import { FieldsLayer, FitTo, HomeMarker, RobotMarker, TrackLayer } from '../../map/layers';
import { MapView } from '../../map/MapView';
import { useLinkState, useRobot, useTopic } from '../../ros/RobotContext';
import {
  RECORDING_STATE,
  SERVICES,
  TOPICS,
  type FieldMsg,
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
  | { kind: 'edit'; draft: FieldDraft; fromRecording: boolean; returnTo: string | null; notices?: string[] };

const newDraft = (source: FieldDraft['source'], boundary: LatLon[] = []): FieldDraft => ({
  id: '',
  name: '',
  boundary,
  keepOut: [],
  headlandWidthM: DEFAULT_HEADLAND_M,
  source,
  notes: '',
});

/** Manage Fields (spec 4.1): list, view, add (draw or record), edit, rename, delete. The home point is on the Settings page. */
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
    (draft: FieldDraft, fromRecording = false, notices?: string[]) =>
      setView((v) => ({ kind: 'edit', draft, fromRecording, notices, returnTo: v.kind === 'list' ? v.selectedId : null })),
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
          notices={view.notices}
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
              msg={fields!.find((f) => f.id === selected.id)!}
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
                onImported={(draft, notices) => openEditor(draft, false, notices)}
              />
            )
          )}
        </div>

        <MapView className="fields-map">
          <FitTo points={selected ? selected.boundary : allPoints.length ? allPoints : startAt} fitKey={`${view.selectedId}:${fields === null ? 'loading' : drafts.length}`} />
          <FieldsLayer fields={drafts} selectedId={view.selectedId} onSelect={select} />
          <HomeMarker position={home ?? null} />
          <RobotMarker />
          {recordingVisible && <TrackLayer points={recording.points} closed={recording.state === RECORDING_STATE.FINISHED} />}
        </MapView>
      </div>
    </section>
  );
}

function AddFieldCard({
  canRecord,
  onDraw,
  onRecordStarted,
  onImported,
}: {
  canRecord: boolean;
  onDraw: () => void;
  onRecordStarted: () => void;
  onImported: (draft: FieldDraft, notices: string[]) => void;
}) {
  const robot = useRobot();
  const [error, setError] = useState<string | null>(null);

  async function importFile(file: File | undefined) {
    if (!file) return;
    setError(null);
    if (file.size > 5_000_000) return setError('That file is too big for a field (over 5 MB).');
    try {
      const f = parseFieldFile(await file.text(), file.name);
      onImported(
        { ...newDraft('imported', f.boundary), name: f.name, keepOut: f.keepOut, headlandWidthM: f.headlandWidthM, notes: f.notes },
        f.notices,
      );
    } catch (err) {
      setError(err instanceof ImportError ? err.message : `Couldn't read the file: ${(err as Error).message}`);
    }
  }
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
        <div>
          <label className="button secondary file-button">
            Import file
            <input
              type="file"
              accept=".geojson,.json,.kml,application/geo+json,application/vnd.google-earth.kml+xml"
              onChange={(e) => {
                void importFile(e.target.files?.[0]);
                e.target.value = '';
              }}
            />
          </label>
          <p className="muted small">A GeoJSON or KML polygon, e.g. from QGIS or Google Earth, in latitude/longitude (WGS 84).</p>
        </div>
      </div>
      {error && <p className="error">{error}</p>}
    </Card>
  );
}

function FieldDetails({
  field,
  msg,
  areaM2,
  created,
  modified,
  otherNames,
  onEdit,
  onDeleted,
  onRenamed,
}: {
  field: FieldDraft;
  msg: FieldMsg;
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
          <button type="button" className="secondary" onClick={() => downloadText(fieldToGeoJson(msg), exportFileName(field.name), 'application/geo+json')}>
            Export
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

/** Saves text as a file through the browser (spec FM-9 export). */
function downloadText(text: string, fileName: string, type: string) {
  const url = URL.createObjectURL(new Blob([text], { type }));
  const a = document.createElement('a');
  a.href = url;
  a.download = fileName;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
