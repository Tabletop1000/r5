import { useEffect, useRef, type MutableRefObject } from 'react';
import { TerraDraw, TerraDrawPolygonMode, TerraDrawSelectMode, type GeoJSONStoreFeatures } from 'terra-draw';
import { TerraDrawMapLibreGLAdapter } from 'terra-draw-maplibre-gl-adapter';
import { fromRing, toRing } from '../fields/geo';
import type { LatLon } from '../ros/topics';
import { FIELD_COLORS } from './layers';
import { useMap } from './MapView';

export type Role = 'boundary' | 'keep_out';

export interface DrawShapes {
  boundary: LatLon[];
  keepOut: LatLon[][];
}

/** Commands the editor form sends to the map. */
export interface DrawApi {
  /** Start drawing a new shape. A new boundary replaces the old one. */
  draw(role: Role): void;
  /** Stop drawing and go back to editing corners. */
  cancelDrawing(): void;
  addKeepOut(points: LatLon[]): void;
  removeSelected(): void;
}

interface Props {
  initial: DrawShapes;
  apiRef: MutableRefObject<DrawApi | null>;
  onChange: (shapes: DrawShapes) => void;
  onSelect: (role: Role | null) => void;
  onModeChange: (drawing: Role | null) => void;
  /** A starting shape couldn't be loaded for editing. */
  onLoadProblem: (message: string) => void;
}

const color = (f: GeoJSONStoreFeatures) => (f.properties.role === 'keep_out' ? FIELD_COLORS.keepOut : FIELD_COLORS.selected);

/**
 * Drawing and corner editing on the map with terra-draw (spec FM-3, FM-5).
 * In edit mode corners can be dragged, midpoints dragged to add corners, and corners
 * right-clicked to delete them.
 */
export function DrawLayer({ initial, apiRef, onChange, onSelect, onModeChange, onLoadProblem }: Props) {
  const map = useMap();
  const callbacks = useRef({ onChange, onSelect, onModeChange, onLoadProblem });
  callbacks.current = { onChange, onSelect, onModeChange, onLoadProblem };
  const initialRef = useRef(initial);

  useEffect(() => {
    const draw = new TerraDraw({
      adapter: new TerraDrawMapLibreGLAdapter({ map }),
      modes: [
        new TerraDrawPolygonMode({
          pointerDistance: 30,
          styles: {
            fillColor: FIELD_COLORS.selected,
            fillOpacity: 0.15,
            outlineColor: FIELD_COLORS.selected,
            outlineWidth: 3,
            closingPointColor: '#ffffff',
            closingPointOutlineColor: FIELD_COLORS.selected,
            closingPointWidth: 7,
            closingPointOutlineWidth: 3,
          },
        }),
        new TerraDrawSelectMode({
          pointerDistance: 30,
          flags: {
            polygon: {
              feature: {
                draggable: false,
                selfIntersectable: false,
                coordinates: { draggable: true, midpoints: true, deletable: true },
              },
            },
          },
          styles: {
            selectedPolygonColor: color,
            selectedPolygonFillOpacity: 0.15,
            selectedPolygonOutlineColor: color,
            selectedPolygonOutlineWidth: 3,
            selectionPointColor: '#ffffff',
            selectionPointOutlineColor: color,
            selectionPointWidth: 7,
            selectionPointOutlineWidth: 3,
            midPointColor: color,
            midPointOutlineColor: '#ffffff',
            midPointWidth: 4,
          },
        }),
      ],
    });
    let drawingRole: Role | null = null;
    const featuresOf = (role: Role) =>
      draw.getSnapshot().filter((f) => f.properties.role === role && f.geometry.type === 'Polygon');

    const emit = () => {
      const ring = (f: GeoJSONStoreFeatures) => fromRing((f.geometry.coordinates as number[][][])[0]);
      const [boundary] = featuresOf('boundary');
      callbacks.current.onChange({
        boundary: boundary ? ring(boundary) : [],
        keepOut: featuresOf('keep_out').map(ring),
      });
    };
    const add = (points: LatLon[], role: Role) => {
      const id = draw.getFeatureId();
      const [result] = draw.addFeatures([
        { id, type: 'Feature', geometry: { type: 'Polygon', coordinates: [toRing(points)] }, properties: { mode: 'polygon', role } },
      ]);
      if (result && !result.valid) {
        const what = role === 'boundary' ? 'The boundary' : 'A keep-out zone';
        callbacks.current.onLoadProblem(`${what} couldn't be loaded for editing (${result.reason ?? 'invalid shape'}). Redraw it.`);
      }
      return id;
    };
    const setDrawing = (role: Role | null) => {
      drawingRole = role;
      draw.setMode(role ? 'polygon' : 'select');
      callbacks.current.onModeChange(role);
    };

    // The adapter attaches to the map asynchronously; features and modes only work after 'ready'.
    draw.on('ready', () => {
      const { boundary, keepOut } = initialRef.current;
      if (boundary.length >= 3) add(boundary, 'boundary');
      for (const z of keepOut) if (z.length >= 3) add(z, 'keep_out');
      setDrawing(boundary.length >= 3 ? null : 'boundary');
      emit();
    });
    draw.on('finish', (id, context) => {
      if (context.action === 'draw' && drawingRole) {
        if (drawingRole === 'boundary')
          draw.removeFeatures(featuresOf('boundary').map((f) => f.id as string).filter((fid) => fid !== id));
        draw.updateFeatureProperties(id, { role: drawingRole });
        setDrawing(null);
        draw.selectFeature(id);
      }
      emit();
    });
    draw.on('change', (_ids, type) => {
      if (type !== 'styling') emit();
    });
    draw.on('select', (id) => {
      const role = draw.getSnapshotFeature(id)?.properties.role;
      callbacks.current.onSelect(role === 'boundary' || role === 'keep_out' ? role : null);
    });
    draw.on('deselect', () => callbacks.current.onSelect(null));
    draw.start();

    apiRef.current = {
      draw: (role) => setDrawing(role),
      cancelDrawing: () => setDrawing(null),
      addKeepOut: (points) => {
        add(points, 'keep_out');
        emit();
      },
      removeSelected: () => {
        const selected = draw.getSnapshot().filter((f) => f.properties.selected && f.properties.role === 'keep_out');
        draw.removeFeatures(selected.map((f) => f.id as string));
        callbacks.current.onSelect(null);
        emit();
      },
    };

    return () => {
      apiRef.current = null;
      try {
        draw.stop();
      } catch {
        // The map has already been removed.
      }
    };
  }, [map, apiRef]);

  return null;
}
