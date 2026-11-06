import {
  LngLatBounds,
  Marker,
  type CircleLayerSpecification,
  type FillLayerSpecification,
  type LayerSpecification,
  type LineLayerSpecification,
  type MapLayerMouseEvent,
} from 'maplibre-gl';
import { useEffect, useMemo, useRef } from 'react';
import type { Feature, FeatureCollection } from 'geojson';
import { innerArea } from '../fields/headland';
import { toRing, type FieldDraft } from '../fields/geo';
import { useTopic } from '../ros/RobotContext';
import { TOPICS, type Imu, type LatLon, type NavSatFix } from '../ros/topics';
import { compassHeading, quaternionToEuler } from '../robot/imu';
import { useRobotName } from '../settings/useSettings';
import { useMap } from './MapView';

type WithoutSource<T> = T extends unknown ? Omit<T, 'source'> : never;
type LayerWithoutSource = WithoutSource<FillLayerSpecification | LineLayerSpecification | CircleLayerSpecification>;

/** Adds a GeoJSON source and its layers while mounted, and keeps the data current. */
export function useGeoJsonLayers(id: string, data: FeatureCollection, layers: LayerWithoutSource[]) {
  const map = useMap();
  const layersRef = useRef(layers);
  useEffect(() => {
    map.addSource(id, { type: 'geojson', data });
    for (const layer of layersRef.current) map.addLayer({ ...layer, source: id } as LayerSpecification);
    return () => {
      try {
        for (const layer of layersRef.current) if (map.getLayer(layer.id)) map.removeLayer(layer.id);
        if (map.getSource(id)) map.removeSource(id);
      } catch {
        // The map has already been removed.
      }
    };
    // Only on mount: data updates go through setData below.
  }, [map, id]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => {
    const source = map.getSource(id);
    if (source && 'setData' in source) (source as { setData: (d: FeatureCollection) => void }).setData(data);
  }, [map, id, data]);
}

type Hex = `#${string}`;

export const FIELD_COLORS: Record<'headland' | 'inner' | 'keepOut' | 'outline' | 'selected', Hex> = {
  headland: '#d9a441',
  inner: '#4caf73',
  keepOut: '#d64545',
  outline: '#1f6b3f',
  selected: '#0b5cad',
};

function fieldFeatures(f: FieldDraft, selected: boolean): Feature[] {
  if (f.boundary.length < 3) return [];
  const features: Feature[] = [
    { type: 'Feature', properties: { id: f.id, kind: 'field', selected }, geometry: { type: 'Polygon', coordinates: [toRing(f.boundary)] } },
  ];
  const inner = innerArea(f.boundary, f.headlandWidthM);
  if (inner) features.push({ ...inner, properties: { id: f.id, kind: 'inner', selected } });
  for (const zone of f.keepOut)
    if (zone.length >= 3)
      features.push({ type: 'Feature', properties: { id: f.id, kind: 'keepout' }, geometry: { type: 'Polygon', coordinates: [toRing(zone)] } });
  return features;
}

/**
 * Fields on the map: the headland band (whole field in amber, the inner part in green on top),
 * keep-out zones in red, and outlines (spec FM-2, FM-11).
 */
export function FieldsLayer({
  fields,
  selectedId,
  onSelect,
  id = 'fields',
}: {
  fields: FieldDraft[];
  selectedId?: string | null;
  onSelect?: (id: string) => void;
  id?: string;
}) {
  const map = useMap();
  const data = useMemo<FeatureCollection>(
    () => ({ type: 'FeatureCollection', features: fields.flatMap((f) => fieldFeatures(f, f.id === selectedId)) }),
    [fields, selectedId],
  );
  useGeoJsonLayers(id, data, [
    { id: `${id}-headland`, type: 'fill', filter: ['==', ['get', 'kind'], 'field'], paint: { 'fill-color': FIELD_COLORS.headland, 'fill-opacity': 0.45 } },
    { id: `${id}-inner`, type: 'fill', filter: ['==', ['get', 'kind'], 'inner'], paint: { 'fill-color': FIELD_COLORS.inner, 'fill-opacity': 0.45 } },
    { id: `${id}-keepout`, type: 'fill', filter: ['==', ['get', 'kind'], 'keepout'], paint: { 'fill-color': FIELD_COLORS.keepOut, 'fill-opacity': 0.6 } },
    { id: `${id}-keepout-line`, type: 'line', filter: ['==', ['get', 'kind'], 'keepout'], paint: { 'line-color': FIELD_COLORS.keepOut, 'line-width': 2 } },
    {
      id: `${id}-outline`,
      type: 'line',
      filter: ['==', ['get', 'kind'], 'field'],
      paint: {
        'line-color': ['case', ['get', 'selected'], FIELD_COLORS.selected, FIELD_COLORS.outline],
        'line-width': ['case', ['get', 'selected'], 4, 2],
      },
    },
  ]);
  useEffect(() => {
    if (!onSelect) return;
    const layer = `${id}-headland`;
    const click = (e: MapLayerMouseEvent) => {
      const fid = e.features?.[0]?.properties?.id;
      if (typeof fid === 'string') onSelect(fid);
    };
    const enter = () => (map.getCanvas().style.cursor = 'pointer');
    const leave = () => (map.getCanvas().style.cursor = '');
    map.on('click', layer, click);
    map.on('mouseenter', layer, enter);
    map.on('mouseleave', layer, leave);
    return () => {
      map.off('click', layer, click);
      map.off('mouseenter', layer, enter);
      map.off('mouseleave', layer, leave);
    };
  }, [map, id, onSelect]);
  return null;
}

/** The recording track: a line through the positions logged so far, or the finished ring. */
export function TrackLayer({ points, closed }: { points: LatLon[]; closed: boolean }) {
  const data = useMemo<FeatureCollection>(() => {
    const coords = points.map((p) => [p.longitude, p.latitude]);
    if (closed && coords.length > 2) coords.push(coords[0]);
    return {
      type: 'FeatureCollection',
      features: coords.length > 1 ? [{ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: coords } }] : [],
    };
  }, [points, closed]);
  useGeoJsonLayers('track', data, [
    { id: 'track-line', type: 'line', paint: { 'line-color': '#0b5cad', 'line-width': 3, 'line-dasharray': closed ? [1, 0] : [2, 1] } },
  ]);
  return null;
}

function useMarker(className: string, position: LatLon | null, rotation = 0, title = '') {
  const map = useMap();
  const marker = useRef<Marker | null>(null);
  useEffect(() => {
    const el = document.createElement('div');
    el.className = className;
    el.title = title;
    marker.current = new Marker({ element: el, rotationAlignment: 'map' });
    return () => {
      marker.current?.remove();
      marker.current = null;
    };
  }, [map, className, title]);
  useEffect(() => {
    const m = marker.current;
    if (!m) return;
    if (!position) {
      m.remove();
      return;
    }
    m.setLngLat([position.longitude, position.latitude]).setRotation(rotation);
    if (!m.getElement().isConnected) m.addTo(map);
  }, [map, position, rotation]);
}

/** Robot position from /fix and heading from the IMU. */
export function RobotMarker() {
  const fix = useTopic<NavSatFix>(TOPICS.fix).message;
  const imu = useTopic<Imu>(TOPICS.imu).message;
  const position = fix && fix.status.status >= 0 ? { latitude: fix.latitude, longitude: fix.longitude } : null;
  const heading = imu ? compassHeading(quaternionToEuler(imu.orientation).yawDeg) : 0;
  const name = useRobotName();
  useMarker('robot-marker', position, heading, name);
  useLabel('robot-label', position, name);
  return null;
}

/** Text beside a marker. A marker of its own, so it stays upright when the robot marker turns. */
function useLabel(className: string, position: LatLon | null, text: string) {
  const map = useMap();
  const marker = useRef<Marker | null>(null);
  useEffect(() => {
    const el = document.createElement('div');
    el.className = className;
    el.textContent = text;
    marker.current = new Marker({ element: el, anchor: 'left', offset: [16, 0] });
    return () => {
      marker.current?.remove();
      marker.current = null;
    };
  }, [map, className, text]);
  useEffect(() => {
    const m = marker.current;
    if (!m) return;
    if (!position) {
      m.remove();
      return;
    }
    m.setLngLat([position.longitude, position.latitude]);
    if (!m.getElement().isConnected) m.addTo(map);
  }, [map, position, text]);
}

export function HomeMarker({ position }: { position: LatLon | null }) {
  useMarker('home-marker', position, 0, 'Home');
  return null;
}

/** Moves the view to show these points, when the key changes. */
export function FitTo({ points, fitKey, maxZoom = 19 }: { points: LatLon[]; fitKey: string; maxZoom?: number }) {
  const map = useMap();
  const pointsRef = useRef(points);
  pointsRef.current = points;
  useEffect(() => {
    const pts = pointsRef.current;
    if (pts.length === 0) return;
    if (pts.length === 1) {
      map.jumpTo({ center: [pts[0].longitude, pts[0].latitude], zoom: 18 });
      return;
    }
    const b = new LngLatBounds();
    for (const p of pts) b.extend([p.longitude, p.latitude]);
    map.fitBounds(b, { padding: 48, maxZoom, duration: 0 });
  }, [map, fitKey, maxZoom]);
  return null;
}
