import type { MapMouseEvent } from 'maplibre-gl';
import { useEffect, useMemo, useRef, useState } from 'react';
import type { Feature, FeatureCollection } from 'geojson';
import { LocalProjection, nearestOnRing, type XY } from '../fields/geo';
import type { LatLon } from '../ros/topics';
import { FIELD_COLORS, useGeoJsonLayers } from './layers';
import { useMap } from './MapView';

/** How close to the boundary, in screen pixels, the pointer must be to place a post. */
const SNAP_PX = 40;

interface Props {
  boundary: LatLon[];
  /** The first post, once placed. */
  firstPost: LatLon | null;
  onPick: (post: LatLon) => void;
}

/**
 * Places a gate post on the field boundary. A dot follows the pointer along the nearest part of the
 * boundary, and a click places the post there. On a touch screen, a tap places it at the nearest
 * point on the boundary.
 */
export function GatePicker({ boundary, firstPost, onPick }: Props) {
  const map = useMap();
  const [cursor, setCursor] = useState<LatLon | null>(null);
  const onPickRef = useRef(onPick);
  onPickRef.current = onPick;

  useEffect(() => {
    if (boundary.length < 3) return;
    const proj = LocalProjection.around(boundary);
    const ringM = boundary.map((p) => proj.toXY(p));

    /** The point on the boundary nearest the pointer, if it's close enough on screen. */
    const snap = (e: MapMouseEvent): LatLon | null => {
      const ringPx = boundary.map((p): XY => {
        const { x, y } = map.project([p.longitude, p.latitude]);
        return [x, y];
      });
      const [x, y] = nearestOnRing([e.point.x, e.point.y], ringPx);
      if (Math.hypot(x - e.point.x, y - e.point.y) > SNAP_PX) return null;
      const { lng, lat } = map.unproject([x, y]);
      // Snap again in metres, so the post is exactly on the boundary the robot checks.
      return proj.toLatLon(nearestOnRing(proj.toXY({ latitude: lat, longitude: lng }), ringM));
    };
    const move = (e: MapMouseEvent) => {
      const p = snap(e);
      setCursor(p);
      map.getCanvas().style.cursor = p ? 'pointer' : 'crosshair';
    };
    const click = (e: MapMouseEvent) => {
      const p = snap(e);
      if (p) onPickRef.current(p);
    };
    const leave = () => setCursor(null);
    map.getCanvas().style.cursor = 'crosshair';
    map.on('mousemove', move);
    map.on('click', click);
    map.on('mouseout', leave);
    return () => {
      map.off('mousemove', move);
      map.off('click', click);
      map.off('mouseout', leave);
      map.getCanvas().style.cursor = '';
    };
  }, [map, boundary]);

  const data = useMemo<FeatureCollection>(() => {
    const point = (p: LatLon, kind: string): Feature => ({
      type: 'Feature',
      properties: { kind },
      geometry: { type: 'Point', coordinates: [p.longitude, p.latitude] },
    });
    const features: Feature[] = [];
    if (firstPost && cursor)
      features.push({
        type: 'Feature',
        properties: { kind: 'line' },
        geometry: { type: 'LineString', coordinates: [firstPost, cursor].map((p) => [p.longitude, p.latitude]) },
      });
    if (firstPost) features.push(point(firstPost, 'post'));
    if (cursor) features.push(point(cursor, 'cursor'));
    return { type: 'FeatureCollection', features };
  }, [firstPost, cursor]);

  useGeoJsonLayers('gate-picker', data, [
    {
      id: 'gate-picker-line',
      type: 'line',
      filter: ['==', ['get', 'kind'], 'line'],
      paint: { 'line-color': FIELD_COLORS.gate, 'line-width': 4, 'line-dasharray': [2, 1] },
    },
    {
      id: 'gate-picker-points',
      type: 'circle',
      filter: ['!=', ['get', 'kind'], 'line'],
      paint: {
        'circle-radius': ['case', ['==', ['get', 'kind'], 'cursor'], 7, 6],
        'circle-color': ['case', ['==', ['get', 'kind'], 'cursor'], '#ffffff', FIELD_COLORS.gate],
        'circle-stroke-color': FIELD_COLORS.gate,
        'circle-stroke-width': 3,
      },
    },
  ]);
  return null;
}
