import { Marker, type MapMouseEvent } from 'maplibre-gl';
import { useEffect, useMemo, useRef, useState } from 'react';
import type { Feature, FeatureCollection } from 'geojson';
import { formatLength, type Units } from '../display/units';
import { LocalProjection } from '../fields/geo';
import type { LatLon } from '../ros/topics';
import { useGeoJsonLayers } from './layers';
import { useMap } from './MapView';

const GOTO_COLOR = '#1565c0';

/** Straight-line distance in metres, good for the short distances a go-to covers. */
export function distanceM(a: LatLon, b: LatLon): number {
  const proj = LocalProjection.around([a]);
  const [x, y] = proj.toXY(b);
  return Math.hypot(x, y);
}

interface Props {
  robot: LatLon | null;
  /** Clicks place a target only while enabled. */
  enabled: boolean;
  /** The target waiting for confirmation, or the one the robot is driving to. */
  target: LatLon | null;
  units: Units;
  onPick: (target: LatLon) => void;
}

/**
 * Go-to on the map (Robot page). With a mouse, a dotted line runs from the robot to the pointer
 * with the distance beside it; a click picks the point. On a touch screen, a tap picks it and the
 * line is drawn to it instead.
 */
export function GoToLayer({ robot, enabled, target, units, onPick }: Props) {
  const map = useMap();
  const [hover, setHover] = useState<LatLon | null>(null);
  const onPickRef = useRef(onPick);
  onPickRef.current = onPick;

  useEffect(() => {
    if (!enabled) {
      setHover(null);
      return;
    }
    const at = (e: MapMouseEvent): LatLon => ({ latitude: e.lngLat.lat, longitude: e.lngLat.lng });
    const move = (e: MapMouseEvent) => setHover(at(e));
    const leave = () => setHover(null);
    const click = (e: MapMouseEvent) => onPickRef.current(at(e));
    const canvas = map.getCanvas();
    canvas.style.cursor = 'crosshair';
    map.on('mousemove', move);
    map.on('mouseout', leave);
    map.on('click', click);
    return () => {
      map.off('mousemove', move);
      map.off('mouseout', leave);
      map.off('click', click);
      canvas.style.cursor = '';
    };
  }, [map, enabled]);

  // The line follows the pointer, or goes to the chosen point when there's no pointer (touch).
  const end = hover ?? target;
  const data = useMemo<FeatureCollection>(() => {
    const features: Feature[] = [];
    if (robot && end)
      features.push({
        type: 'Feature',
        properties: { kind: 'line' },
        geometry: { type: 'LineString', coordinates: [robot, end].map((p) => [p.longitude, p.latitude]) },
      });
    if (target)
      features.push({ type: 'Feature', properties: { kind: 'target' }, geometry: { type: 'Point', coordinates: [target.longitude, target.latitude] } });
    return { type: 'FeatureCollection', features };
  }, [robot, end, target]);

  useGeoJsonLayers('goto', data, [
    {
      id: 'goto-line',
      type: 'line',
      filter: ['==', ['get', 'kind'], 'line'],
      layout: { 'line-cap': 'round' },
      paint: { 'line-color': GOTO_COLOR, 'line-width': 3, 'line-dasharray': [1, 2] },
    },
    {
      id: 'goto-target',
      type: 'circle',
      filter: ['==', ['get', 'kind'], 'target'],
      paint: { 'circle-radius': 7, 'circle-color': '#ffffff', 'circle-stroke-color': GOTO_COLOR, 'circle-stroke-width': 3 },
    },
  ]);

  useDistanceLabel(robot && end ? end : null, robot && end ? formatLength(distanceM(robot, end), units) : '');
  return null;
}

/** The distance, beside the end of the line. */
function useDistanceLabel(position: LatLon | null, text: string) {
  const map = useMap();
  const marker = useRef<Marker | null>(null);
  useEffect(() => {
    const el = document.createElement('div');
    el.className = 'goto-label';
    marker.current = new Marker({ element: el, anchor: 'bottom-left', offset: [10, -6] });
    return () => {
      marker.current?.remove();
      marker.current = null;
    };
  }, [map]);
  useEffect(() => {
    const m = marker.current;
    if (!m) return;
    if (!position) {
      m.remove();
      return;
    }
    m.getElement().textContent = text;
    m.setLngLat([position.longitude, position.latitude]);
    if (!m.getElement().isConnected) m.addTo(map);
  }, [map, position, text]);
}
