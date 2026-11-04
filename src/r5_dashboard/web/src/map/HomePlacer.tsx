import type { MapMouseEvent } from 'maplibre-gl';
import { useEffect } from 'react';
import type { LatLon } from '../ros/topics';
import { useMap } from './MapView';

/** While active, the next click on the map picks a position. */
export function MapClickPicker({ active, onPick }: { active: boolean; onPick: (p: LatLon) => void }) {
  const map = useMap();
  useEffect(() => {
    if (!active) return;
    const canvas = map.getCanvas();
    canvas.style.cursor = 'crosshair';
    const click = (e: MapMouseEvent) => onPick({ latitude: e.lngLat.lat, longitude: e.lngLat.lng });
    map.once('click', click);
    return () => {
      map.off('click', click);
      canvas.style.cursor = '';
    };
  }, [map, active, onPick]);
  return null;
}
