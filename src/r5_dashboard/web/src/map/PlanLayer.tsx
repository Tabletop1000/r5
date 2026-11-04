import { useMemo } from 'react';
import type { FeatureCollection } from 'geojson';
import { SECTION_KIND, type PathSection } from '../ros/topics';
import { useGeoJsonLayers } from './layers';

/**
 * A coverage path: rows (sprayed) solid, turns and headland moves dashed. During a job the
 * sections already driven are shown darker (spec AS-3, AS-6).
 */
export function PlanLayer({ sections, doneBefore = 0 }: { sections: PathSection[]; doneBefore?: number }) {
  const data = useMemo<FeatureCollection>(
    () => ({
      type: 'FeatureCollection',
      features: sections
        .filter((s) => s.points.length > 1)
        .map((s, i) => ({
          type: 'Feature',
          properties: { swath: s.kind === SECTION_KIND.SWATH, done: i < doneBefore },
          geometry: { type: 'LineString', coordinates: s.points.map((p) => [p.longitude, p.latitude]) },
        })),
    }),
    [sections, doneBefore],
  );
  useGeoJsonLayers('plan', data, [
    {
      id: 'plan-turns',
      type: 'line',
      filter: ['!', ['get', 'swath']],
      paint: { 'line-color': ['case', ['get', 'done'], '#5b6b5b', '#8a948a'], 'line-width': 1.5, 'line-dasharray': [2, 2] },
    },
    {
      id: 'plan-rows',
      type: 'line',
      filter: ['get', 'swath'],
      paint: {
        'line-color': ['case', ['get', 'done'], '#0b5cad', '#1f6b3f'],
        'line-width': ['case', ['get', 'done'], 4, 2.5],
        'line-opacity': ['case', ['get', 'done'], 0.9, 0.8],
      },
    },
  ]);
  return null;
}
