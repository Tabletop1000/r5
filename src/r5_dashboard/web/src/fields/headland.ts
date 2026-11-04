import { buffer } from '@turf/buffer';
import { polygon } from '@turf/helpers';
import type { Feature, MultiPolygon, Polygon } from 'geojson';
import type { LatLon } from '../ros/topics';
import { toRing } from './geo';

/**
 * The part of the field inside the headland (spec FM-11): the boundary shrunk by the headland
 * width. Null if the headland swallows the whole field or the shape is invalid.
 */
export function innerArea(boundary: LatLon[], headlandWidthM: number): Feature<Polygon | MultiPolygon> | null {
  if (boundary.length < 3) return null;
  try {
    const shape = polygon([toRing(boundary)]);
    if (headlandWidthM <= 0) return shape;
    return (buffer(shape, -headlandWidthM, { units: 'meters' }) as Feature<Polygon | MultiPolygon> | undefined) ?? null;
  } catch {
    return null;
  }
}
