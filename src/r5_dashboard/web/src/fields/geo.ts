// Field geometry and validation. Mirrors server/.../geo.py and fields.py, so problems show up
// while editing; the robot checks again before saving (spec FM-6).

import { formatArea, formatLength, type Units } from '../display/units';
import type { FieldMsg, LatLon } from '../ros/topics';

export type XY = [number, number];

const EARTH_RADIUS_M = 6_371_008.8;
const DEG = Math.PI / 180;

/** Flat metres east/north around an origin. Accurate to well under a centimetre over a few km. */
export class LocalProjection {
  readonly kx: number;
  readonly ky: number;
  constructor(readonly lat0: number, readonly lon0: number) {
    this.kx = DEG * EARTH_RADIUS_M * Math.cos(lat0 * DEG);
    this.ky = DEG * EARTH_RADIUS_M;
  }
  static around(points: LatLon[]): LocalProjection {
    const n = points.length || 1;
    return new LocalProjection(
      points.reduce((a, p) => a + p.latitude, 0) / n,
      points.reduce((a, p) => a + p.longitude, 0) / n,
    );
  }
  toXY(p: LatLon): XY {
    return [(p.longitude - this.lon0) * this.kx, (p.latitude - this.lat0) * this.ky];
  }
  toLatLon([x, y]: XY): LatLon {
    return { latitude: this.lat0 + y / this.ky, longitude: this.lon0 + x / this.kx };
  }
}

export function ringArea(ring: XY[]): number {
  let s = 0;
  for (let i = 0; i < ring.length; i++) {
    const [x1, y1] = ring[i];
    const [x2, y2] = ring[(i + 1) % ring.length];
    s += x1 * y2 - x2 * y1;
  }
  return Math.abs(s) / 2;
}

const orient = (a: XY, b: XY, c: XY) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
const onSegment = (a: XY, b: XY, p: XY) =>
  Math.min(a[0], b[0]) <= p[0] && p[0] <= Math.max(a[0], b[0]) && Math.min(a[1], b[1]) <= p[1] && p[1] <= Math.max(a[1], b[1]);

export function segmentsIntersect(p1: XY, p2: XY, q1: XY, q2: XY): boolean {
  const d1 = orient(q1, q2, p1);
  const d2 = orient(q1, q2, p2);
  const d3 = orient(p1, p2, q1);
  const d4 = orient(p1, p2, q2);
  if (d1 && d2 && d3 && d4 && d1 > 0 !== d2 > 0 && d3 > 0 !== d4 > 0) return true;
  return (
    (d1 === 0 && onSegment(q1, q2, p1)) ||
    (d2 === 0 && onSegment(q1, q2, p2)) ||
    (d3 === 0 && onSegment(p1, p2, q1)) ||
    (d4 === 0 && onSegment(p1, p2, q2))
  );
}

const edges = (ring: XY[]): [XY, XY][] => ring.map((p, i) => [p, ring[(i + 1) % ring.length]]);

export function ringSelfIntersects(ring: XY[]): boolean {
  const e = edges(ring);
  const n = e.length;
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      if (segmentsIntersect(e[i][0], e[i][1], e[j][0], e[j][1])) return true;
    }
  }
  return false;
}

export function ringsCross(a: XY[], b: XY[]): boolean {
  return edges(a).some(([p1, p2]) => edges(b).some(([q1, q2]) => segmentsIntersect(p1, p2, q1, q2)));
}

export function pointInRing(p: XY, ring: XY[]): boolean {
  let inside = false;
  for (let i = 0; i < ring.length; i++) {
    const [x1, y1] = ring[i];
    const [x2, y2] = ring[(i + 1) % ring.length];
    if (y1 > p[1] !== y2 > p[1] && p[0] < x1 + ((p[1] - y1) * (x2 - x1)) / (y2 - y1)) inside = !inside;
  }
  return inside;
}

export function ringInside(inner: XY[], outer: XY[]): boolean {
  return inner.every((p) => pointInRing(p, outer)) && !ringsCross(inner, outer);
}

export function areaM2(boundary: LatLon[]): number {
  if (boundary.length < 3) return 0;
  const proj = LocalProjection.around(boundary);
  return ringArea(boundary.map((p) => proj.toXY(p)));
}

export const MAX_NAME_LENGTH = 60;
export const MAX_HEADLAND_M = 20;
export const DEFAULT_HEADLAND_M = 2;
/** Outside this range the user gets a warning, not an error (spec FM-6). */
export const TYPICAL_AREA_HA = { min: 0.1, max: 3 };

export interface FieldDraft {
  id: string;
  name: string;
  boundary: LatLon[];
  keepOut: LatLon[][];
  headlandWidthM: number;
  source: 'drawn' | 'recorded' | 'imported';
  notes: string;
}

export interface FieldCheck {
  errors: string[];
  warnings: string[];
  areaM2: number;
}

export function checkField(d: FieldDraft, otherNames: string[], u: Units = 'metric'): FieldCheck {
  const errors: string[] = [];
  const warnings: string[] = [];
  const name = d.name.trim();
  if (!name) errors.push('Give the field a name.');
  else if (name.length > MAX_NAME_LENGTH) errors.push(`The name is too long (at most ${MAX_NAME_LENGTH} characters).`);
  else if (otherNames.some((n) => n.toLowerCase() === name.toLowerCase())) errors.push(`There is already a field called “${name}”.`);
  if (!(d.headlandWidthM >= 0 && d.headlandWidthM <= MAX_HEADLAND_M))
    errors.push(`The headland width must be between 0 and ${formatLength(MAX_HEADLAND_M, u, 0)}.`);

  if (d.boundary.length < 3) {
    errors.push('Draw the boundary: at least 3 corners.');
    return { errors, warnings, areaM2: 0 };
  }
  const proj = LocalProjection.around(d.boundary);
  const outer = d.boundary.map((p) => proj.toXY(p));
  const area = ringArea(outer);
  if (ringSelfIntersects(outer)) errors.push('The boundary crosses itself.');
  else if (area < 1) errors.push('The boundary encloses no area.');

  const zones = d.keepOut.map((z) => z.map((p) => proj.toXY(p)));
  zones.forEach((z, i) => {
    if (z.length < 3) errors.push(`Keep-out zone ${i + 1} needs at least 3 corners.`);
    else if (ringSelfIntersects(z)) errors.push(`Keep-out zone ${i + 1} crosses itself.`);
    else if (!ringInside(z, outer)) errors.push(`Keep-out zone ${i + 1} must lie inside the boundary.`);
  });
  for (let i = 0; i < zones.length; i++)
    for (let j = i + 1; j < zones.length; j++)
      if (ringsCross(zones[i], zones[j])) errors.push(`Keep-out zones ${i + 1} and ${j + 1} overlap. Merge them into one.`);

  const ha = area / 10_000;
  if (errors.length === 0 && (ha < TYPICAL_AREA_HA.min || ha > TYPICAL_AREA_HA.max))
    warnings.push(
      `The field is ${formatArea(area, u)}. R5 is meant for fields of ${formatArea(TYPICAL_AREA_HA.min * 10_000, u)} to ${formatArea(TYPICAL_AREA_HA.max * 10_000, u)}; check the boundary is right.`,
    );
  return { errors, warnings, areaM2: area };
}

export { formatArea } from '../display/units';

// ---- Conversions ----

export function draftFromMsg(f: FieldMsg): FieldDraft {
  return {
    id: f.id,
    name: f.name,
    boundary: f.boundary,
    keepOut: f.keep_out.map((z) => z.points),
    headlandWidthM: f.headland_width_m,
    source: (['drawn', 'recorded', 'imported'].includes(f.source) ? f.source : 'drawn') as FieldDraft['source'],
    notes: f.notes,
  };
}

export function msgFromDraft(d: FieldDraft): Omit<FieldMsg, 'area_m2' | 'created' | 'modified'> {
  return {
    id: d.id,
    name: d.name.trim(),
    boundary: d.boundary,
    keep_out: d.keepOut.map((points) => ({ points })),
    headland_width_m: d.headlandWidthM,
    source: d.source,
    notes: d.notes,
  };
}

/** GeoJSON ring ([lon, lat], closed) from a LatLon ring. */
export function toRing(points: LatLon[]): [number, number][] {
  // 9 decimal places is about 0.1 mm, and terra-draw refuses more.
  const r = (v: number) => Math.round(v * 1e9) / 1e9;
  const ring = points.map((p): [number, number] => [r(p.longitude), r(p.latitude)]);
  return ring.length ? [...ring, ring[0]] : ring;
}

/** LatLon ring (open) from a GeoJSON ring. */
export function fromRing(ring: number[][]): LatLon[] {
  const pts = ring.map(([lon, lat]) => ({ latitude: lat, longitude: lon }));
  if (pts.length > 1 && pts[0].latitude === pts[pts.length - 1].latitude && pts[0].longitude === pts[pts.length - 1].longitude) pts.pop();
  return pts;
}

export interface Bounds {
  west: number;
  south: number;
  east: number;
  north: number;
}

export function boundsOf(points: LatLon[]): Bounds | null {
  if (points.length === 0) return null;
  return {
    west: Math.min(...points.map((p) => p.longitude)),
    east: Math.max(...points.map((p) => p.longitude)),
    south: Math.min(...points.map((p) => p.latitude)),
    north: Math.max(...points.map((p) => p.latitude)),
  };
}
