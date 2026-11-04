// Field import and export (spec FM-9), so fields can be prepared or checked in QGIS or Google Earth.
// Export writes the same GeoJSON the robot stores. Import reads GeoJSON or KML and opens the result
// in the editor, where it is checked before saving.

import type { FieldMsg, LatLon } from '../ros/topics';
import { DEFAULT_HEADLAND_M, LocalProjection, ringArea, toRing } from './geo';

// ---- Export ----

export function fieldToGeoJson(f: FieldMsg): string {
  const iso = (t: { sec: number }) => (t.sec ? new Date(t.sec * 1000).toISOString().replace(/\.\d+Z$/, 'Z') : null);
  const collection = {
    type: 'FeatureCollection',
    features: [
      {
        type: 'Feature',
        properties: {
          role: 'boundary',
          id: f.id,
          name: f.name,
          headland_width_m: f.headland_width_m,
          source: f.source,
          notes: f.notes,
          area_m2: Math.round(f.area_m2 * 100) / 100,
          created: iso(f.created),
          modified: iso(f.modified),
        },
        geometry: { type: 'Polygon', coordinates: [toRing(f.boundary)] },
      },
      ...f.keep_out.map((z, i) => ({
        type: 'Feature',
        properties: { role: 'keep_out', index: i },
        geometry: { type: 'Polygon', coordinates: [toRing(z.points)] },
      })),
    ],
  };
  return JSON.stringify(collection, null, 2) + '\n';
}

export function exportFileName(name: string): string {
  const safe = name.trim().replace(/[^\p{L}\p{N}_-]+/gu, '_').replace(/^_+|_+$/g, '') || 'field';
  return `${safe}.geojson`;
}

// ---- Import ----

export interface ImportedField {
  name: string;
  boundary: LatLon[];
  keepOut: LatLon[][];
  headlandWidthM: number;
  notes: string;
  /** Things the user should know, e.g. which polygon was used. */
  notices: string[];
}

export class ImportError extends Error {}

type Ring = number[][]; // [lon, lat, (alt)][]
interface Poly {
  outer: Ring;
  holes: Ring[];
  props: Record<string, unknown>;
}

function toLatLon(ring: Ring): LatLon[] {
  const pts = ring.map((c) => {
    if (!Array.isArray(c) || c.length < 2 || !c.slice(0, 2).every((v) => typeof v === 'number' && Number.isFinite(v)))
      throw new ImportError('A coordinate in the file is not a pair of numbers.');
    const [lon, lat] = c;
    if (Math.abs(lat) > 90 || Math.abs(lon) > 180)
      throw new ImportError(
        'The coordinates are not latitude/longitude. Save the file in WGS 84 (EPSG:4326); in QGIS use Export → Save Features As, CRS EPSG:4326.',
      );
    return { latitude: lat, longitude: lon };
  });
  const first = pts[0];
  const last = pts[pts.length - 1];
  if (pts.length > 1 && first.latitude === last.latitude && first.longitude === last.longitude) pts.pop();
  return pts;
}

function areaOf(ring: LatLon[]): number {
  if (ring.length < 3) return 0;
  const proj = LocalProjection.around(ring);
  return ringArea(ring.map((p) => proj.toXY(p)));
}

function geoJsonPolygons(data: unknown): Poly[] {
  const out: Poly[] = [];
  const visit = (node: unknown, props: Record<string, unknown>) => {
    if (!node || typeof node !== 'object') return;
    const n = node as { type?: string; features?: unknown[]; geometry?: unknown; properties?: Record<string, unknown>; coordinates?: unknown; geometries?: unknown[] };
    switch (n.type) {
      case 'FeatureCollection':
        (n.features ?? []).forEach((f) => visit(f, {}));
        break;
      case 'Feature':
        visit(n.geometry, n.properties ?? {});
        break;
      case 'GeometryCollection':
        (n.geometries ?? []).forEach((g) => visit(g, props));
        break;
      case 'Polygon': {
        const [outer, ...holes] = n.coordinates as Ring[];
        if (outer) out.push({ outer, holes, props });
        break;
      }
      case 'MultiPolygon':
        for (const poly of n.coordinates as Ring[][]) {
          const [outer, ...holes] = poly;
          if (outer) out.push({ outer, holes, props });
        }
        break;
    }
  };
  const crs = (data as { crs?: { properties?: { name?: string } } })?.crs?.properties?.name;
  if (crs && !/CRS84|4326/.test(crs))
    throw new ImportError(`The file uses the coordinate system ${crs}. Save it in WGS 84 (EPSG:4326) and try again.`);
  visit(data, {});
  return out;
}

function kmlPolygons(text: string): Poly[] {
  // Plain text matching is enough for KML from Google Earth and QGIS, and works without a browser.
  const tag = (name: string) => new RegExp(`<(?:\\w+:)?${name}\\b[^>]*>([\\s\\S]*?)</(?:\\w+:)?${name}>`, 'gi');
  const first = (xml: string, name: string) => tag(name).exec(xml)?.[1] ?? null;
  const all = (xml: string, name: string) => Array.from(xml.matchAll(tag(name)), (m) => m[1]);
  const coords = (xml: string | null): Ring =>
    (first(xml ?? '', 'coordinates') ?? '')
      .trim()
      .split(/\s+/)
      .filter(Boolean)
      .map((t) => t.split(',').map(Number));
  if (!/<(?:\w+:)?kml\b/i.test(text)) throw new ImportError("The KML file couldn't be read.");
  const out: Poly[] = [];
  for (const placemark of all(text, 'Placemark')) {
    const name = first(placemark, 'name')?.replace(/<!\[CDATA\[|\]\]>/g, '').trim();
    for (const polygon of all(placemark, 'Polygon')) {
      const outer = coords(first(polygon, 'outerBoundaryIs'));
      const holes = all(polygon, 'innerBoundaryIs').map((ib) => coords(ib));
      if (outer.length) out.push({ outer, holes, props: name ? { name } : {} });
    }
  }
  return out;
}

function stem(fileName: string): string {
  return fileName.replace(/\.[^.]+$/, '').replace(/[_-]+/g, ' ').trim();
}

/**
 * Reads a field from GeoJSON or KML. The boundary is the polygon marked as such (by this
 * dashboard's own exports) or else the largest polygon. Holes in it, and polygons marked as
 * keep-out zones or lying inside it, become keep-out zones.
 */
export function parseFieldFile(text: string, fileName: string): ImportedField {
  const isKml = /\.kml$/i.test(fileName) || /^\s*<\?xml|<kml[\s>]/i.test(text.slice(0, 200));
  let polys: Poly[];
  if (isKml) {
    polys = kmlPolygons(text);
  } else {
    let data: unknown;
    try {
      data = JSON.parse(text);
    } catch {
      throw new ImportError("The file isn't valid GeoJSON or KML.");
    }
    polys = geoJsonPolygons(data);
  }
  if (polys.length === 0) throw new ImportError('The file has no polygon in it. A field needs an area, not just lines or points.');

  const notices: string[] = [];
  const withArea = polys.map((p) => ({ ...p, ll: toLatLon(p.outer), holesLl: p.holes.map(toLatLon) })).map((p) => ({ ...p, area: areaOf(p.ll) }));
  const marked = withArea.find((p) => p.props.role === 'boundary');
  const boundary = marked ?? withArea.reduce((a, b) => (b.area > a.area ? b : a));
  if (boundary.ll.length < 3) throw new ImportError('The boundary has fewer than 3 corners.');

  const keepOut: LatLon[][] = [...boundary.holesLl];
  const others = withArea.filter((p) => p !== boundary);
  for (const p of others) {
    if (p.props.role === 'keep_out' || p.area < boundary.area) keepOut.push(p.ll);
  }
  if (!marked && others.length > 0)
    notices.push(
      `The file has ${withArea.length} polygons. The largest is used as the boundary and the others as keep-out zones; check them before saving.`,
    );

  const props = boundary.props;
  const name = String(props.name ?? props.Name ?? props.NAME ?? '').trim() || stem(fileName);
  const headland = Number(props.headland_width_m);
  return {
    name: name.slice(0, 60),
    boundary: boundary.ll,
    keepOut,
    headlandWidthM: Number.isFinite(headland) && headland >= 0 ? headland : DEFAULT_HEADLAND_M,
    notes: typeof props.notes === 'string' ? props.notes : '',
    notices,
  };
}
