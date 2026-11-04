import { describe, expect, it } from 'vitest';
import type { FieldMsg } from '../ros/topics';
import { areaM2, LocalProjection } from './geo';
import { exportFileName, fieldToGeoJson, ImportError, parseFieldFile } from './importExport';

const proj = new LocalProjection(-36.85, 174.76);
const ll = (pts: [number, number][]) => pts.map((p) => proj.toLatLon(p));
const square = ll([[0, 0], [100, 0], [100, 100], [0, 100]]);
const zone = ll([[40, 40], [60, 40], [60, 60]]);
const ring = (pts: { latitude: number; longitude: number }[]) => [...pts, pts[0]].map((p) => [p.longitude, p.latitude]);

const field: FieldMsg = {
  id: 'abc',
  name: 'East field',
  boundary: square,
  keep_out: [{ points: zone }],
  headland_width_m: 3,
  source: 'drawn',
  notes: 'Wet corner',
  area_m2: 10_000,
  created: { sec: 1_700_000_000, nanosec: 0 },
  modified: { sec: 1_700_000_100, nanosec: 0 },
};

describe('export', () => {
  it('round-trips through import', () => {
    const back = parseFieldFile(fieldToGeoJson(field), 'East_field.geojson');
    expect(back.name).toBe('East field');
    expect(back.headlandWidthM).toBe(3);
    expect(back.notes).toBe('Wet corner');
    expect(back.keepOut).toHaveLength(1);
    expect(areaM2(back.boundary)).toBeCloseTo(10_000, -1);
    expect(back.notices).toEqual([]);
  });
  it('makes a safe file name', () => {
    expect(exportFileName('River paddock / north')).toBe('River_paddock_north.geojson');
    expect(exportFileName('  ')).toBe('field.geojson');
  });
});

describe('import GeoJSON', () => {
  it('takes a bare polygon with a hole, naming it from the file', () => {
    const f = parseFieldFile(JSON.stringify({ type: 'Polygon', coordinates: [ring(square), ring(zone)] }), 'rice_02.geojson');
    expect(f.name).toBe('rice 02');
    expect(f.keepOut).toHaveLength(1);
    expect(f.headlandWidthM).toBe(2);
  });
  it('uses the largest polygon as the boundary and says so', () => {
    const fc = {
      type: 'FeatureCollection',
      features: [
        { type: 'Feature', properties: { name: 'Shed' }, geometry: { type: 'Polygon', coordinates: [ring(zone)] } },
        { type: 'Feature', properties: { Name: 'Big paddock' }, geometry: { type: 'MultiPolygon', coordinates: [[ring(square)]] } },
      ],
    };
    const f = parseFieldFile(JSON.stringify(fc), 'x.json');
    expect(f.name).toBe('Big paddock');
    expect(f.keepOut).toHaveLength(1);
    expect(f.notices[0]).toMatch('largest');
  });
  it('refuses projected coordinates and other systems with a helpful message', () => {
    expect(() => parseFieldFile(JSON.stringify({ type: 'Polygon', coordinates: [[[1745000, 5920000], [1745100, 5920000], [1745100, 5920100], [1745000, 5920000]]] }), 'a.geojson')).toThrow(/EPSG:4326/);
    const withCrs = { type: 'FeatureCollection', crs: { type: 'name', properties: { name: 'urn:ogc:def:crs:EPSG::2193' } }, features: [] };
    expect(() => parseFieldFile(JSON.stringify(withCrs), 'a.geojson')).toThrow(/EPSG::2193/);
  });
  it('explains files without polygons or that are not GeoJSON', () => {
    expect(() => parseFieldFile(JSON.stringify({ type: 'LineString', coordinates: [[0, 0], [1, 1]] }), 'a.geojson')).toThrow(ImportError);
    expect(() => parseFieldFile('hello', 'a.geojson')).toThrow(/valid GeoJSON or KML/);
  });
});

describe('import KML', () => {
  const coords = (pts: { latitude: number; longitude: number }[]) => ring(pts).map(([lon, lat]) => `${lon},${lat},0`).join(' ');
  const kml = `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><Placemark><name><![CDATA[River paddock]]></name>
<Polygon><outerBoundaryIs><LinearRing><coordinates>
${coords(square)}
</coordinates></LinearRing></outerBoundaryIs>
<innerBoundaryIs><LinearRing><coordinates>${coords(zone)}</coordinates></LinearRing></innerBoundaryIs></Polygon>
</Placemark></Document></kml>`;
  it('reads a Google Earth polygon with a hole', () => {
    const f = parseFieldFile(kml, 'doc.kml');
    expect(f.name).toBe('River paddock');
    expect(f.boundary).toHaveLength(4);
    expect(f.keepOut).toHaveLength(1);
    expect(areaM2(f.boundary)).toBeCloseTo(10_000, -1);
  });
});
