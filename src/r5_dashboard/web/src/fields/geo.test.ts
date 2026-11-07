import { describe, expect, it } from 'vitest';
import type { LatLon } from '../ros/topics';
import { areaM2, checkField, formatArea, fromRing, LocalProjection, ringSelfIntersects, toRing, type FieldDraft } from './geo';
import { innerArea } from './headland';
import { gridLines, gridSpacingM } from '../map/grid';

const proj = new LocalProjection(-33.86, 151.21);
const ll = (pts: [number, number][]): LatLon[] => pts.map((p) => proj.toLatLon(p));
const square = ll([[0, 0], [100, 0], [100, 100], [0, 100]]);

const draft = (over: Partial<FieldDraft> = {}): FieldDraft => ({
  id: '',
  name: 'East field',
  boundary: square,
  keepOut: [],
  headlandWidthM: 2,
  source: 'drawn',
  notes: '',
  ...over,
});

describe('geometry', () => {
  it('computes area in m²', () => {
    expect(areaM2(square)).toBeCloseTo(10_000, -1);
    expect(formatArea(10_000)).toBe('1.00 ha');
    expect(formatArea(400)).toBe('400 m²');
  });
  it('detects a crossed boundary', () => {
    expect(ringSelfIntersects([[0, 0], [10, 10], [10, 0], [0, 10]])).toBe(true);
    expect(ringSelfIntersects([[0, 0], [10, 0], [10, 10], [0, 10]])).toBe(false);
  });
  it('converts GeoJSON rings both ways', () => {
    const ring = toRing(square);
    expect(ring).toHaveLength(5);
    expect(ring[0]).toEqual(ring[4]);
    const back = fromRing(ring);
    expect(back).toHaveLength(4);
    back.forEach((p, i) => {
      // Rounded to 9 decimal places (about 0.1 mm) for terra-draw.
      expect(p.latitude).toBeCloseTo(square[i].latitude, 9);
      expect(p.longitude).toBeCloseTo(square[i].longitude, 9);
    });
  });
});

describe('checkField (mirrors the robot, spec FM-6)', () => {
  it('accepts a good field', () => {
    const c = checkField(draft({ keepOut: [ll([[40, 40], [60, 40], [60, 60]])] }), []);
    expect(c.errors).toEqual([]);
    expect(c.warnings).toEqual([]);
  });
  it.each([
    [{ name: '' }, 'name'],
    [{ boundary: square.slice(0, 2) }, 'at least 3'],
    [{ boundary: ll([[0, 0], [100, 100], [100, 0], [0, 100]]) }, 'crosses itself'],
    [{ keepOut: [ll([[90, 90], [120, 90], [120, 120]])] }, 'inside the boundary'],
    [{ keepOut: [ll([[10, 10], [30, 10], [30, 30]]), ll([[20, 20], [40, 20], [40, 40]])] }, 'overlap'],
    [{ headlandWidthM: 30 }, 'headland'],
    [{ headlandWidthM: NaN }, 'headland'],
  ] as [Partial<FieldDraft>, string][])('rejects %o', (over, reason) => {
    expect(checkField(draft(over), []).errors.join(' ')).toMatch(reason);
  });
  it('rejects a duplicate name, ignoring case', () => {
    expect(checkField(draft({ name: 'EAST field' }), ['East field']).errors[0]).toMatch('already');
  });
  it('warns, without blocking, about unusual sizes', () => {
    const small = checkField(draft({ boundary: ll([[0, 0], [20, 0], [20, 20], [0, 20]]) }), []);
    expect(small.errors).toEqual([]);
    expect(small.warnings[0]).toMatch('0.10 ha to 3.00 ha');
    const us = checkField(draft({ boundary: ll([[0, 0], [20, 0], [20, 20], [0, 20]]) }), [], 'us');
    expect(us.warnings[0]).toMatch('0.25 acres to 7.41 acres');
  });
});

describe('headland', () => {
  it('shrinks the field by the headland width', () => {
    const inner = innerArea(square, 10)!;
    const ring = (inner.geometry.coordinates[0] as number[][]).map(([lon, lat]) => ({ latitude: lat, longitude: lon }));
    expect(areaM2(ring)).toBeGreaterThan(80 * 80 * 0.97);
    expect(areaM2(ring)).toBeLessThan(80 * 80 * 1.03);
  });
  it('returns the whole field for zero width, and null when it swallows it', () => {
    expect(innerArea(square, 0)).not.toBeNull();
    expect(innerArea(square, 60)).toBeNull();
  });
});

describe('grid', () => {
  it('picks spacing by zoom and stays bounded', () => {
    expect(gridSpacingM(12)).toBeNull();
    expect(gridSpacingM(18)).toBe(10);
    const g = gridLines({ west: 151.2, south: -33.87, east: 151.21, north: -33.86 }, 16);
    expect(g.features.length).toBeGreaterThan(10);
    expect(g.features.length).toBeLessThanOrEqual(800);
  });
});
