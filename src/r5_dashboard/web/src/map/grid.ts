import type { FeatureCollection, LineString } from 'geojson';

/** Grid spacing in metres for a zoom level, or null when too zoomed out to be useful. */
export function gridSpacingM(zoom: number): number | null {
  if (zoom >= 18) return 10;
  if (zoom >= 16) return 50;
  if (zoom >= 14) return 100;
  return null;
}

/**
 * A metric grid over the view, so fields, the robot and paths still make sense without map tiles
 * (no internet, spec NF-11).
 */
export function gridLines(
  bounds: { west: number; south: number; east: number; north: number },
  zoom: number,
): FeatureCollection<LineString> {
  const spacing = gridSpacingM(zoom);
  const features: FeatureCollection<LineString>['features'] = [];
  if (spacing !== null) {
    const midLat = (bounds.north + bounds.south) / 2;
    const dLat = spacing / 111_320;
    const dLon = spacing / (111_320 * Math.cos((midLat * Math.PI) / 180));
    const lat0 = Math.floor(bounds.south / dLat) * dLat;
    const lon0 = Math.floor(bounds.west / dLon) * dLon;
    for (let lat = lat0; lat <= bounds.north + dLat && features.length < 400; lat += dLat)
      features.push({ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[bounds.west, lat], [bounds.east, lat]] } });
    for (let lon = lon0; lon <= bounds.east + dLon && features.length < 800; lon += dLon)
      features.push({ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[lon, bounds.south], [lon, bounds.north]] } });
  }
  return { type: 'FeatureCollection', features };
}
