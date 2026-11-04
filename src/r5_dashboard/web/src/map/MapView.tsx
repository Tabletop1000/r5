import { AttributionControl, Map as MlMap, NavigationControl, ScaleControl, setWorkerUrl, type StyleSpecification } from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
// MapLibre works out its worker's address at run time, which the bundler can't see, so the
// worker file is added to the build explicitly.
import workerUrl from 'maplibre-gl/dist/maplibre-gl-worker.mjs?url';
import { createContext, useContext, useEffect, useRef, useState, type ReactNode } from 'react';
import { getMapConfig, type MapConfig } from '../api';
import { gridLines } from './grid';

setWorkerUrl(workerUrl);

const MapContext = createContext<MlMap | null>(null);

/** The map, for components drawn on it. Only available inside <MapView>. */
export function useMap(): MlMap {
  const map = useContext(MapContext);
  if (!map) throw new Error('useMap must be used inside <MapView>.');
  return map;
}

function buildStyle(config: MapConfig, dark: boolean): StyleSpecification {
  const style: StyleSpecification = {
    version: 8,
    sources: { grid: { type: 'geojson', data: { type: 'FeatureCollection', features: [] } } },
    layers: [
      { id: 'background', type: 'background', paint: { 'background-color': dark ? '#1f251f' : '#e8ebe3' } },
      { id: 'grid', type: 'line', source: 'grid', paint: { 'line-color': dark ? '#3a443a' : '#c9cfc1', 'line-width': 1 } },
    ],
  };
  config.layers.forEach((layer, i) => {
    style.sources[`base-${layer.id}`] = {
      type: 'raster',
      tiles: layer.tiles,
      tileSize: layer.tileSize,
      maxzoom: layer.maxzoom,
      attribution: layer.attribution,
    };
    style.layers.push({
      id: `base-${layer.id}`,
      type: 'raster',
      source: `base-${layer.id}`,
      layout: { visibility: i === 0 ? 'visible' : 'none' },
    });
  });
  return style;
}

interface Props {
  children?: ReactNode;
  className?: string;
  /** Extra controls drawn over the map's top-left corner. */
  overlay?: ReactNode;
}

/**
 * Base map: tiles from the robot's map config over a metric grid, which is all that shows when
 * there's no internet (spec NF-11). Children add fields, markers and editing on top.
 */
export function MapView({ children, className, overlay }: Props) {
  const container = useRef<HTMLDivElement>(null);
  const [map, setMap] = useState<MlMap | null>(null);
  const [config, setConfig] = useState<MapConfig | null>(null);
  const [baseLayer, setBaseLayer] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    void getMapConfig().then((c) => !cancelled && setConfig(c));
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    if (!config || !container.current) return;
    const dark = window.matchMedia?.('(prefers-color-scheme: dark)').matches ?? false;
    const m = new MlMap({
      container: container.current,
      style: buildStyle(config, dark),
      center: [0, 20],
      zoom: 1.5,
      maxZoom: 21,
      attributionControl: false,
      dragRotate: false,
      pitchWithRotate: false,
    });
    m.touchZoomRotate.disableRotation();
    m.addControl(new NavigationControl({ showCompass: false }), 'top-right');
    m.addControl(new ScaleControl({ unit: 'metric' }), 'bottom-left');
    m.addControl(new AttributionControl({ compact: true }), 'bottom-right');

    const updateGrid = () => {
      const b = m.getBounds();
      const source = m.getSource('grid');
      if (source && 'setData' in source)
        (source as { setData: (d: unknown) => void }).setData(
          gridLines({ west: b.getWest(), south: b.getSouth(), east: b.getEast(), north: b.getNorth() }, m.getZoom()),
        );
    };
    m.on('moveend', updateGrid);
    m.on('load', () => {
      updateGrid();
      setMap(m);
    });
    const resize = new ResizeObserver(() => m.resize());
    resize.observe(container.current);
    setBaseLayer(config.layers[0]?.id ?? null);
    return () => {
      resize.disconnect();
      setMap(null);
      // React cleans up this component before the layers drawn on the map, so the map is removed
      // a moment later, once they have taken their sources and layers off it.
      setTimeout(() => m.remove(), 0);
    };
  }, [config]);

  function chooseBase(id: string) {
    if (!map || !config) return;
    for (const l of config.layers) map.setLayoutProperty(`base-${l.id}`, 'visibility', l.id === id ? 'visible' : 'none');
    setBaseLayer(id);
  }

  return (
    <div className={`map-view${className ? ` ${className}` : ''}`}>
      <div ref={container} className="map-canvas" />
      {(overlay || (config && config.layers.length > 1)) && (
        <div className="map-overlay">
          {config && config.layers.length > 1 && (
            <select aria-label="Base map" value={baseLayer ?? ''} onChange={(e) => chooseBase(e.target.value)}>
              {config.layers.map((l) => (
                <option key={l.id} value={l.id}>
                  {l.name}
                </option>
              ))}
            </select>
          )}
          {overlay}
        </div>
      )}
      {map && <MapContext.Provider value={map}>{children}</MapContext.Provider>}
    </div>
  );
}
