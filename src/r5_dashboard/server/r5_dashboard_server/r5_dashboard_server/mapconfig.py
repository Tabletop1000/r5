"""Map tile sources for the dashboard (spec NF-11, NF-12).

The default is the OpenStreetMap street map. To add open aerial imagery for your area, write
<data_dir>/map.json, for example:

{
  "layers": [
    {"id": "osm", "name": "Street map", "tiles": ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
     "maxzoom": 19, "attribution": "© OpenStreetMap contributors"},
    {"id": "aerial", "name": "Aerial", "tiles": ["https://example.org/tiles/{z}/{x}/{y}.jpg"],
     "maxzoom": 20, "attribution": "Imagery © its provider, licence ..."}
  ]
}

Only use imagery whose licence allows it. The browser is only allowed to load tiles from the hosts
named here (they're added to the page's Content-Security-Policy). With no internet, the map shows
a plain grid instead.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from urllib.parse import urlsplit

log = logging.getLogger("r5_dashboard_server")

DEFAULT_CONFIG = {
    "layers": [
        {
            "id": "osm",
            "name": "Street map",
            "tiles": ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
            "tileSize": 256,
            "maxzoom": 19,
            "attribution": "© OpenStreetMap contributors",
        }
    ]
}


class MapConfigError(ValueError):
    pass


def _check(config: dict) -> dict:
    layers = config.get("layers")
    if not isinstance(layers, list):
        raise MapConfigError("'layers' must be a list")
    clean = []
    for i, layer in enumerate(layers):
        try:
            tiles = [str(t) for t in layer["tiles"]]
            entry = {
                "id": str(layer.get("id", f"layer{i}")),
                "name": str(layer.get("name", f"Layer {i + 1}")),
                "tiles": tiles,
                "tileSize": int(layer.get("tileSize", 256)),
                "maxzoom": int(layer.get("maxzoom", 19)),
                "attribution": str(layer.get("attribution", "")),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise MapConfigError(f"layer {i}: {exc}") from exc
        for t in tiles:
            if urlsplit(t).scheme != "https" or not all(k in t for k in ("{z}", "{x}", "{y}")):
                raise MapConfigError(f"layer {i}: tile URL must be https and contain {{z}}, {{x}}, {{y}}: {t}")
        clean.append(entry)
    return {"layers": clean}


def load_map_config(data_dir: Path) -> dict:
    path = data_dir / "map.json"
    if not path.is_file():
        return DEFAULT_CONFIG
    try:
        return _check(json.loads(path.read_text()))
    except (OSError, ValueError) as exc:
        log.error("Ignoring %s (%s); using the default street map.", path, exc)
        return DEFAULT_CONFIG


def tile_origins(config: dict) -> list[str]:
    """https://host[:port] for every tile source, for the Content-Security-Policy."""
    origins = []
    for layer in config["layers"]:
        for t in layer["tiles"]:
            parts = urlsplit(t.replace("{z}", "0").replace("{x}", "0").replace("{y}", "0"))
            origin = f"{parts.scheme}://{parts.netloc}"
            if origin not in origins:
                origins.append(origin)
    return origins
