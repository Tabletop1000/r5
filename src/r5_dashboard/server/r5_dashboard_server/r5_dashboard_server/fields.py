"""Field storage, validation and boundary recording (spec 4.1, 6.3, FM-4, FM-6, FM-8, FM-10).

Plain Python, no ROS, so it can be unit-tested. field_manager_node.py connects it to ROS.

Each field is one GeoJSON file, fields/<id>.geojson: a FeatureCollection with the boundary as the
first feature (carrying the field's properties) and one feature per keep-out zone. The files open
directly in QGIS.
"""

from __future__ import annotations

import calendar
import json
import math
import re
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from .auth import write_private_file
from .geo import (
    LatLon,
    LocalProjection,
    close_track,
    path_length_m,
    ring_area_m2,
    ring_inside,
    ring_self_intersects,
    rings_cross,
)

MAX_NAME_LENGTH = 60
MAX_HEADLAND_M = 20.0
DEFAULT_HEADLAND_M = 2.0
SOURCES = ("drawn", "recorded", "imported")
_ID_PATTERN = re.compile(r"^[0-9a-f-]{8,64}$")


@dataclass
class Field:
    id: str
    name: str
    boundary: list[LatLon]
    keep_out: list[list[LatLon]] = field(default_factory=list)
    headland_width_m: float = DEFAULT_HEADLAND_M
    source: str = "drawn"
    notes: str = ""
    area_m2: float = 0.0
    created_s: float = 0.0
    modified_s: float = 0.0

    # ---- GeoJSON ----

    def to_geojson(self) -> dict:
        def ring(points: list[LatLon]) -> list[list[float]]:
            coords = [[lon, lat] for lat, lon in points]
            return coords + [coords[0]]  # GeoJSON rings repeat the first point

        features = [
            {
                "type": "Feature",
                "properties": {
                    "role": "boundary",
                    "id": self.id,
                    "name": self.name,
                    "headland_width_m": self.headland_width_m,
                    "source": self.source,
                    "notes": self.notes,
                    "area_m2": round(self.area_m2, 2),
                    "created": _iso(self.created_s),
                    "modified": _iso(self.modified_s),
                },
                "geometry": {"type": "Polygon", "coordinates": [ring(self.boundary)]},
            }
        ]
        for i, zone in enumerate(self.keep_out):
            features.append(
                {
                    "type": "Feature",
                    "properties": {"role": "keep_out", "index": i},
                    "geometry": {"type": "Polygon", "coordinates": [ring(zone)]},
                }
            )
        return {"type": "FeatureCollection", "features": features}

    @classmethod
    def from_geojson(cls, data: dict) -> Field:
        def ring(feature: dict) -> list[LatLon]:
            coords = feature["geometry"]["coordinates"][0]
            if len(coords) > 1 and coords[0] == coords[-1]:
                coords = coords[:-1]
            return [(float(lat), float(lon)) for lon, lat, *_ in coords]

        features = data["features"]
        boundary = next(f for f in features if f.get("properties", {}).get("role") == "boundary")
        props = boundary["properties"]
        return cls(
            id=str(props["id"]),
            name=str(props["name"]),
            boundary=ring(boundary),
            keep_out=[ring(f) for f in features if f.get("properties", {}).get("role") == "keep_out"],
            headland_width_m=float(props.get("headland_width_m", DEFAULT_HEADLAND_M)),
            source=str(props.get("source", "imported")),
            notes=str(props.get("notes", "")),
            area_m2=float(props.get("area_m2", 0.0)),
            created_s=_from_iso(props.get("created")),
            modified_s=_from_iso(props.get("modified")),
        )


def _iso(t: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))


def _from_iso(text) -> float:
    try:
        return float(calendar.timegm(time.strptime(str(text), "%Y-%m-%dT%H:%M:%SZ")))
    except (TypeError, ValueError):
        return 0.0


# ---- Validation (spec FM-6) ----


class FieldInvalid(ValueError):
    pass


def _finite(points: list[LatLon]) -> bool:
    return all(
        math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90 and abs(lon) <= 180
        for lat, lon in points
    )


def field_area_m2(boundary: list[LatLon]) -> float:
    proj = LocalProjection.around(boundary)
    return ring_area_m2([proj.to_xy(p) for p in boundary])


def validate(f: Field, other_names: list[str]) -> None:
    """Raise FieldInvalid with a plain-language reason if the field can't be saved."""
    name = f.name.strip()
    if not name:
        raise FieldInvalid("Give the field a name.")
    if len(name) > MAX_NAME_LENGTH:
        raise FieldInvalid(f"The name is too long (at most {MAX_NAME_LENGTH} characters).")
    if name.lower() in (n.lower() for n in other_names):
        raise FieldInvalid(f"There is already a field called “{name}”.")
    if f.source not in SOURCES:
        raise FieldInvalid(f"Unknown field source “{f.source}”.")
    if not (0.0 <= f.headland_width_m <= MAX_HEADLAND_M) or not math.isfinite(f.headland_width_m):
        raise FieldInvalid(f"The headland width must be between 0 and {MAX_HEADLAND_M:.0f} m.")
    if len(f.boundary) < 3:
        raise FieldInvalid("The boundary needs at least 3 points.")
    if not _finite(f.boundary) or not all(_finite(z) for z in f.keep_out):
        raise FieldInvalid("A point is outside the valid range of latitude and longitude.")

    proj = LocalProjection.around(f.boundary)
    outer = [proj.to_xy(p) for p in f.boundary]
    if ring_self_intersects(outer):
        raise FieldInvalid("The boundary crosses itself.")
    if ring_area_m2(outer) < 1.0:
        raise FieldInvalid("The boundary encloses no area.")
    zones = []
    for i, zone in enumerate(f.keep_out, start=1):
        if len(zone) < 3:
            raise FieldInvalid(f"Keep-out zone {i} needs at least 3 points.")
        xy = [proj.to_xy(p) for p in zone]
        if ring_self_intersects(xy):
            raise FieldInvalid(f"Keep-out zone {i} crosses itself.")
        if not ring_inside(xy, outer):
            raise FieldInvalid(f"Keep-out zone {i} must lie inside the boundary.")
        zones.append(xy)
    for i in range(len(zones)):
        for j in range(i + 1, len(zones)):
            if rings_cross(zones[i], zones[j]):
                raise FieldInvalid(f"Keep-out zones {i + 1} and {j + 1} overlap. Merge them into one.")


# ---- Storage (spec FM-8, 6.2, 6.3) ----


class FieldStore:
    def __init__(self, data_dir: Path, clock=time.time):
        self.dir = data_dir / "fields"
        self.home_path = data_dir / "home.geojson"
        self._clock = clock

    def _path(self, field_id: str) -> Path:
        if not _ID_PATTERN.match(field_id):
            raise FieldInvalid("Unknown field.")
        return self.dir / f"{field_id}.geojson"

    def list(self) -> tuple[list[Field], list[str]]:
        """All readable fields, sorted by name, and the names of files that couldn't be read."""
        fields, problems = [], []
        if self.dir.is_dir():
            for path in sorted(self.dir.glob("*.geojson")):
                try:
                    fields.append(Field.from_geojson(json.loads(path.read_text())))
                except (OSError, ValueError, KeyError, StopIteration, TypeError, IndexError):
                    problems.append(path.name)
        fields.sort(key=lambda f: f.name.lower())
        return fields, problems

    def get(self, field_id: str) -> Field | None:
        try:
            return Field.from_geojson(json.loads(self._path(field_id).read_text()))
        except FileNotFoundError:
            return None

    def save(self, f: Field) -> Field:
        """Validate and write. A new field (empty id) gets an id. Returns the field as stored."""
        existing, _ = self.list()
        if f.id:
            previous = next((e for e in existing if e.id == f.id), None)
            if previous is None:
                raise FieldInvalid("That field no longer exists. It may have been deleted on another device.")
            created = previous.created_s
        else:
            f.id = str(uuid.uuid4())
            created = self._clock()
        f.name = f.name.strip()
        validate(f, [e.name for e in existing if e.id != f.id])
        f.area_m2 = field_area_m2(f.boundary)
        f.created_s = created
        f.modified_s = self._clock()
        write_private_file(self._path(f.id), json.dumps(f.to_geojson(), indent=2) + "\n")
        return f

    def delete(self, field_id: str) -> bool:
        try:
            self._path(field_id).unlink()
            return True
        except FileNotFoundError:
            return False

    # ---- Home (spec FM-10) ----

    def get_home(self) -> LatLon | None:
        try:
            data = json.loads(self.home_path.read_text())
            lon, lat = data["features"][0]["geometry"]["coordinates"][:2]
            return float(lat), float(lon)
        except (OSError, ValueError, KeyError, IndexError, TypeError):
            return None

    def set_home(self, position: LatLon) -> None:
        if not _finite([position]):
            raise FieldInvalid("That position is outside the valid range.")
        lat, lon = position
        data = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"role": "home", "set": _iso(self._clock())},
                    "geometry": {"type": "Point", "coordinates": [lon, lat]},
                }
            ],
        }
        write_private_file(self.home_path, json.dumps(data, indent=2) + "\n")


# ---- Boundary recording (spec FM-4) ----

IDLE, RECORDING, FINISHED = 0, 1, 2

# Fix qualities, worst to best.
FIX_NONE, FIX_GPS, FIX_DGPS, FIX_RTK_FLOAT, FIX_RTK_FIXED = range(5)
FIX_NAMES = {
    FIX_NONE: "No fix",
    FIX_GPS: "3D fix",
    FIX_DGPS: "Corrected (SBAS)",
    FIX_RTK_FLOAT: "RTK float",
    FIX_RTK_FIXED: "RTK fixed",
}
REQUIREMENTS = {"any": FIX_GPS, "rtk_float": FIX_RTK_FLOAT, "rtk_fixed": FIX_RTK_FIXED}


def fix_quality(navsat_status: int, carrier: int | None) -> int:
    """Fix quality from NavSatFix status.status and, when known, the u-blox NavPVT carrier
    solution (0 none, 1 RTK float, 2 RTK fixed)."""
    if navsat_status < 0:
        return FIX_NONE
    if carrier == 2:
        return FIX_RTK_FIXED
    if carrier == 1:
        return FIX_RTK_FLOAT
    if navsat_status == 2:
        # GBAS without NavPVT: corrections are in use, but float and fixed can't be told apart.
        return FIX_RTK_FLOAT if carrier is None else FIX_DGPS
    if navsat_status == 1:
        return FIX_DGPS
    return FIX_GPS


class Recorder:
    """Collects positions while the robot is driven around a field edge.

    Points are kept only when the fix is good enough and the robot has moved min_spacing_m from
    the last kept point. Stopping closes the track and simplifies it to within tolerance_m.
    """

    def __init__(self, required: str = "rtk_fixed", min_spacing_m: float = 0.2, tolerance_m: float = 0.1, clock=time.time):
        if required not in REQUIREMENTS:
            raise ValueError(f"required must be one of {sorted(REQUIREMENTS)}")
        self.required = required
        self.min_quality = REQUIREMENTS[required]
        self.min_spacing_m = min_spacing_m
        self.tolerance_m = tolerance_m
        self._clock = clock
        self.reset()

    def reset(self) -> None:
        self.state = IDLE
        self.points: list[LatLon] = []
        self.skipped = 0
        self.length_m = 0.0
        self.current_quality = FIX_NONE
        self.message = ""
        self.started_s = 0.0
        self._proj: LocalProjection | None = None

    def start(self) -> None:
        self.reset()
        self.state = RECORDING
        self.started_s = self._clock()
        self.message = "Drive slowly around the edge of the field."

    def add_fix(self, position: LatLon, quality: int) -> bool:
        """Offer a position. Returns True if it was kept."""
        self.current_quality = quality
        if self.state != RECORDING:
            return False
        if quality < self.min_quality or not _finite([position]):
            self.skipped += 1
            return False
        if self._proj is None:
            self._proj = LocalProjection(position)
        if self.points:
            step = math.dist(self._proj.to_xy(self.points[-1]), self._proj.to_xy(position))
            if step < self.min_spacing_m:
                return False
            self.length_m += step
        self.points.append(position)
        return True

    def stop(self) -> None:
        """Finish: close and simplify the track. Unusable tracks finish with an explanation."""
        if self.state != RECORDING:
            return
        self.state = FINISHED
        if len(self.points) < 3 or self._proj is None:
            self.message = "Too few positions were recorded to make a boundary."
            self.points = []
            return
        track = [self._proj.to_xy(p) for p in self.points]
        ring = close_track(track, self.tolerance_m)
        if len(ring) < 3 or ring_area_m2(ring) < 1.0:
            self.message = "The recorded track doesn't enclose an area."
            self.points = []
            return
        gap = math.dist(track[0], track[-1])
        self.points = [self._proj.to_latlon(p) for p in ring]
        self.length_m = path_length_m(track)
        self.message = (
            f"Boundary recorded: {len(self.points)} corners, {ring_area_m2(ring) / 10_000:.2f} ha."
            + (f" The end was {gap:.1f} m from the start; the gap was closed with a straight line." if gap > 2.0 else "")
        )
        if ring_self_intersects(ring):
            self.message += " The track crosses itself; fix it in the editor before saving."

    def clear(self) -> None:
        if self.state != RECORDING:
            self.reset()
