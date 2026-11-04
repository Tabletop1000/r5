"""Usage counters, service schedule and alert tracking (spec 4.4, RB-2, RB-4, 6.4).

Plain Python, without ROS, so it can be unit-tested. robot_status_node.py connects it to ROS.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from .auth import write_private_file

# Odometry from the local EKF wobbles slightly while standing still. Distance is only added once
# the robot has moved this far from the last counted point, so the wobble isn't counted.
MIN_STEP_M = 0.05
# A bigger jump between two messages is a reset or a glitch, not driving.
MAX_STEP_M = 2.0
# Longer gaps between operating-time ticks (a suspended computer, a clock change) aren't counted.
MAX_TICK_S = 10.0


@dataclass
class Odometer:
    """Counts distance driven from a stream of (x, y) positions."""

    distance_m: float = 0.0
    _anchor: tuple[float, float] | None = None

    def update(self, x: float, y: float) -> None:
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        if self._anchor is None:
            self._anchor = (x, y)
            return
        step = math.hypot(x - self._anchor[0], y - self._anchor[1])
        if step > MAX_STEP_M:
            self._anchor = (x, y)
        elif step >= MIN_STEP_M:
            self.distance_m += step
            self._anchor = (x, y)


@dataclass
class ServiceRecord:
    time_s: float
    distance_m: float


@dataclass
class UsageStats:
    """Everything that must survive a restart. Saved to stats.json."""

    operating_s: float = 0.0
    odometer: Odometer = field(default_factory=Odometer)
    last_service: ServiceRecord | None = None

    def add_operating_time(self, dt_s: float) -> None:
        if 0.0 < dt_s <= MAX_TICK_S:
            self.operating_s += dt_s

    def to_json(self) -> dict:
        return {
            "operating_s": self.operating_s,
            "distance_m": self.odometer.distance_m,
            "last_service": None
            if self.last_service is None
            else {"time_s": self.last_service.time_s, "distance_m": self.last_service.distance_m},
        }

    @classmethod
    def from_json(cls, data: dict) -> UsageStats:
        stats = cls(
            operating_s=float(data.get("operating_s", 0.0)),
            odometer=Odometer(distance_m=float(data.get("distance_m", 0.0))),
        )
        last = data.get("last_service")
        if last:
            stats.last_service = ServiceRecord(float(last["time_s"]), float(last["distance_m"]))
        return stats


class StatsStore:
    """Loads and saves UsageStats, and appends to the service log."""

    def __init__(self, data_dir: Path):
        self.stats_path = data_dir / "stats.json"
        self.service_log_path = data_dir / "service_log.json"

    def load(self) -> UsageStats:
        try:
            return UsageStats.from_json(json.loads(self.stats_path.read_text()))
        except FileNotFoundError:
            return UsageStats()
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            # Keep the damaged file for inspection rather than overwrite it.
            backup = self.stats_path.with_suffix(".damaged.json")
            self.stats_path.replace(backup)
            raise StatsFileDamaged(backup) from exc

    def save(self, stats: UsageStats) -> None:
        write_private_file(self.stats_path, json.dumps(stats.to_json(), indent=2) + "\n")

    def log_service(self, record: ServiceRecord) -> None:
        try:
            log = json.loads(self.service_log_path.read_text())
            if not isinstance(log, list):
                log = []
        except (OSError, ValueError):
            log = []
        log.append({"time_s": record.time_s, "distance_km": record.distance_m / 1000.0})
        write_private_file(self.service_log_path, json.dumps(log, indent=2) + "\n")


class StatsFileDamaged(Exception):
    def __init__(self, backup: Path):
        super().__init__(f"stats.json couldn't be read and was moved to {backup}. Counters restart at zero.")
        self.backup = backup


@dataclass
class ServiceStatus:
    km_to_service: float
    due_soon: bool
    overdue: bool


def service_status(stats: UsageStats, interval_km: float, warn_km: float) -> ServiceStatus:
    """Service is due every interval_km of driving, counted from the last service (or from zero)."""
    since_m = stats.last_service.distance_m if stats.last_service else 0.0
    driven_km = (stats.odometer.distance_m - since_m) / 1000.0
    remaining = interval_km - driven_km
    return ServiceStatus(
        km_to_service=remaining, due_soon=0.0 < remaining <= warn_km, overdue=remaining <= 0.0
    )


# ---- Alerts ----

INFO, WARNING, ERROR = 0, 1, 2

# diagnostic_msgs/DiagnosticStatus levels.
DIAG_OK, DIAG_WARN, DIAG_ERROR, DIAG_STALE = 0, 1, 2, 3


@dataclass
class AlertState:
    id: str
    severity: int
    source: str
    message: str
    since_s: float
    acknowledged: bool = False


@dataclass
class DiagnosticEntry:
    name: str
    level: int
    message: str


def leaf_entries(entries: list[DiagnosticEntry]) -> list[DiagnosticEntry]:
    """Drop aggregator group entries ("/Sensors") that only repeat their members ("/Sensors/IMU")."""
    names = [e.name for e in entries]
    return [e for e in entries if not any(n != e.name and n.startswith(e.name + "/") for n in names)]


def _source_name(diag_name: str) -> str:
    return diag_name.rstrip("/").rsplit("/", 1)[-1] or diag_name


class AlertTracker:
    """Turns diagnostics and the robot's own checks into a list of active alerts.

    An alert keeps its start time and acknowledgement while it stays active. It is dropped when the
    problem clears. If it gets worse (warning to error) it needs acknowledging again.
    """

    def __init__(self):
        self._alerts: dict[str, AlertState] = {}

    @property
    def alerts(self) -> list[AlertState]:
        return sorted(self._alerts.values(), key=lambda a: (-a.severity, a.since_s, a.id))

    def _set(self, id: str, severity: int, source: str, message: str, now_s: float) -> bool:
        current = self._alerts.get(id)
        if current is None:
            self._alerts[id] = AlertState(id, severity, source, message, now_s)
            return True
        changed = current.severity != severity or current.message != message
        if severity > current.severity:
            current.acknowledged = False
        current.severity = severity
        current.message = message
        return changed

    def _clear(self, id: str) -> bool:
        return self._alerts.pop(id, None) is not None

    def update_diagnostics(self, entries: list[DiagnosticEntry], now_s: float) -> bool:
        """Replace all diagnostics-based alerts. Returns True if anything changed."""
        changed = False
        active: set[str] = set()
        for entry in leaf_entries(entries):
            if entry.level == DIAG_OK:
                continue
            severity = ERROR if entry.level == DIAG_ERROR else WARNING
            message = entry.message or ("No recent data" if entry.level == DIAG_STALE else "Problem reported")
            if entry.level == DIAG_STALE and entry.message:
                message = f"No recent data ({entry.message})"
            id = f"diag:{entry.name}"
            active.add(id)
            changed |= self._set(id, severity, _source_name(entry.name), message, now_s)
        for id in [a for a in self._alerts if a.startswith("diag:") and a not in active]:
            changed |= self._clear(id)
        return changed

    def set_condition(
        self, id: str, active: bool, severity: int, source: str, message: str, now_s: float
    ) -> bool:
        """Raise or clear one of the robot's own alerts. Returns True if anything changed."""
        if active:
            return self._set(id, severity, source, message, now_s)
        return self._clear(id)

    def acknowledge(self, id: str) -> bool:
        alert = self._alerts.get(id)
        if alert is None:
            return False
        alert.acknowledged = True
        return True
