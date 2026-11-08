"""Robot settings that the user changes on the dashboard's Settings page.

Each setting is described once, in SETTINGS. The Settings page builds its form from these
descriptions, which robot_status publishes on /r5/settings. To add a setting, add it here and read
it where it's needed, from the values on /r5/settings. The page needs no change.

The values are kept in <data_dir>/settings.json. A value that's missing or no longer valid there
falls back to its default.
"""

from __future__ import annotations

import json
import math
import zoneinfo
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from .auth import write_private_file

# 0.6 m/s: linear.x.max_velocity in r5's diffbot_controllers.yaml and Nav2's desired_linear_vel.
HARDWARE_MAX_SPEED_KMH = 2.16

COMMS_CONTINUE = "continue"
COMMS_RETURN_HOME = "return_home"
COMMS_DELAYED_RETURN_HOME = "delayed_return_home"
COMMS_STOP = "stop"

BATTERY_WARN = "warn"
BATTERY_RETURN_HOME = "return_home"
BATTERY_STOP = "stop"

UNITS_METRIC = "metric"
UNITS_US = "us"

DEVICE_TIME_ZONE = "device"

# job_manager starts its job status messages with these when it stops or goes home by itself.
# robot_status raises an alert for each while the job waits for Resume.
LINK_LOST = "The link to the dashboard was lost"
BATTERY_LOW = "The battery is low"
TOO_STEEP = "The robot is on a slope steeper than the limit"


@lru_cache(maxsize=1)
def time_zone_choices() -> tuple[tuple[str, str], ...]:
    """IANA time zones, as browsers know them, after "this device's time zone"."""
    try:
        zones = zoneinfo.available_timezones()
    except Exception:  # noqa: BLE001 - no time zone database: offer only UTC
        zones = set()
    names = sorted(z for z in zones if "/" in z and not z.startswith(("Etc/", "SystemV/", "posix/", "right/")))
    return ((DEVICE_TIME_ZONE, "This device's time zone"), ("UTC", "UTC"), *((z, z.replace("_", " ")) for z in names))


@dataclass(frozen=True)
class Setting:
    key: str
    label: str
    group: str
    kind: str  # "number", "text" or "choice"
    default: float | str
    help: str = ""
    unit: str = ""
    minimum: float | None = None
    maximum: float | None = None
    step: float | None = None
    max_length: int | None = None
    # (value, label) pairs, for kind "choice".
    choices: tuple[tuple[str, str], ...] = ()
    # (key, value): the page only shows this setting while that setting has that value.
    shown_when: tuple[str, str] | None = None
    # "speed" for a value in km/h that the page shows in the chosen units. The value, minimum and
    # maximum are always stored in km/h.
    quantity: str = ""

    def to_json(self) -> dict:
        out = {
            "key": self.key,
            "label": self.label,
            "group": self.group,
            "kind": self.kind,
            "default": self.default,
            "help": self.help,
            "unit": self.unit,
        }
        if self.minimum is not None:
            out["minimum"] = self.minimum
        if self.maximum is not None:
            out["maximum"] = self.maximum
        if self.step is not None:
            out["step"] = self.step
        if self.max_length is not None:
            out["max_length"] = self.max_length
        if self.choices:
            out["choices"] = [{"value": v, "label": label} for v, label in self.choices]
        if self.shown_when:
            out["shown_when"] = {"key": self.shown_when[0], "value": self.shown_when[1]}
        if self.quantity:
            out["quantity"] = self.quantity
        return out


SETTINGS: tuple[Setting, ...] = (
    Setting(
        key="robot_name",
        label="Nickname",
        group="Robot",
        kind="text",
        default="R5",
        max_length=20,
        help="Shown at the top of the screen and next to the robot on the map.",
    ),
    Setting(
        key="units",
        label="Units",
        group="Display",
        kind="choice",
        default=UNITS_METRIC,
        choices=((UNITS_METRIC, "Metric (km/h, hectares, litres)"), (UNITS_US, "US (mph, acres, US gallons, feet)")),
        help="For speeds, areas, volumes and distances on the dashboard. Sensor readings stay in SI units.",
    ),
    Setting(
        key="time_zone",
        label="Time zone",
        group="Display",
        kind="choice",
        default=DEVICE_TIME_ZONE,
        choices=time_zone_choices(),
        help="For times and dates on the dashboard.",
    ),
    Setting(
        key="max_speed_kmh",
        label="Maximum speed",
        group="Driving",
        kind="number",
        default=HARDWARE_MAX_SPEED_KMH,
        unit="km/h",
        minimum=0.1,
        maximum=HARDWARE_MAX_SPEED_KMH,
        step=0.01,
        quantity="speed",
        help="No part of a spray job goes faster than this. The Xbox controller has its own speeds.",
    ),
    Setting(
        key="transit_speed_kmh",
        label="Transit speed",
        group="Driving",
        kind="number",
        default=HARDWARE_MAX_SPEED_KMH,
        unit="km/h",
        minimum=0.1,
        maximum=HARDWARE_MAX_SPEED_KMH,
        step=0.01,
        quantity="speed",
        help="Driving to the field, home and back, with the sprayer off. Never more than the maximum speed.",
    ),
    Setting(
        key="turn_speed_kmh",
        label="Turn speed",
        group="Driving",
        kind="number",
        default=0.5,
        unit="km/h",
        minimum=0.1,
        maximum=HARDWARE_MAX_SPEED_KMH,
        step=0.01,
        quantity="speed",
        help="Turning between rows. Never more than the maximum speed.",
    ),
    Setting(
        key="comms_loss_action",
        label="If the link is lost",
        group="Link loss",
        kind="choice",
        default=COMMS_CONTINUE,
        choices=(
            (COMMS_CONTINUE, "Keep working"),
            (COMMS_RETURN_HOME, "Return home"),
            (COMMS_DELAYED_RETURN_HOME, "Keep working, then return home"),
            (COMMS_STOP, "Stop and wait"),
        ),
        help=(
            "What a running spray job does when no dashboard has been connected for the time below. "
            "The sprayer is off whenever the robot stops or drives home. After returning home or "
            "stopping, the job waits for Resume."
        ),
    ),
    Setting(
        key="comms_loss_timeout_s",
        label="Link counts as lost after",
        group="Link loss",
        kind="number",
        default=30.0,
        unit="s",
        minimum=15.0,
        maximum=600.0,
        step=1.0,
        help=(
            "Shorter dropouts are ignored. It takes about 15 s to notice a dashboard has gone. "
            "A phone with its screen locked usually counts as disconnected."
        ),
    ),
    Setting(
        key="comms_loss_delay_min",
        label="Return home after",
        group="Link loss",
        kind="number",
        default=5.0,
        unit="min",
        minimum=1.0,
        maximum=120.0,
        step=1.0,
        shown_when=("comms_loss_action", COMMS_DELAYED_RETURN_HOME),
        help="How long the robot keeps working without a link before it drives home.",
    ),
    Setting(
        key="low_battery_action",
        label="If the battery is low",
        group="Battery",
        kind="choice",
        default=BATTERY_RETURN_HOME,
        choices=(
            (BATTERY_WARN, "Only warn"),
            (BATTERY_RETURN_HOME, "Return home"),
            (BATTERY_STOP, "Stop and wait"),
        ),
        help=(
            "What a running spray job does when the battery drops below the level below. There's "
            "always an alert. Resume needs the battery above the level again. Nothing happens until "
            "the robot reports its battery level."
        ),
    ),
    Setting(
        key="low_battery_percent",
        label="Low battery level",
        group="Battery",
        kind="number",
        default=20.0,
        unit="%",
        minimum=5.0,
        maximum=80.0,
        step=1.0,
        help="Counts as low after 10 s below this, so a short dip under load doesn't.",
    ),
    Setting(
        key="slope_limit_deg",
        label="Slope limit",
        group="Slope",
        kind="number",
        default=15.0,
        unit="°",
        minimum=5.0,
        maximum=45.0,
        step=1.0,
        help=(
            "A spray job stops if the robot tilts more than this for a second, from the IMU. After "
            "Resume it doesn't stop again until it has been on level ground, so it can drive off."
        ),
    ),
)

BY_KEY = {s.key: s for s in SETTINGS}


class SettingsError(ValueError):
    """A value the user tried to set isn't allowed. The message is shown to them."""


def defaults() -> dict:
    return {s.key: s.default for s in SETTINGS}


def schema_json() -> str:
    return json.dumps([s.to_json() for s in SETTINGS])


def check_value(setting: Setting, value) -> float | str:
    """The value to store, or SettingsError with a message for the user."""
    if setting.kind == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise SettingsError(f"{setting.label} must be a number.")
        value = float(value)
        unit = f" {setting.unit}" if setting.unit else ""
        if setting.minimum is not None and value < setting.minimum:
            raise SettingsError(f"{setting.label} must be at least {setting.minimum:g}{unit}.")
        if setting.maximum is not None and value > setting.maximum:
            raise SettingsError(f"{setting.label} can be at most {setting.maximum:g}{unit}.")
        return value
    if setting.kind == "text":
        if not isinstance(value, str):
            raise SettingsError(f"{setting.label} must be text.")
        value = " ".join(value.split())  # also removes tabs and line breaks
        if not value:
            raise SettingsError(f"{setting.label} can't be empty.")
        if setting.max_length is not None and len(value) > setting.max_length:
            raise SettingsError(f"{setting.label} can be at most {setting.max_length} characters.")
        return value
    if setting.kind == "choice":
        if value not in {v for v, _ in setting.choices}:
            raise SettingsError(f"{value!r} isn't one of the choices for {setting.label}.")
        return value
    raise SettingsError(f"{setting.label} has an unknown kind {setting.kind!r}.")


def check_changes(changes) -> dict:
    """Checks every value first, so a bad one means nothing is changed."""
    if not isinstance(changes, dict):
        raise SettingsError("Settings must be sent as a JSON object.")
    out = {}
    for key, value in changes.items():
        setting = BY_KEY.get(key)
        if setting is None:
            raise SettingsError(f"There's no setting called {key!r}.")
        out[key] = check_value(setting, value)
    return out


def values_from_json(text: str) -> dict:
    """Values from /r5/settings, with defaults for anything missing. For the nodes that use them."""
    values = defaults()
    try:
        loaded = json.loads(text) if text else {}
    except json.JSONDecodeError:
        return values
    if isinstance(loaded, dict):
        for key, value in loaded.items():
            if key in BY_KEY:
                try:
                    values[key] = check_value(BY_KEY[key], value)
                except SettingsError:
                    pass
    return values


class SettingsStore:
    def __init__(self, data_dir: Path):
        self.path = Path(data_dir) / "settings.json"

    def load(self) -> tuple[dict, list[str]]:
        """The values, and the keys whose saved value was ignored because it's no longer valid."""
        values = defaults()
        ignored: list[str] = []
        try:
            saved = json.loads(self.path.read_text())
        except FileNotFoundError:
            return values, ignored
        except (OSError, json.JSONDecodeError):
            return values, [str(self.path)]
        if not isinstance(saved, dict):
            return values, [str(self.path)]
        for key, value in saved.items():
            setting = BY_KEY.get(key)
            if setting is None:
                continue  # A setting that has since been removed.
            try:
                values[key] = check_value(setting, value)
            except SettingsError:
                ignored.append(key)
        return values, ignored

    def update(self, current: dict, changes: dict) -> dict:
        """Checks and saves the changes. Returns the new values. Raises SettingsError or OSError."""
        values = {**current, **check_changes(changes)}
        write_private_file(self.path, json.dumps(values, indent=2) + "\n")
        return values
