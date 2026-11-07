"""Quantities in the text the robot sends to the dashboard, in the units chosen in Settings.

The robot works in metric throughout. Only text meant for the user is converted. The web app does
the same for everything else it shows (web/src/display/units.ts).
"""

from __future__ import annotations

from .settings import UNITS_US

KM_PER_MILE = 1.609344
L_PER_US_GALLON = 3.785411784
M2_PER_ACRE = 4046.8564224
FT_PER_M = 1 / 0.3048


def format_speed(kmh: float, units: str) -> str:
    if units == UNITS_US:
        return f"{kmh / KM_PER_MILE:.2f} mph"
    return f"{kmh:.2f} km/h"


def format_area(m2: float, units: str) -> str:
    if units == UNITS_US:
        acres = m2 / M2_PER_ACRE
        return f"{acres:.2f} acres" if acres >= 0.1 else f"{m2 * FT_PER_M**2:,.0f} ft²"
    return f"{m2 / 10_000:.2f} ha" if m2 >= 1000 else f"{m2:.0f} m²"


def format_volume(litres: float, units: str) -> str:
    if units == UNITS_US:
        return f"{litres / L_PER_US_GALLON:.1f} gal"
    return f"{litres:.1f} L"


def format_flow(l_min: float, units: str) -> str:
    if units == UNITS_US:
        return f"{l_min / L_PER_US_GALLON:.2f} gal/min"
    return f"{l_min:.1f} L/min"
