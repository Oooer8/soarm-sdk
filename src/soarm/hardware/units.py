from __future__ import annotations

import math

from ..constants import (
    DEFAULT_POSITION_ACCELERATION,
    DEFAULT_POSITION_SPEED,
    MAX_SPEED_SETTING,
    POSITION_TICKS_PER_REVOLUTION,
)

SPEED_UNIT_RPM = 0.732
ACCELERATION_UNIT_DEG_S2 = 8.7


def rad_to_deg(value: float) -> float:
    return value * 180.0 / math.pi


def deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0


def rad_to_ticks_delta(value: float) -> float:
    return value / (2.0 * math.pi) * POSITION_TICKS_PER_REVOLUTION


def ticks_delta_to_rad(value: float) -> float:
    return value / POSITION_TICKS_PER_REVOLUTION * 2.0 * math.pi


def speed_setting_from_rad_s(value: float | None) -> int:
    if value is None:
        return DEFAULT_POSITION_SPEED
    if value <= 0:
        return DEFAULT_POSITION_SPEED
    rpm = value * 60.0 / (2.0 * math.pi)
    setting = int(round(rpm / SPEED_UNIT_RPM))
    return max(1, min(MAX_SPEED_SETTING, setting))


def acceleration_setting_from_rad_s2(value: float | None) -> int:
    if value is None:
        return DEFAULT_POSITION_ACCELERATION
    if value <= 0:
        return DEFAULT_POSITION_ACCELERATION
    deg_s2 = rad_to_deg(value)
    return max(1, int(round(deg_s2 / ACCELERATION_UNIT_DEG_S2)))
