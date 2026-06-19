from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Mapping

from ..constants import MAX_POSITION_TICK, MIN_POSITION_TICK, POSITION_TICKS_PER_REVOLUTION
from ..errors import ConfigurationError, LimitViolation
from ..hardware.units import deg_to_rad, rad_to_deg, rad_to_ticks_delta, ticks_delta_to_rad


@dataclass(frozen=True)
class JointConfig:
    name: str
    id: int
    zero_tick: int
    direction: int
    min_rad: float
    max_rad: float
    max_vel_rad_s: float
    max_acc_rad_s2: float

    @classmethod
    def from_mapping(cls, name: str, data: Mapping[str, Any]) -> "JointConfig":
        required = [
            "id",
            "zero_tick",
            "direction",
            "min_rad",
            "max_rad",
            "max_vel_rad_s",
            "max_acc_rad_s2",
        ]
        missing = [key for key in required if key not in data]
        if missing:
            raise ConfigurationError(f"joints.{name} is missing: {', '.join(missing)}")
        joint = cls(
            name=name,
            id=int(data["id"]),
            zero_tick=int(data["zero_tick"]),
            direction=int(data["direction"]),
            min_rad=float(data["min_rad"]),
            max_rad=float(data["max_rad"]),
            max_vel_rad_s=float(data["max_vel_rad_s"]),
            max_acc_rad_s2=float(data["max_acc_rad_s2"]),
        )
        joint.validate()
        return joint

    def validate(self) -> None:
        if self.id <= 0:
            raise ConfigurationError(f"Joint {self.name!r} id must be positive")
        if self.direction not in (-1, 1):
            raise ConfigurationError(f"Joint {self.name!r} direction must be 1 or -1")
        if not MIN_POSITION_TICK <= self.zero_tick <= MAX_POSITION_TICK:
            raise ConfigurationError(
                f"Joint {self.name!r} zero_tick must be between "
                f"{MIN_POSITION_TICK} and {MAX_POSITION_TICK}"
            )
        values = [
            self.min_rad,
            self.max_rad,
            self.max_vel_rad_s,
            self.max_acc_rad_s2,
        ]
        if any(not math.isfinite(value) for value in values):
            raise ConfigurationError(f"Joint {self.name!r} contains non-finite values")
        if self.min_rad >= self.max_rad:
            raise ConfigurationError(f"Joint {self.name!r} min_rad must be smaller than max_rad")
        if self.max_vel_rad_s <= 0:
            raise ConfigurationError(f"Joint {self.name!r} max_vel_rad_s must be positive")
        if self.max_acc_rad_s2 <= 0:
            raise ConfigurationError(f"Joint {self.name!r} max_acc_rad_s2 must be positive")

    def check_limit(self, position_rad: float) -> None:
        if not self.min_rad <= position_rad <= self.max_rad:
            raise LimitViolation(
                f"{self.name} target {position_rad:.4f} rad is outside "
                f"[{self.min_rad:.4f}, {self.max_rad:.4f}] rad"
            )

    def rad_to_tick(self, position_rad: float) -> int:
        self.check_limit(position_rad)
        tick = self.zero_tick + self.direction * rad_to_ticks_delta(position_rad)
        tick = int(round(tick))
        if not MIN_POSITION_TICK <= tick <= MAX_POSITION_TICK:
            raise LimitViolation(
                f"{self.name} target {position_rad:.4f} rad maps to raw tick {tick}, "
                f"outside [{MIN_POSITION_TICK}, {MAX_POSITION_TICK}]"
            )
        return tick

    def tick_to_rad(self, tick: int) -> float:
        if not MIN_POSITION_TICK <= tick <= MAX_POSITION_TICK:
            raise ValueError(f"tick must be between {MIN_POSITION_TICK} and {MAX_POSITION_TICK}")
        delta = (tick - self.zero_tick) * self.direction
        return ticks_delta_to_rad(delta)

    def deg_to_tick(self, position_deg: float) -> int:
        return self.rad_to_tick(deg_to_rad(position_deg))

    def tick_to_deg(self, tick: int) -> float:
        return rad_to_deg(self.tick_to_rad(tick))

    def with_calibration(
        self,
        *,
        zero_tick: int | None = None,
        direction: int | None = None,
        min_rad: float | None = None,
        max_rad: float | None = None,
    ) -> "JointConfig":
        joint = replace(
            self,
            zero_tick=self.zero_tick if zero_tick is None else int(zero_tick),
            direction=self.direction if direction is None else int(direction),
            min_rad=self.min_rad if min_rad is None else float(min_rad),
            max_rad=self.max_rad if max_rad is None else float(max_rad),
        )
        joint.validate()
        return joint

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "zero_tick": self.zero_tick,
            "direction": self.direction,
            "min_rad": self.min_rad,
            "max_rad": self.max_rad,
            "max_vel_rad_s": self.max_vel_rad_s,
            "max_acc_rad_s2": self.max_acc_rad_s2,
        }

    @property
    def ticks_per_revolution(self) -> int:
        return POSITION_TICKS_PER_REVOLUTION
