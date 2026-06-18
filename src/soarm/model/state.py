from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass(frozen=True)
class JointState:
    name: str
    id: int
    position_tick: int | None
    position_rad: float | None
    voltage: float | None = None
    online: bool = True
    error: str | None = None
    timestamp: float = field(default_factory=time.time)


@dataclass(frozen=True)
class ArmState:
    connected: bool
    enabled: bool
    emergency_stopped: bool
    joints: dict[str, JointState]
    timestamp: float = field(default_factory=time.time)
