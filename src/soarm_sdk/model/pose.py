from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..errors import ConfigurationError


@dataclass(frozen=True)
class Pose:
    name: str
    joints: dict[str, float]

    @classmethod
    def from_mapping(cls, name: str, data: Mapping[str, Any]) -> "Pose":
        if not isinstance(data, Mapping):
            raise ConfigurationError(f"poses.{name} must be a mapping")
        return cls(name=name, joints={str(key): float(value) for key, value in data.items()})

    def to_dict(self) -> dict[str, float]:
        return dict(self.joints)
