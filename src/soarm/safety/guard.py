from __future__ import annotations

from typing import Mapping

from ..config import SOARMConfig
from ..errors import EmergencyStopActive, SafetyError
from .limits import (
    ensure_acceleration_feasible,
    ensure_joint_limits,
    ensure_known_joints,
    ensure_step_limits,
    ensure_velocity_feasible,
)


class SafetyGuard:
    def __init__(self, config: SOARMConfig) -> None:
        self.config = config
        self.emergency_stopped = False

    def check_not_emergency_stopped(self) -> None:
        if self.emergency_stopped:
            raise EmergencyStopActive("Emergency stop is active; call reset_emergency_stop() first")

    def set_emergency_stop(self) -> None:
        self.emergency_stopped = True

    def reset_emergency_stop(self) -> None:
        self.emergency_stopped = False

    def validate_motion(
        self,
        *,
        current: Mapping[str, float],
        target: Mapping[str, float],
        moving_joints: set[str],
        duration: float,
        voltages: Mapping[int, float | None] | None = None,
    ) -> None:
        self.check_not_emergency_stopped()
        ensure_known_joints(self.config, target)
        ensure_joint_limits(self.config, target)
        ensure_step_limits(self.config, current, target, moving_joints)
        ensure_velocity_feasible(self.config, current, target, moving_joints, duration)
        ensure_acceleration_feasible(self.config, current, target, moving_joints, duration)
        if voltages:
            self.check_voltages(voltages)

    def check_voltages(self, voltages: Mapping[int, float | None]) -> None:
        low: list[str] = []
        id_to_name = {joint.id: name for name, joint in self.config.joints.items()}
        for servo_id, voltage in voltages.items():
            if voltage is None:
                continue
            if voltage < self.config.arm.low_voltage:
                low.append(f"{id_to_name.get(servo_id, servo_id)}={voltage:.2f}V")
        if low:
            raise SafetyError(
                f"Low servo voltage detected below {self.config.arm.low_voltage:.2f}V: "
                + ", ".join(low)
            )
