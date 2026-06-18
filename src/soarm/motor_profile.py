from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .config import MotorProfileSettings
from .model import JointConfig


EEPROM_PROFILE_REGISTERS = {
    "Operating_Mode",
    "Return_Delay_Time",
    "Maximum_Acceleration",
    "P_Coefficient",
    "I_Coefficient",
    "D_Coefficient",
    "Max_Torque_Limit",
    "Protection_Current",
    "Phase",
}


@dataclass(frozen=True)
class MotorProfileResult:
    joint: str
    servo_id: int
    register: str
    current: int | None
    target: int
    action: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint": self.joint,
            "servo_id": self.servo_id,
            "register": self.register,
            "current": self.current,
            "target": self.target,
            "action": self.action,
        }


def motor_profile_requires_torque_disabled(
    profile: MotorProfileSettings,
    *,
    force: bool = False,
    check_only: bool = False,
) -> bool:
    if not profile.active or check_only:
        return False
    mode = "always" if force else profile.write_mode
    return mode in {"always", "write_if_drift"}


def apply_motor_profile(
    bus,
    *,
    profile: MotorProfileSettings,
    joints: Mapping[str, JointConfig],
    force: bool = False,
    check_only: bool = False,
) -> list[MotorProfileResult]:
    if not profile.active:
        return []

    mode = "check_only" if check_only else ("always" if force else profile.write_mode)
    results: list[MotorProfileResult] = []
    for joint_name, joint in joints.items():
        servo_id = int(joint.id)
        targets = _profile_targets(profile, joint_name)
        targets.update(_phase_targets(bus, servo_id, profile.clear_phase_feedback_overflow_bit))

        pending_writes: list[tuple[str, int, int | None]] = []
        for register, target in targets.items():
            current = _read_register(bus, register, servo_id)
            drifted = current != int(target)
            if mode == "check_only":
                if drifted:
                    results.append(
                        MotorProfileResult(
                            joint=joint_name,
                            servo_id=servo_id,
                            register=register,
                            current=current,
                            target=int(target),
                            action="drift",
                        )
                    )
                continue

            if mode == "always" or drifted:
                pending_writes.append((register, int(target), current))

        if not pending_writes:
            continue

        eeprom_writes = [item for item in pending_writes if item[0] in EEPROM_PROFILE_REGISTERS]
        ram_writes = [item for item in pending_writes if item[0] not in EEPROM_PROFILE_REGISTERS]
        if eeprom_writes:
            bus.unlock_eeprom(servo_id)
            try:
                for register, target, current in eeprom_writes:
                    bus.write_register(register, servo_id, target)
                    results.append(
                        MotorProfileResult(
                            joint=joint_name,
                            servo_id=servo_id,
                            register=register,
                            current=current,
                            target=target,
                            action="written",
                        )
                    )
            finally:
                bus.lock_eeprom(servo_id)
        for register, target, current in ram_writes:
            bus.write_register(register, servo_id, target)
            results.append(
                MotorProfileResult(
                    joint=joint_name,
                    servo_id=servo_id,
                    register=register,
                    current=current,
                    target=target,
                    action="written",
                )
            )

    return results


def _profile_targets(profile: MotorProfileSettings, joint_name: str) -> dict[str, int]:
    targets = {
        "Operating_Mode": profile.operating_mode,
        "Return_Delay_Time": profile.return_delay_time,
        "Maximum_Acceleration": profile.maximum_acceleration,
        "Acceleration": profile.startup_acceleration,
        "P_Coefficient": profile.pid.p,
        "I_Coefficient": profile.pid.i,
        "D_Coefficient": profile.pid.d,
    }
    override = profile.overrides.get(joint_name)
    if override is not None:
        if override.max_torque_limit is not None:
            targets["Max_Torque_Limit"] = override.max_torque_limit
        if override.protection_current is not None:
            targets["Protection_Current"] = override.protection_current
    return targets


def _phase_targets(bus, servo_id: int, enabled: bool) -> dict[str, int]:
    if not enabled:
        return {}
    try:
        phase = _read_register(bus, "Phase", servo_id)
    except Exception:  # noqa: BLE001
        return {}
    target = int(phase) & ~0x10
    if target == int(phase):
        return {}
    return {"Phase": target}


def _read_register(bus, register: str, servo_id: int) -> int:
    return int(bus.read_register(register, servo_id, raw=True))
