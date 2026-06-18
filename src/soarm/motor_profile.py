from .hardware.motor_profile import (
    EEPROM_PROFILE_REGISTERS,
    MotorProfileResult,
    apply_motor_profile,
    motor_profile_requires_torque_disabled,
)

__all__ = [
    "EEPROM_PROFILE_REGISTERS",
    "MotorProfileResult",
    "apply_motor_profile",
    "motor_profile_requires_torque_disabled",
]
