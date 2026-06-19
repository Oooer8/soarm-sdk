from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Register:
    name: str
    address: int
    size: int
    readonly: bool = False
    signed_bit: int | None = None


def _r(
    name: str,
    address: int,
    size: int,
    *,
    readonly: bool = False,
    signed_bit: int | None = None,
) -> Register:
    return Register(
        name=name,
        address=address,
        size=size,
        readonly=readonly,
        signed_bit=signed_bit,
    )


# STS/SMS-series control table used by Feetech STS3215 servos. The names mirror
# LeRobot's Feetech table so future ports from LeRobot remain mechanical.
REGISTER_TABLE: dict[str, Register] = {
    # EEPROM
    "Firmware_Major_Version": _r("Firmware_Major_Version", 0, 1, readonly=True),
    "Firmware_Minor_Version": _r("Firmware_Minor_Version", 1, 1, readonly=True),
    "Model_Number": _r("Model_Number", 3, 2, readonly=True),
    "ID": _r("ID", 5, 1),
    "Baud_Rate": _r("Baud_Rate", 6, 1),
    "Return_Delay_Time": _r("Return_Delay_Time", 7, 1),
    "Response_Status_Level": _r("Response_Status_Level", 8, 1),
    "Min_Position_Limit": _r("Min_Position_Limit", 9, 2),
    "Max_Position_Limit": _r("Max_Position_Limit", 11, 2),
    "Max_Temperature_Limit": _r("Max_Temperature_Limit", 13, 1),
    "Max_Voltage_Limit": _r("Max_Voltage_Limit", 14, 1),
    "Min_Voltage_Limit": _r("Min_Voltage_Limit", 15, 1),
    "Max_Torque_Limit": _r("Max_Torque_Limit", 16, 2),
    "Phase": _r("Phase", 18, 1),
    "Unloading_Condition": _r("Unloading_Condition", 19, 1),
    "LED_Alarm_Condition": _r("LED_Alarm_Condition", 20, 1),
    "P_Coefficient": _r("P_Coefficient", 21, 1),
    "D_Coefficient": _r("D_Coefficient", 22, 1),
    "I_Coefficient": _r("I_Coefficient", 23, 1),
    "Minimum_Startup_Force": _r("Minimum_Startup_Force", 24, 2),
    "CW_Dead_Zone": _r("CW_Dead_Zone", 26, 1),
    "CCW_Dead_Zone": _r("CCW_Dead_Zone", 27, 1),
    "Protection_Current": _r("Protection_Current", 28, 2),
    "Angular_Resolution": _r("Angular_Resolution", 30, 1),
    "Homing_Offset": _r("Homing_Offset", 31, 2, signed_bit=11),
    "Operating_Mode": _r("Operating_Mode", 33, 1),
    "Protective_Torque": _r("Protective_Torque", 34, 1),
    "Protection_Time": _r("Protection_Time", 35, 1),
    "Overload_Torque": _r("Overload_Torque", 36, 1),
    "Velocity_closed_loop_P_proportional_coefficient": _r(
        "Velocity_closed_loop_P_proportional_coefficient",
        37,
        1,
    ),
    "Over_Current_Protection_Time": _r("Over_Current_Protection_Time", 38, 1),
    "Velocity_closed_loop_I_integral_coefficient": _r(
        "Velocity_closed_loop_I_integral_coefficient",
        39,
        1,
    ),
    # SRAM
    "Torque_Enable": _r("Torque_Enable", 40, 1),
    "Acceleration": _r("Acceleration", 41, 1),
    "Goal_Position": _r("Goal_Position", 42, 2),
    "Goal_Time": _r("Goal_Time", 44, 2),
    "Goal_Velocity": _r("Goal_Velocity", 46, 2, signed_bit=15),
    "Torque_Limit": _r("Torque_Limit", 48, 2),
    "Lock": _r("Lock", 55, 1),
    "Present_Position": _r("Present_Position", 56, 2, readonly=True),
    "Present_Velocity": _r("Present_Velocity", 58, 2, readonly=True, signed_bit=15),
    "Present_Load": _r("Present_Load", 60, 2, readonly=True, signed_bit=10),
    "Present_Voltage": _r("Present_Voltage", 62, 1, readonly=True),
    "Present_Temperature": _r("Present_Temperature", 63, 1, readonly=True),
    "Status": _r("Status", 65, 1, readonly=True),
    "Moving": _r("Moving", 66, 1, readonly=True),
    "Present_Current": _r("Present_Current", 69, 2, readonly=True, signed_bit=15),
    "Goal_Position_2": _r("Goal_Position_2", 71, 2, readonly=True),
    # Factory / firmware-specific tuning registers.
    "Moving_Velocity": _r("Moving_Velocity", 80, 1),
    "Moving_Velocity_Threshold": _r("Moving_Velocity_Threshold", 80, 1),
    "DTs": _r("DTs", 81, 1),
    "Velocity_Unit_factor": _r("Velocity_Unit_factor", 82, 1),
    "Hts": _r("Hts", 83, 1),
    "Maximum_Velocity_Limit": _r("Maximum_Velocity_Limit", 84, 1),
    "Maximum_Acceleration": _r("Maximum_Acceleration", 85, 1),
    "Acceleration_Multiplier": _r("Acceleration_Multiplier", 86, 1),
}


ALIASES = {
    "Goal_Speed": "Goal_Velocity",
    "Present_Speed": "Present_Velocity",
    "Present_Temp": "Present_Temperature",
    "Voltage": "Present_Voltage",
    "Temperature": "Present_Temperature",
    "Current": "Present_Current",
    "Mode": "Operating_Mode",
    "Torque": "Torque_Enable",
}


BAUDRATE_TO_CODE = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    57_600: 5,
    38_400: 6,
    19_200: 7,
}

CODE_TO_BAUDRATE = {code: baudrate for baudrate, code in BAUDRATE_TO_CODE.items()}


def get_register(name: str) -> Register:
    canonical = ALIASES.get(name, name)
    try:
        return REGISTER_TABLE[canonical]
    except KeyError as exc:
        known = ", ".join(sorted(REGISTER_TABLE))
        raise KeyError(f"Unknown Feetech register {name!r}. Known registers: {known}") from exc


def canonical_register_name(name: str) -> str:
    return get_register(name).name


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    if value < 0:
        return (-value) | (1 << sign_bit)
    return value


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    if value & (1 << sign_bit):
        return -(value & ~(1 << sign_bit))
    return value
