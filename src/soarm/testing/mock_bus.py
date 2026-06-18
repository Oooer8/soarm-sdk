from __future__ import annotations

from typing import Mapping

from ..constants import DEFAULT_POSITION_ACCELERATION, DEFAULT_POSITION_SPEED, MIDDLE_POSITION_TICK
from ..config import SOARMConfig
from ..errors import NotConnectedError
from ..hardware.registers import get_register


class MockBus:
    """In-memory bus for examples, CLI dry runs, and future tests."""

    def __init__(self, config: SOARMConfig, *, voltage: float = 12.0) -> None:
        self.config = config
        self.servo_ids = config.servo_ids
        self.voltage = voltage
        self._connected = False
        self._positions = {joint.id: joint.zero_tick for joint in config.joints.values()}
        self._registers: dict[tuple[int, str], int] = {}
        for servo_id in self.servo_ids:
            self._registers[(servo_id, "ID")] = servo_id
            self._registers[(servo_id, "Operating_Mode")] = 0
            self._registers[(servo_id, "Torque_Enable")] = 0
            self._registers[(servo_id, "Acceleration")] = DEFAULT_POSITION_ACCELERATION
            self._registers[(servo_id, "Goal_Velocity")] = DEFAULT_POSITION_SPEED
            self._registers[(servo_id, "Return_Delay_Time")] = 0
            self._registers[(servo_id, "Maximum_Acceleration")] = 254
            self._registers[(servo_id, "P_Coefficient")] = 16
            self._registers[(servo_id, "I_Coefficient")] = 0
            self._registers[(servo_id, "D_Coefficient")] = 32
        self._disabled = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True
        self._disabled = False

    def disconnect(self) -> None:
        self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise NotConnectedError("Mock bus is not connected")

    def scan(self, ids: list[int] | None = None) -> dict[int, bool]:
        self._require_connected()
        return {servo_id: servo_id in self._positions for servo_id in (ids or self.servo_ids)}

    def broadcast_ping(self, ids: list[int] | None = None) -> dict[int, int]:
        self._require_connected()
        return {servo_id: 3215 for servo_id in (ids or self.servo_ids) if servo_id in self._positions}

    def read_position(self, servo_id: int) -> int:
        self._require_connected()
        return self._positions[servo_id]

    def read_register(self, name: str, servo_id: int, *, raw: bool = False) -> int:
        self._require_connected()
        register = get_register(name)
        if register.name == "Present_Position":
            return self._positions[servo_id]
        if register.name == "Present_Voltage":
            return int(round(self.voltage * 10))
        return self._registers.get((int(servo_id), register.name), 0)

    def write_register(
        self,
        name: str,
        servo_id: int,
        value: int,
        *,
        unlock: bool = False,
        tx_only: bool = False,
    ) -> bool:
        self._require_connected()
        register = get_register(name)
        self._registers[(int(servo_id), register.name)] = int(value)
        return True

    def read(
        self,
        name: str,
        servo_id: int | None = None,
        *,
        ids: list[int] | None = None,
        raw: bool = False,
        strict: bool = True,
    ):
        if servo_id is not None:
            return self.read_register(name, servo_id, raw=raw)
        return self.sync_read(name, ids=ids, raw=raw, strict=strict)

    def write(
        self,
        name: str,
        values,
        *,
        servo_id: int | None = None,
        unlock: bool = False,
    ):
        if servo_id is not None:
            return self.write_register(name, servo_id, values, unlock=unlock)
        return self.sync_write(name, values)

    def sync_read(
        self,
        name: str,
        *,
        ids: list[int] | None = None,
        raw: bool = False,
        strict: bool = True,
    ) -> dict[int, int | None]:
        self._require_connected()
        return {servo_id: self.read_register(name, servo_id, raw=raw) for servo_id in (ids or self.servo_ids)}

    def sync_write(self, name: str, values: Mapping[int, int]) -> dict[int, bool]:
        self._require_connected()
        result: dict[int, bool] = {}
        for servo_id, value in values.items():
            if int(servo_id) in self.servo_ids:
                self.write_register(name, int(servo_id), int(value))
                result[int(servo_id)] = True
            else:
                result[int(servo_id)] = False
        return result

    def read_positions(
        self,
        ids: list[int] | None = None,
        *,
        strict: bool = True,
    ) -> dict[int, int | None]:
        self._require_connected()
        return {servo_id: self._positions.get(servo_id) for servo_id in (ids or self.servo_ids)}

    def read_voltage(self, servo_id: int) -> float:
        self._require_connected()
        if servo_id not in self._positions:
            raise KeyError(servo_id)
        return self.voltage

    def read_voltages(
        self,
        ids: list[int] | None = None,
        *,
        strict: bool = False,
    ) -> dict[int, float | None]:
        self._require_connected()
        return {
            servo_id: self.voltage if servo_id in self._positions else None
            for servo_id in (ids or self.servo_ids)
        }

    def write_positions(
        self,
        positions: Mapping[int, int],
        *,
        speed: int = DEFAULT_POSITION_SPEED,
        acceleration: int = DEFAULT_POSITION_ACCELERATION,
    ) -> dict[int, bool]:
        self._require_connected()
        result: dict[int, bool] = {}
        for servo_id, position in positions.items():
            if servo_id in self._positions:
                self._positions[int(servo_id)] = int(position)
                self._registers[(int(servo_id), "Goal_Position")] = int(position)
                self._registers[(int(servo_id), "Goal_Velocity")] = int(speed)
                self._registers[(int(servo_id), "Acceleration")] = int(acceleration)
                self._registers[(int(servo_id), "Torque_Enable")] = 1
                result[int(servo_id)] = True
            else:
                result[int(servo_id)] = False
        self._disabled = False
        return result

    def disable_all(self) -> None:
        self._require_connected()
        self._disabled = True
        for servo_id in self.servo_ids:
            self._registers[(servo_id, "Torque_Enable")] = 0

    def unlock_eeprom(self, servo_id: int) -> bool:
        self._require_connected()
        self._registers[(int(servo_id), "Lock")] = 0
        return True

    def lock_eeprom(self, servo_id: int) -> bool:
        self._require_connected()
        self._registers[(int(servo_id), "Lock")] = 1
        return True

    def enable_torque(self, servo_id: int | None = None):
        self._require_connected()
        ids = [servo_id] if servo_id is not None else self.servo_ids
        for item in ids:
            self._registers[(int(item), "Torque_Enable")] = 1
        self._disabled = False
        if servo_id is not None:
            return True
        return {item: True for item in ids}

    def disable_torque(self, servo_id: int | None = None):
        self._require_connected()
        ids = [servo_id] if servo_id is not None else self.servo_ids
        for item in ids:
            self._registers[(int(item), "Torque_Enable")] = 0
        self._disabled = True
        if servo_id is not None:
            return True
        return {item: True for item in ids}

    def get_pid(self, servo_id: int) -> dict[str, int]:
        self._require_connected()
        return {
            "p": self.read_register("P_Coefficient", servo_id, raw=True),
            "i": self.read_register("I_Coefficient", servo_id, raw=True),
            "d": self.read_register("D_Coefficient", servo_id, raw=True),
        }

    def set_pid(
        self,
        servo_id: int,
        *,
        p: int | None = None,
        i: int | None = None,
        d: int | None = None,
        lock: bool = True,
    ) -> dict[str, int]:
        self._require_connected()
        if p is not None:
            self.write_register("P_Coefficient", servo_id, p)
        if i is not None:
            self.write_register("I_Coefficient", servo_id, i)
        if d is not None:
            self.write_register("D_Coefficient", servo_id, d)
        return self.get_pid(servo_id)

    def read_model_numbers(self, ids: list[int] | None = None) -> dict[int, int | None]:
        self._require_connected()
        return {servo_id: 3215 if servo_id in self._positions else None for servo_id in (ids or self.servo_ids)}

    def read_firmware_versions(self, ids: list[int] | None = None) -> dict[int, str | None]:
        self._require_connected()
        return {servo_id: "0.0" if servo_id in self._positions else None for servo_id in (ids or self.servo_ids)}

    def set_middle_position(self, ids: list[int] | None = None) -> bool:
        self._require_connected()
        ids = ids or self.servo_ids
        for servo_id in ids:
            if servo_id in self._positions:
                self._positions[servo_id] = MIDDLE_POSITION_TICK
        return True

    def set_motor_id(self, current_id: int, new_id: int, *, confirm: bool = True) -> bool:
        self._require_connected()
        if current_id not in self._positions:
            return False
        self._positions[new_id] = self._positions.pop(current_id)
        updates = {}
        for (servo_id, register), value in list(self._registers.items()):
            if servo_id == current_id:
                updates[(new_id, register)] = value
                del self._registers[(servo_id, register)]
        self._registers.update(updates)
        self._registers[(new_id, "ID")] = new_id
        return True
