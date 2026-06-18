from __future__ import annotations

import glob
import time
from typing import Mapping, Sequence

from ..constants import (
    DEFAULT_BAUDRATE,
    DEFAULT_POSITION_ACCELERATION,
    DEFAULT_POSITION_SPEED,
    MAX_POSITION_TICK,
    MAX_SPEED_SETTING,
    MIN_POSITION_TICK,
    MIDDLE_POSITION_TICK,
)
from ..errors import HardwareError, NotConnectedError
from .registers import (
    BAUDRATE_TO_CODE,
    Register,
    decode_sign_magnitude,
    encode_sign_magnitude,
    get_register,
)


POSITION_MODE = 0
TORQUE_DISABLED = 0
TORQUE_ENABLED = 1
EEPROM_UNLOCKED = 0
EEPROM_LOCKED = 1

_ACCELERATION_REGISTER = get_register("Acceleration")
_GOAL_POSITION_REGISTER = get_register("Goal_Position")
_GOAL_VELOCITY_REGISTER = get_register("Goal_Velocity")
_POSITION_COMMAND_SIZE = 7
_GOAL_TIME_DISABLED = 0


class ServoBus:
    """Lightweight Feetech STS/SMS bus for SOARM.

    The public name stays ``ServoBus`` so the rest of SOARM does not care which
    low-level SDK backs the hardware layer. Internally this talks directly to
    ``scservo_sdk`` instead of the former Vassar high-level wrapper.
    """

    def __init__(
        self,
        servo_ids: list[int],
        *,
        port: str | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        auto_disable: bool = True,
    ) -> None:
        self.servo_ids = list(servo_ids)
        self.port = port
        self.baudrate = baudrate
        self.auto_disable = auto_disable
        self._scs = None
        self._port_handler = None
        self._packet = None
        self._connected = False
        self._torque_disabled = False
        self._position_ready: set[int] = set()
        self._profile_cache: dict[int, tuple[int, int]] = {}
        self._sync_readers: dict[tuple[str, tuple[int, ...]], object] = {}

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return
        try:
            import scservo_sdk as scs
        except ImportError as exc:
            raise HardwareError(
                "Missing dependency scservo_sdk. Install SOARM with "
                "`pip install -e .` so ftservo-python-sdk is available."
            ) from exc

        port = self.port or self._auto_port()
        try:
            print(
                f"hardware: connecting to Feetech servo bus "
                f"(port={port}, baudrate={self.baudrate}, ids={self.servo_ids})"
            )
            port_handler = scs.PortHandler(port)
            self._patch_packet_timeout(port_handler, scs)
            if not port_handler.openPort():
                raise HardwareError(f"Failed to open servo port {port}")
            if not port_handler.setBaudRate(self.baudrate):
                raise HardwareError(f"Failed to set servo baudrate {self.baudrate}")
            self._scs = scs
            self._port_handler = port_handler
            self._packet = scs.sms_sts(port_handler)
            self.port = port
        except Exception:
            self._close_port()
            self._scs = None
            self._packet = None
            raise

        self._connected = True
        self._torque_disabled = False
        self._position_ready.clear()
        self._profile_cache.clear()
        self._sync_readers.clear()

    def disconnect(self) -> None:
        if not self._connected:
            self._close_port()
            return
        try:
            if self.auto_disable and not self._torque_disabled:
                print("hardware: disconnecting; disabling servo torque before closing the port.")
                self.disable_all()
            else:
                print("hardware: disconnecting; closing the Feetech servo port.")
        finally:
            self._close_port()
            self._scs = None
            self._packet = None
            self._connected = False
            self._torque_disabled = False
            self._position_ready.clear()
            self._profile_cache.clear()
            self._sync_readers.clear()

    def __enter__(self) -> "ServoBus":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def _require_packet(self):
        if not self._connected or self._packet is None:
            raise NotConnectedError("Servo bus is not connected")
        return self._packet

    def _require_scs(self):
        if self._scs is None:
            raise NotConnectedError("Servo bus is not connected")
        return self._scs

    def _auto_port(self) -> str:
        ports = self.list_ports()
        if len(ports) == 1:
            return ports[0]
        if not ports:
            raise HardwareError("No candidate Feetech servo ports found")
        available = ", ".join(ports)
        raise HardwareError(f"Multiple candidate servo ports found: {available}")

    def _close_port(self) -> None:
        port_handler = self._port_handler
        self._port_handler = None
        if port_handler is None:
            return
        try:
            if getattr(port_handler, "is_open", False):
                port_handler.closePort()
        except Exception:
            pass

    @staticmethod
    def _patch_packet_timeout(port_handler, scs) -> None:
        def set_packet_timeout(self, packet_length):  # noqa: ANN001
            self.packet_start_time = self.getCurrentTime()
            self.packet_timeout = (self.tx_time_per_byte * packet_length) + (
                self.tx_time_per_byte * 3.0
            ) + 50

        if hasattr(scs, "PortHandler") and hasattr(port_handler, "setPacketTimeout"):
            port_handler.setPacketTimeout = set_packet_timeout.__get__(port_handler, scs.PortHandler)

    def _comm_success(self, result: int) -> bool:
        return result == self._require_scs().COMM_SUCCESS

    def _format_comm(self, result: int) -> str:
        packet = self._require_packet()
        return packet.getTxRxResult(result) or f"comm_result={result}"

    def _format_error(self, error: int) -> str:
        if error == 0:
            return "no servo error"
        packet = self._require_packet()
        return packet.getRxPacketError(error) or f"servo_error=0x{error:02x}"

    def _check_comm(self, result: int, error: int, context: str) -> None:
        if not self._comm_success(result):
            raise HardwareError(f"{context}: {self._format_comm(result)}")
        if error:
            raise HardwareError(f"{context}: {self._format_error(error)}")

    def _bus_io_error(self, context: str, exc: Exception) -> HardwareError:
        port = self.port or "<auto>"
        return HardwareError(f"{context} on servo bus {port}: {exc}")

    def _read_raw(self, servo_id: int, register: Register) -> tuple[int, int, int]:
        packet = self._require_packet()
        if register.size == 1:
            value, result, error = packet.read1ByteTxRx(servo_id, register.address)
        elif register.size == 2:
            value, result, error = packet.read2ByteTxRx(servo_id, register.address)
        elif register.size == 4:
            value, result, error = packet.read4ByteTxRx(servo_id, register.address)
        else:
            data, result, error = packet.readTxRx(servo_id, register.address, register.size)
            value = int.from_bytes(bytes(data), "little")
        return int(value), int(result), int(error)

    def _write_raw(
        self,
        servo_id: int,
        register: Register,
        value: int,
        *,
        tx_only: bool = False,
    ) -> tuple[int, int]:
        packet = self._require_packet()
        if register.size == 1:
            if tx_only:
                return int(packet.write1ByteTxOnly(servo_id, register.address, value)), 0
            result, error = packet.write1ByteTxRx(servo_id, register.address, value)
        elif register.size == 2:
            if tx_only:
                return int(packet.write2ByteTxOnly(servo_id, register.address, value)), 0
            result, error = packet.write2ByteTxRx(servo_id, register.address, value)
        elif register.size == 4:
            if tx_only:
                return int(packet.write4ByteTxOnly(servo_id, register.address, value)), 0
            result, error = packet.write4ByteTxRx(servo_id, register.address, value)
        else:
            payload = list(int(value).to_bytes(register.size, "little", signed=False))
            if tx_only:
                return int(packet.writeTxOnly(servo_id, register.address, register.size, payload)), 0
            result, error = packet.writeTxRx(servo_id, register.address, register.size, payload)
        return int(result), int(error)

    @staticmethod
    def _decode(value: int, register: Register, *, raw: bool) -> int:
        if raw or register.signed_bit is None:
            return int(value)
        return decode_sign_magnitude(int(value), register.signed_bit)

    @staticmethod
    def _encode(value: int, register: Register) -> int:
        value = int(value)
        if register.signed_bit is not None:
            value = encode_sign_magnitude(value, register.signed_bit)
        return value

    def _new_sync_reader(self, register: Register):
        scs = self._require_scs()
        packet = self._require_packet()
        try:
            return scs.GroupSyncRead(packet, register.address, register.size)
        except TypeError:
            return scs.GroupSyncRead(self._port_handler, packet, register.address, register.size)

    def _sync_reader(self, register: Register, ids: Sequence[int], *, strict: bool):
        key = (register.name, tuple(ids))
        reader = self._sync_readers.get(key)
        if reader is not None:
            return reader

        reader = self._new_sync_reader(register)
        for servo_id in ids:
            ok = reader.addParam(servo_id)
            if not ok and strict:
                raise HardwareError(f"Failed to add servo {servo_id} to sync read {register.name}")
        if hasattr(reader, "makeParam"):
            reader.makeParam()
        if hasattr(reader, "is_param_changed"):
            reader.is_param_changed = False
        self._sync_readers[key] = reader
        return reader

    def read_register(self, name: str, servo_id: int, *, raw: bool = False) -> int:
        register = get_register(name)
        try:
            value, result, error = self._read_raw(int(servo_id), register)
        except HardwareError:
            raise
        except Exception as exc:
            raise self._bus_io_error(
                f"Failed to read {register.name} from servo {servo_id}",
                exc,
            ) from exc
        self._check_comm(result, error, f"Failed to read {register.name} from servo {servo_id}")
        return self._decode(value, register, raw=raw)

    def write_register(
        self,
        name: str,
        servo_id: int,
        value: int,
        *,
        unlock: bool = False,
        tx_only: bool = False,
    ) -> bool:
        register = get_register(name)
        if register.readonly:
            raise HardwareError(f"Register {register.name} is read-only")
        servo_id = int(servo_id)
        value = self._encode(int(value), register)
        if unlock:
            self.unlock_eeprom(servo_id)
        try:
            result, error = self._write_raw(servo_id, register, value, tx_only=tx_only)
        except HardwareError:
            raise
        except Exception as exc:
            raise self._bus_io_error(
                f"Failed to write {register.name} on servo {servo_id}",
                exc,
            ) from exc
        self._check_comm(result, error, f"Failed to write {register.name} on servo {servo_id}")
        return True

    def read(
        self,
        name: str,
        servo_id: int | None = None,
        *,
        ids: Sequence[int] | None = None,
        raw: bool = False,
        strict: bool = True,
    ) -> int | dict[int, int | None]:
        if servo_id is not None:
            return self.read_register(name, servo_id, raw=raw)
        return self.sync_read(name, ids=ids, raw=raw, strict=strict)

    def write(
        self,
        name: str,
        values: Mapping[int, int] | int,
        *,
        servo_id: int | None = None,
        unlock: bool = False,
    ) -> dict[int, bool] | bool:
        if servo_id is not None:
            if not isinstance(values, int):
                raise ValueError("values must be an int when servo_id is provided")
            return self.write_register(name, servo_id, values, unlock=unlock)
        if not isinstance(values, Mapping):
            raise ValueError("values must be a mapping when servo_id is omitted")
        return self.sync_write(name, values)

    def sync_read(
        self,
        name: str,
        *,
        ids: Sequence[int] | None = None,
        raw: bool = False,
        strict: bool = True,
    ) -> dict[int, int | None]:
        register = get_register(name)
        ids = [int(servo_id) for servo_id in (ids or self.servo_ids)]
        reader = self._sync_reader(register, ids, strict=strict)
        try:
            result = int(reader.txRxPacket())
        except HardwareError:
            raise
        except Exception as exc:
            if strict:
                raise self._bus_io_error(f"Failed to sync read {register.name}", exc) from exc
            return {servo_id: None for servo_id in ids}
        if not self._comm_success(result):
            if strict:
                raise HardwareError(f"Failed to sync read {register.name}: {self._format_comm(result)}")
            return {servo_id: None for servo_id in ids}
        values: dict[int, int | None] = {}
        for servo_id in ids:
            availability = reader.isAvailable(servo_id, register.address, register.size)
            if isinstance(availability, tuple):
                available, error = availability
            else:
                available, error = bool(availability), 0
            if not available:
                if strict:
                    raise HardwareError(f"No {register.name} returned for servo {servo_id}")
                values[servo_id] = None
                continue
            if error:
                if strict:
                    raise HardwareError(
                        f"Servo {servo_id} returned an error while reading "
                        f"{register.name}: {self._format_error(int(error))}"
                    )
                values[servo_id] = None
                continue
            value = int(reader.getData(servo_id, register.address, register.size))
            values[servo_id] = self._decode(value, register, raw=raw)
        return values

    def sync_write(self, name: str, values: Mapping[int, int]) -> dict[int, bool]:
        register = get_register(name)
        return self._sync_write_register(register, values)

    def _sync_write_register(self, register: Register, values: Mapping[int, int]) -> dict[int, bool]:
        if register.readonly:
            raise HardwareError(f"Register {register.name} is read-only")
        packet = self._require_packet()
        results: dict[int, bool] = {}
        param: list[int] = []
        for servo_id, value in values.items():
            servo_id = int(servo_id)
            encoded = self._encode(int(value), register)
            if not self._value_fits(encoded, register.size):
                results[servo_id] = False
                continue
            param.append(servo_id)
            param.extend(self._value_to_bytes(encoded, register.size))
            results[servo_id] = True
        if not param or not all(results.values()):
            return results
        try:
            result = int(
                packet.syncWriteTxOnly(
                    register.address,
                    register.size,
                    param,
                    len(param),
                )
            )
        except HardwareError:
            raise
        except Exception as exc:
            raise self._bus_io_error(f"Failed to sync write {register.name}", exc) from exc
        if not self._comm_success(result):
            raise HardwareError(f"Failed to sync write {register.name}: {self._format_comm(result)}")
        return results

    def scan(self, ids: list[int] | None = None) -> dict[int, bool]:
        ids = list(ids or self.servo_ids)
        result: dict[int, bool] = {}
        for servo_id in ids:
            result[servo_id] = self.ping(servo_id)
        return result

    def broadcast_ping(self, ids: list[int] | None = None) -> dict[int, int]:
        ids = list(ids or self.servo_ids)
        found: dict[int, int] = {}
        for servo_id in ids:
            try:
                model = self.read_register("Model_Number", servo_id, raw=True)
            except HardwareError:
                continue
            found[servo_id] = int(model)
        return found

    def ping(self, servo_id: int) -> bool:
        packet = self._require_packet()
        model, result, error = packet.ping(int(servo_id))
        return self._comm_success(int(result)) and int(error) == 0 and int(model) != 0

    def read_model_numbers(self, ids: list[int] | None = None) -> dict[int, int | None]:
        ids = list(ids or self.servo_ids)
        values: dict[int, int | None] = {}
        for servo_id in ids:
            try:
                values[servo_id] = self.read_register("Model_Number", servo_id, raw=True)
            except HardwareError:
                values[servo_id] = None
        return values

    def read_firmware_versions(self, ids: list[int] | None = None) -> dict[int, str | None]:
        ids = list(ids or self.servo_ids)
        values: dict[int, str | None] = {}
        for servo_id in ids:
            try:
                major = self.read_register("Firmware_Major_Version", servo_id, raw=True)
                minor = self.read_register("Firmware_Minor_Version", servo_id, raw=True)
            except HardwareError:
                values[servo_id] = None
                continue
            values[servo_id] = f"{major}.{minor}"
        return values

    def read_position(self, servo_id: int) -> int:
        value = self.read_register("Present_Position", int(servo_id), raw=True)
        return int(value)

    def read_positions(
        self,
        ids: list[int] | None = None,
        *,
        strict: bool = True,
    ) -> dict[int, int | None]:
        return self.sync_read("Present_Position", ids=ids, raw=True, strict=strict)

    def read_voltage(self, servo_id: int) -> float:
        value = self.read_register("Present_Voltage", int(servo_id), raw=True)
        return self._voltage_to_float(value)

    def read_voltages(
        self,
        ids: list[int] | None = None,
        *,
        strict: bool = False,
    ) -> dict[int, float | None]:
        values = self.sync_read("Present_Voltage", ids=ids, raw=True, strict=strict)
        return {
            servo_id: None if value is None else self._voltage_to_float(value)
            for servo_id, value in values.items()
        }

    def read_temperature(self, servo_id: int) -> int:
        return int(self.read_register("Present_Temperature", int(servo_id), raw=True))

    def read_temperatures(
        self,
        ids: list[int] | None = None,
        *,
        strict: bool = False,
    ) -> dict[int, int | None]:
        values = self.sync_read("Present_Temperature", ids=ids, raw=True, strict=strict)
        return {servo_id: None if value is None else int(value) for servo_id, value in values.items()}

    def read_currents(
        self,
        ids: list[int] | None = None,
        *,
        strict: bool = False,
    ) -> dict[int, int | None]:
        return self.sync_read("Present_Current", ids=ids, raw=False, strict=strict)

    def write_positions(
        self,
        positions: Mapping[int, int],
        *,
        speed: int = DEFAULT_POSITION_SPEED,
        acceleration: int = DEFAULT_POSITION_ACCELERATION,
    ) -> dict[int, bool]:
        clean = {
            int(servo_id): max(MIN_POSITION_TICK, min(MAX_POSITION_TICK, int(position)))
            for servo_id, position in positions.items()
        }
        if not clean:
            return {}
        self.prepare_position_control(list(clean))
        result = self._sync_write_position_command(
            clean,
            speed=int(speed),
            acceleration=int(acceleration),
        )
        if any(result.values()):
            self._torque_disabled = False
        return result

    def prepare_position_control(self, ids: Sequence[int] | None = None) -> None:
        ids = [int(servo_id) for servo_id in (ids or self.servo_ids)]
        for servo_id in ids:
            if servo_id in self._position_ready:
                continue
            mode = self.read_register("Operating_Mode", servo_id, raw=True)
            if mode != POSITION_MODE:
                self.set_operating_mode(servo_id, POSITION_MODE)
            torque = self.read_register("Torque_Enable", servo_id, raw=True)
            if torque != TORQUE_ENABLED:
                self.enable_torque(servo_id)
            self._position_ready.add(servo_id)

    def _write_profile_if_needed(self, ids: Sequence[int], *, speed: int, acceleration: int) -> None:
        speed = max(0, min(MAX_SPEED_SETTING, int(speed)))
        acceleration = max(0, min(254, int(acceleration)))
        profile = (speed, acceleration)
        changed = [servo_id for servo_id in ids if self._profile_cache.get(servo_id) != profile]
        if not changed:
            return
        self._sync_write_register(_ACCELERATION_REGISTER, {servo_id: acceleration for servo_id in changed})
        self._sync_write_register(_GOAL_VELOCITY_REGISTER, {servo_id: speed for servo_id in changed})
        for servo_id in changed:
            self._profile_cache[servo_id] = profile

    def _sync_write_position_command(
        self,
        positions: Mapping[int, int],
        *,
        speed: int,
        acceleration: int,
    ) -> dict[int, bool]:
        """Write STS/SMS position commands in the packet shape the vendor SDK uses.

        ``sms_sts.SyncWritePosEx`` starts at ``Acceleration`` and writes seven
        bytes per servo: acceleration, goal position, goal time, and goal
        velocity. Writing only ``Goal_Position`` updates the register but does
        not reliably trigger motion on the STS3215.
        """
        packet = self._require_packet()
        speed = max(0, min(MAX_SPEED_SETTING, int(speed)))
        acceleration = max(0, min(254, int(acceleration)))
        results: dict[int, bool] = {}
        param: list[int] = []

        for servo_id, position in positions.items():
            servo_id = int(servo_id)
            position = max(MIN_POSITION_TICK, min(MAX_POSITION_TICK, int(position)))
            if not self._value_fits(position, _GOAL_POSITION_REGISTER.size):
                results[servo_id] = False
                continue
            if not self._value_fits(speed, _GOAL_VELOCITY_REGISTER.size):
                results[servo_id] = False
                continue
            param.append(servo_id)
            param.append(acceleration)
            param.extend(self._value_to_bytes(position, _GOAL_POSITION_REGISTER.size))
            param.extend(self._value_to_bytes(_GOAL_TIME_DISABLED, 2))
            param.extend(self._value_to_bytes(speed, _GOAL_VELOCITY_REGISTER.size))
            results[servo_id] = True

        if not param or not all(results.values()):
            return results
        try:
            result = int(
                packet.syncWriteTxOnly(
                    _ACCELERATION_REGISTER.address,
                    _POSITION_COMMAND_SIZE,
                    param,
                    len(param),
                )
            )
        except HardwareError:
            raise
        except Exception as exc:
            raise self._bus_io_error("Failed to sync write position command", exc) from exc
        if not self._comm_success(result):
            raise HardwareError(f"Failed to sync write position command: {self._format_comm(result)}")
        for servo_id in positions:
            self._profile_cache[int(servo_id)] = (speed, acceleration)
        return results

    def enable_torque(self, servo_id: int | None = None) -> dict[int, bool] | bool:
        if servo_id is not None:
            ok = self.write_register("Torque_Enable", int(servo_id), TORQUE_ENABLED)
            if ok:
                self._torque_disabled = False
            return ok
        result = self.sync_write("Torque_Enable", {servo_id: TORQUE_ENABLED for servo_id in self.servo_ids})
        if any(result.values()):
            self._torque_disabled = False
        return result

    def disable_torque(self, servo_id: int | None = None) -> dict[int, bool] | bool:
        if servo_id is not None:
            ok = self.write_register("Torque_Enable", int(servo_id), TORQUE_DISABLED)
            self._position_ready.discard(int(servo_id))
            if ok and all(servo_id not in self._position_ready for servo_id in self.servo_ids):
                self._torque_disabled = True
            return ok
        result = self.sync_write("Torque_Enable", {servo_id: TORQUE_DISABLED for servo_id in self.servo_ids})
        self._position_ready.clear()
        self._profile_cache.clear()
        self._torque_disabled = True
        return result

    def disable_all(self) -> None:
        self.disable_torque()

    def set_operating_mode(self, servo_id: int, mode: int) -> bool:
        servo_id = int(servo_id)
        self.unlock_eeprom(servo_id)
        try:
            ok = self.write_register("Operating_Mode", servo_id, int(mode))
            self._position_ready.discard(servo_id)
            return ok
        finally:
            self.lock_eeprom(servo_id)

    def get_pid(self, servo_id: int) -> dict[str, int]:
        return {
            "p": self.read_register("P_Coefficient", int(servo_id), raw=True),
            "i": self.read_register("I_Coefficient", int(servo_id), raw=True),
            "d": self.read_register("D_Coefficient", int(servo_id), raw=True),
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
        servo_id = int(servo_id)
        self.unlock_eeprom(servo_id)
        try:
            if p is not None:
                self.write_register("P_Coefficient", servo_id, self._byte_value(p, "p"))
            if i is not None:
                self.write_register("I_Coefficient", servo_id, self._byte_value(i, "i"))
            if d is not None:
                self.write_register("D_Coefficient", servo_id, self._byte_value(d, "d"))
            return self.get_pid(servo_id)
        finally:
            if lock:
                self.lock_eeprom(servo_id)

    def unlock_eeprom(self, servo_id: int) -> bool:
        return self.write_register("Lock", int(servo_id), EEPROM_UNLOCKED)

    def lock_eeprom(self, servo_id: int) -> bool:
        return self.write_register("Lock", int(servo_id), EEPROM_LOCKED)

    def set_middle_position(self, ids: list[int] | None = None) -> bool:
        ids = ids or self.servo_ids
        result = self.write_positions({int(servo_id): MIDDLE_POSITION_TICK for servo_id in ids})
        return all(result.values())

    def set_motor_id(self, current_id: int, new_id: int, *, confirm: bool = True) -> bool:
        current_id = int(current_id)
        new_id = int(new_id)
        if current_id == new_id:
            return True
        self.disable_torque(current_id)
        self.unlock_eeprom(current_id)
        try:
            self.write_register("ID", current_id, new_id)
            time.sleep(0.05)
            self._sync_readers.clear()
            if not confirm:
                return True
            return self.ping(new_id)
        finally:
            try:
                self.lock_eeprom(new_id)
            except HardwareError:
                pass

    def set_baudrate_code(self, servo_id: int, baudrate: int) -> bool:
        try:
            code = BAUDRATE_TO_CODE[int(baudrate)]
        except KeyError as exc:
            supported = ", ".join(str(value) for value in sorted(BAUDRATE_TO_CODE))
            raise HardwareError(f"Unsupported Feetech baudrate {baudrate}. Supported: {supported}") from exc
        self.unlock_eeprom(int(servo_id))
        try:
            return self.write_register("Baud_Rate", int(servo_id), code)
        finally:
            self.lock_eeprom(int(servo_id))

    @staticmethod
    def _value_to_bytes(value: int, size: int) -> list[int]:
        return list(int(value).to_bytes(size, "little", signed=False))

    @staticmethod
    def _value_fits(value: int, size: int) -> bool:
        return 0 <= int(value) < (1 << (8 * size))

    @staticmethod
    def _byte_value(value: int, name: str) -> int:
        value = int(value)
        if not 0 <= value <= 255:
            raise ValueError(f"PID {name} must be in [0, 255]")
        return value

    @staticmethod
    def _voltage_to_float(value: int) -> float:
        return float(value) / 10.0

    @staticmethod
    def list_ports() -> list[str]:
        try:
            from serial.tools import list_ports
        except ImportError as exc:
            raise HardwareError("Missing dependency pyserial") from exc

        ports = [port.device for port in list_ports.comports()]
        if ports:
            candidates = [
                port
                for port in ports
                if "usb" in port.lower()
                or "acm" in port.lower()
                or "modem" in port.lower()
                or "wch" in port.lower()
            ]
            return sorted(candidates or ports)
        fallback = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/cu.usb*")
        return sorted(fallback)
