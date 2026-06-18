from __future__ import annotations

from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import yaml

from .constants import (
    DEFAULT_BAUDRATE,
    DEFAULT_CONTROL_HZ,
    DEFAULT_FEEDBACK_HZ,
    DEFAULT_LOW_VOLTAGE,
    DEFAULT_MAX_STEP_RAD,
    DEFAULT_TRAJECTORY_INPUT_HZ,
    DEFAULT_WEB_MOTION_HZ,
    DEFAULT_WEB_RENDER_HZ,
    DEFAULT_WEB_STATE_HZ,
)
from .errors import ConfigurationError
from .model import JointConfig, Pose


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def _read_yaml_file(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except OSError as exc:
        raise ConfigurationError(f"Failed to read config file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"Failed to parse YAML config {path}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ConfigurationError(f"Config file {path} must contain a mapping")
    return dict(data)


def _resolve_include(base_dir: Path, value: Any) -> Path:
    path = Path(str(value))
    if not path.is_absolute():
        path = base_dir / path
    return path


def _merge_mapping_section(
    data: dict[str, Any],
    section: str,
    incoming: Mapping[str, Any],
    *,
    source_name: str,
) -> None:
    current = data.get(section) or {}
    if not isinstance(current, Mapping):
        raise ConfigurationError(f"{section} must be a mapping")
    overlap = set(current) & set(incoming)
    if overlap:
        names = ", ".join(sorted(str(name) for name in overlap))
        raise ConfigurationError(
            f"{section} settings are duplicated between main config and {source_name}: {names}"
        )
    data[section] = {**current, **incoming}


@dataclass(frozen=True)
class ArmSettings:
    name: str = "soarm"
    port: str | None = None
    baudrate: int = DEFAULT_BAUDRATE
    control_hz: int = DEFAULT_CONTROL_HZ
    low_voltage: float = DEFAULT_LOW_VOLTAGE
    max_step_rad: float = DEFAULT_MAX_STEP_RAD
    auto_disable: bool = True

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "ArmSettings":
        data = data or {}
        settings = cls(
            name=str(data.get("name", "soarm")),
            port=data.get("port"),
            baudrate=int(data.get("baudrate", DEFAULT_BAUDRATE)),
            control_hz=int(data.get("control_hz", DEFAULT_CONTROL_HZ)),
            low_voltage=float(data.get("low_voltage", DEFAULT_LOW_VOLTAGE)),
            max_step_rad=float(data.get("max_step_rad", DEFAULT_MAX_STEP_RAD)),
            auto_disable=bool(data.get("auto_disable", True)),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.baudrate <= 0:
            raise ConfigurationError("arm.baudrate must be positive")
        if self.control_hz <= 0:
            raise ConfigurationError("arm.control_hz must be positive")
        if self.low_voltage <= 0:
            raise ConfigurationError("arm.low_voltage must be positive")
        if self.max_step_rad <= 0:
            raise ConfigurationError("arm.max_step_rad must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "port": self.port,
            "baudrate": self.baudrate,
            "control_hz": self.control_hz,
            "low_voltage": self.low_voltage,
            "max_step_rad": self.max_step_rad,
            "auto_disable": self.auto_disable,
        }


@dataclass(frozen=True)
class CalibrationMetadata:
    calibrated: bool = False
    calibrated_at: str | None = None
    version: int = 1

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "CalibrationMetadata":
        data = data or {}
        return cls(
            calibrated=bool(data.get("calibrated", False)),
            calibrated_at=data.get("calibrated_at"),
            version=int(data.get("version", 1)),
        )

    def mark_calibrated(self) -> "CalibrationMetadata":
        return replace(
            self,
            calibrated=True,
            calibrated_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "calibrated": self.calibrated,
            "calibrated_at": self.calibrated_at,
            "version": self.version,
        }


@dataclass(frozen=True)
class FrequencySettings:
    trajectory_input_hz: int = DEFAULT_TRAJECTORY_INPUT_HZ
    feedback_hz: int = DEFAULT_FEEDBACK_HZ
    web_motion_hz: int = DEFAULT_WEB_MOTION_HZ
    web_state_hz: int = DEFAULT_WEB_STATE_HZ
    web_render_hz: int = DEFAULT_WEB_RENDER_HZ

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "FrequencySettings":
        data = data or {}
        settings = cls(
            trajectory_input_hz=int(data.get("trajectory_input_hz", DEFAULT_TRAJECTORY_INPUT_HZ)),
            feedback_hz=int(data.get("feedback_hz", DEFAULT_FEEDBACK_HZ)),
            web_motion_hz=int(data.get("web_motion_hz", DEFAULT_WEB_MOTION_HZ)),
            web_state_hz=int(data.get("web_state_hz", DEFAULT_WEB_STATE_HZ)),
            web_render_hz=int(data.get("web_render_hz", DEFAULT_WEB_RENDER_HZ)),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        for name, value in self.to_dict().items():
            if int(value) <= 0:
                raise ConfigurationError(f"frequencies.{name} must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {
            "trajectory_input_hz": self.trajectory_input_hz,
            "feedback_hz": self.feedback_hz,
            "web_motion_hz": self.web_motion_hz,
            "web_state_hz": self.web_state_hz,
            "web_render_hz": self.web_render_hz,
        }


@dataclass(frozen=True)
class PIDSettings:
    p: int = 16
    i: int = 0
    d: int = 32

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "PIDSettings":
        data = data or {}
        settings = cls(
            p=int(data.get("p", 16)),
            i=int(data.get("i", 0)),
            d=int(data.get("d", 32)),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        for name, value in self.to_dict().items():
            if not 0 <= int(value) <= 255:
                raise ConfigurationError(f"motor_profile.pid.{name} must be in [0, 255]")

    def to_dict(self) -> dict[str, int]:
        return {"p": self.p, "i": self.i, "d": self.d}


@dataclass(frozen=True)
class MotorOverrideSettings:
    max_torque_limit: int | None = None
    protection_current: int | None = None

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "MotorOverrideSettings":
        data = data or {}
        settings = cls(
            max_torque_limit=_optional_int(data.get("max_torque_limit")),
            protection_current=_optional_int(data.get("protection_current")),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        for name, value in self.to_dict().items():
            if value is not None and int(value) < 0:
                raise ConfigurationError(f"motor_profile.overrides.{name} must be non-negative")

    def to_dict(self) -> dict[str, int | None]:
        values: dict[str, int | None] = {}
        if self.max_torque_limit is not None:
            values["max_torque_limit"] = self.max_torque_limit
        if self.protection_current is not None:
            values["protection_current"] = self.protection_current
        return values


@dataclass(frozen=True)
class MotorProfileSettings:
    enabled: bool = True
    apply_on_enable: bool = True
    write_mode: str = "write_if_drift"
    operating_mode: int = 0
    return_delay_time: int = 0
    maximum_acceleration: int = 254
    startup_acceleration: int = 254
    clear_phase_feedback_overflow_bit: bool = True
    pid: PIDSettings = field(default_factory=PIDSettings)
    overrides: dict[str, MotorOverrideSettings] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "MotorProfileSettings":
        data = data or {}
        raw_overrides = data.get("overrides") or {}
        if not isinstance(raw_overrides, Mapping):
            raise ConfigurationError("motor_profile.overrides must be a mapping")
        settings = cls(
            enabled=bool(data.get("enabled", True)),
            apply_on_enable=bool(data.get("apply_on_enable", True)),
            write_mode=str(data.get("write_mode", "write_if_drift")),
            operating_mode=int(data.get("operating_mode", 0)),
            return_delay_time=int(data.get("return_delay_time", 0)),
            maximum_acceleration=int(data.get("maximum_acceleration", 254)),
            startup_acceleration=int(data.get("startup_acceleration", 254)),
            clear_phase_feedback_overflow_bit=bool(
                data.get("clear_phase_feedback_overflow_bit", True)
            ),
            pid=PIDSettings.from_mapping(data.get("pid")),
            overrides={
                str(name): MotorOverrideSettings.from_mapping(value)
                for name, value in raw_overrides.items()
            },
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.write_mode not in {"write_if_drift", "always", "check_only", "disabled"}:
            raise ConfigurationError(
                "motor_profile.write_mode must be one of: "
                "write_if_drift, always, check_only, disabled"
            )
        for name in (
            "operating_mode",
            "return_delay_time",
            "maximum_acceleration",
            "startup_acceleration",
        ):
            value = int(getattr(self, name))
            if not 0 <= value <= 255:
                raise ConfigurationError(f"motor_profile.{name} must be in [0, 255]")
        self.pid.validate()
        for override in self.overrides.values():
            override.validate()

    @property
    def active(self) -> bool:
        return bool(self.enabled and self.write_mode != "disabled")

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "enabled": self.enabled,
            "apply_on_enable": self.apply_on_enable,
            "write_mode": self.write_mode,
            "operating_mode": self.operating_mode,
            "return_delay_time": self.return_delay_time,
            "maximum_acceleration": self.maximum_acceleration,
            "startup_acceleration": self.startup_acceleration,
            "clear_phase_feedback_overflow_bit": self.clear_phase_feedback_overflow_bit,
            "pid": self.pid.to_dict(),
        }
        if self.overrides:
            payload["overrides"] = {
                name: override.to_dict()
                for name, override in self.overrides.items()
                if override.to_dict()
            }
        return payload


@dataclass(frozen=True)
class ConfigSource:
    main_path: Path | None = None
    runtime_path: Path | None = None
    runtime_ref: str | None = None
    motor_profile_path: Path | None = None
    motor_profile_ref: str | None = None

    @property
    def split(self) -> bool:
        return self.runtime_path is not None or self.motor_profile_path is not None


@dataclass(frozen=True)
class SOARMConfig:
    arm: ArmSettings = field(default_factory=ArmSettings)
    joints: dict[str, JointConfig] = field(default_factory=dict)
    poses: dict[str, Pose] = field(default_factory=dict)
    calibration: CalibrationMetadata = field(default_factory=CalibrationMetadata)
    frequencies: FrequencySettings = field(default_factory=FrequencySettings)
    motor_profile: MotorProfileSettings = field(default_factory=MotorProfileSettings)
    source: ConfigSource = field(default_factory=ConfigSource, repr=False, compare=False)

    @classmethod
    def from_file(cls, path: str | Path) -> "SOARMConfig":
        path = Path(path)
        data = _read_yaml_file(path)
        base_dir = path.parent
        raw_includes = data.pop("includes", None) or {}
        if not isinstance(raw_includes, Mapping):
            raise ConfigurationError("includes must be a mapping")

        runtime_ref = raw_includes.get("runtime")
        runtime_path = None
        if runtime_ref is not None:
            runtime_path = _resolve_include(base_dir, runtime_ref)
            runtime_data = _read_yaml_file(runtime_path)
            for section in ("arm", "frequencies"):
                incoming = runtime_data.get(section)
                if incoming is None:
                    continue
                if not isinstance(incoming, Mapping):
                    raise ConfigurationError(f"{runtime_path}:{section} must be a mapping")
                _merge_mapping_section(
                    data,
                    section,
                    incoming,
                    source_name=str(runtime_path),
                )

        motor_profile_ref = raw_includes.get("motor_profile")
        motor_profile_path = None
        if motor_profile_ref is not None:
            if "motor_profile" in data:
                raise ConfigurationError(
                    "motor_profile is duplicated between main config and included profile"
                )
            motor_profile_path = _resolve_include(base_dir, motor_profile_ref)
            profile_data = _read_yaml_file(motor_profile_path)
            if "motor_profile" in profile_data:
                profile_data = profile_data["motor_profile"]
            data["motor_profile"] = profile_data

        source = ConfigSource(
            main_path=path,
            runtime_path=runtime_path,
            runtime_ref=None if runtime_ref is None else str(runtime_ref),
            motor_profile_path=motor_profile_path,
            motor_profile_ref=None if motor_profile_ref is None else str(motor_profile_ref),
        )
        return cls.from_mapping(data, source=source)

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        *,
        source: ConfigSource | None = None,
    ) -> "SOARMConfig":
        if not isinstance(data, Mapping):
            raise ConfigurationError("Config root must be a mapping")

        arm = ArmSettings.from_mapping(data.get("arm"))

        raw_joints = data.get("joints")
        if not isinstance(raw_joints, Mapping) or not raw_joints:
            raise ConfigurationError("Config must define at least one joint")

        joints: dict[str, JointConfig] = {}
        for name, joint_data in raw_joints.items():
            if not isinstance(joint_data, Mapping):
                raise ConfigurationError(f"joints.{name} must be a mapping")
            joints[str(name)] = JointConfig.from_mapping(str(name), joint_data)

        raw_poses = data.get("poses") or {}
        if not isinstance(raw_poses, Mapping):
            raise ConfigurationError("poses must be a mapping")

        poses: dict[str, Pose] = {}
        for name, pose_data in raw_poses.items():
            poses[str(name)] = Pose.from_mapping(str(name), pose_data)

        calibration = CalibrationMetadata.from_mapping(data.get("calibration"))
        frequencies = FrequencySettings.from_mapping(data.get("frequencies"))
        motor_profile = MotorProfileSettings.from_mapping(data.get("motor_profile"))

        config = cls(
            arm=arm,
            joints=joints,
            poses=poses,
            calibration=calibration,
            frequencies=frequencies,
            motor_profile=motor_profile,
            source=source or ConfigSource(),
        )
        config.validate()
        return config

    def validate(self) -> None:
        self.arm.validate()
        self.frequencies.validate()
        self.motor_profile.validate()

        ids: dict[int, str] = {}
        for name, joint in self.joints.items():
            joint.validate()
            if joint.id in ids:
                raise ConfigurationError(
                    f"Joint {name!r} and {ids[joint.id]!r} both use id {joint.id}"
                )
            ids[joint.id] = name

        known = set(self.joints)
        for pose_name, pose in self.poses.items():
            unknown = set(pose.joints) - known
            if unknown:
                names = ", ".join(sorted(unknown))
                raise ConfigurationError(f"Pose {pose_name!r} references unknown joints: {names}")
            for joint_name, value in pose.joints.items():
                self.joints[joint_name].check_limit(value)

        unknown_overrides = set(self.motor_profile.overrides) - known
        if unknown_overrides:
            names = ", ".join(sorted(unknown_overrides))
            raise ConfigurationError(f"motor_profile.overrides references unknown joints: {names}")

    @property
    def joint_names(self) -> list[str]:
        return list(self.joints)

    @property
    def servo_ids(self) -> list[int]:
        return [joint.id for joint in self.joints.values()]

    def get_pose(self, name: str) -> Pose:
        try:
            return self.poses[name]
        except KeyError as exc:
            available = ", ".join(sorted(self.poses)) or "<none>"
            raise ConfigurationError(f"Unknown pose {name!r}; available poses: {available}") from exc

    def replace_joint_calibrations(
        self,
        calibration_by_name: Mapping[str, Mapping[str, Any]],
    ) -> "SOARMConfig":
        joints = dict(self.joints)
        for name, calibration in calibration_by_name.items():
            if name not in joints:
                raise ConfigurationError(f"Unknown joint {name!r}")
            joints[name] = joints[name].with_calibration(
                zero_tick=int(calibration["zero_tick"]),
                direction=int(calibration["direction"]),
                min_rad=float(calibration["min_rad"]),
                max_rad=float(calibration["max_rad"]),
            )
        config = SOARMConfig(
            arm=self.arm,
            joints=joints,
            poses=self.poses,
            calibration=self.calibration.mark_calibrated(),
            frequencies=self.frequencies,
            motor_profile=self.motor_profile,
            source=self.source,
        )
        config.validate()
        return config

    def replace_pose(self, name: str, positions: Mapping[str, float]) -> "SOARMConfig":
        poses = dict(self.poses)
        poses[name] = Pose(name=name, joints={key: float(value) for key, value in positions.items()})
        config = SOARMConfig(
            arm=self.arm,
            joints=self.joints,
            poses=poses,
            calibration=self.calibration,
            frequencies=self.frequencies,
            motor_profile=self.motor_profile,
            source=self.source,
        )
        config.validate()
        return config

    def replace_arm_port(self, port: str | None) -> "SOARMConfig":
        config = SOARMConfig(
            arm=replace(self.arm, port=port),
            joints=self.joints,
            poses=self.poses,
            calibration=self.calibration,
            frequencies=self.frequencies,
            motor_profile=self.motor_profile,
            source=self.source,
        )
        config.validate()
        return config

    def replace_arm_identity(self, *, name: str, port: str | None) -> "SOARMConfig":
        clean_name = name.strip()
        if not clean_name:
            raise ConfigurationError("arm.name must not be empty")
        config = SOARMConfig(
            arm=replace(self.arm, name=clean_name, port=port),
            joints=self.joints,
            poses=self.poses,
            calibration=self.calibration,
            frequencies=self.frequencies,
            motor_profile=self.motor_profile,
            source=self.source,
        )
        config.validate()
        return config

    def replace_runtime_settings(
        self,
        *,
        control_hz: int,
        trajectory_input_hz: int,
        feedback_hz: int,
        web_motion_hz: int,
        web_state_hz: int,
        web_render_hz: int,
    ) -> "SOARMConfig":
        config = SOARMConfig(
            arm=replace(self.arm, control_hz=int(control_hz)),
            joints=self.joints,
            poses=self.poses,
            calibration=self.calibration,
            frequencies=FrequencySettings(
                trajectory_input_hz=int(trajectory_input_hz),
                feedback_hz=int(feedback_hz),
                web_motion_hz=int(web_motion_hz),
                web_state_hz=int(web_state_hz),
                web_render_hz=int(web_render_hz),
            ),
            motor_profile=self.motor_profile,
            source=self.source,
        )
        config.validate()
        return config

    def to_dict(self) -> dict[str, Any]:
        return {
            "arm": self.arm.to_dict(),
            "frequencies": self.frequencies.to_dict(),
            "motor_profile": self.motor_profile.to_dict(),
            "calibration": self.calibration.to_dict(),
            "joints": {name: joint.to_dict() for name, joint in self.joints.items()},
            "poses": {name: pose.to_dict() for name, pose in self.poses.items()},
        }

    def to_main_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        includes: dict[str, str] = {}
        if self.source.runtime_ref is not None:
            includes["runtime"] = self.source.runtime_ref
        if self.source.motor_profile_ref is not None:
            includes["motor_profile"] = self.source.motor_profile_ref
        if includes:
            payload["includes"] = includes
        payload["arm"] = {
            "name": self.arm.name,
            "port": self.arm.port,
            "baudrate": self.arm.baudrate,
        }
        payload["calibration"] = self.calibration.to_dict()
        payload["joints"] = {name: joint.to_dict() for name, joint in self.joints.items()}
        payload["poses"] = {name: pose.to_dict() for name, pose in self.poses.items()}
        return payload

    def to_runtime_dict(self) -> dict[str, Any]:
        return {
            "arm": {
                "control_hz": self.arm.control_hz,
                "low_voltage": self.arm.low_voltage,
                "max_step_rad": self.arm.max_step_rad,
                "auto_disable": self.arm.auto_disable,
            },
            "frequencies": self.frequencies.to_dict(),
        }

    def save(self, path: str | Path) -> None:
        path = Path(path)
        if self.source.split and self.source.main_path is not None:
            if path.absolute() == self.source.main_path.absolute():
                self._write_yaml(self.source.main_path, self.to_main_dict())
                if self.source.runtime_path is not None:
                    self._write_yaml(self.source.runtime_path, self.to_runtime_dict())
                if self.source.motor_profile_path is not None:
                    self._write_yaml(self.source.motor_profile_path, self.motor_profile.to_dict())
                return
            self._write_yaml(path, self.to_dict())
            return
        self._write_yaml(path, self.to_dict())

    @staticmethod
    def _write_yaml(path: Path, payload: Mapping[str, Any]) -> None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(dict(payload), sort_keys=False))
        except OSError as exc:
            raise ConfigurationError(f"Failed to write config file {path}: {exc}") from exc
