from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, TYPE_CHECKING

from ..config import SOARMConfig
from ..errors import ConfigurationError, LimitViolation, MotionError
from ..motion import InterpolationMode, TimedJointTrajectory, TrajectoryPoint
from ..safety.limits import minimum_motion_duration

if TYPE_CHECKING:
    from ..arm import SOARM


DEMONSTRATION_FORMAT = "soarm_sdk.demonstration.v1"
_STEP_SAFETY_FACTOR = 0.8
_POSITION_TOLERANCE_RAD = 0.03
_MAX_STEP_RETRIES = 5


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _ensure_finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ConfigurationError(f"{name} must be finite")
    return value


def _normalize_joints(config: SOARMConfig, joints: Iterable[str] | None) -> list[str]:
    if joints is None:
        return list(config.joints)
    names = [str(name) for name in joints]
    if not names:
        raise ConfigurationError("at least one joint must be selected")
    unknown = set(names) - set(config.joints)
    if unknown:
        raise ConfigurationError(f"Unknown joints: {', '.join(sorted(unknown))}")
    if len(names) != len(set(names)):
        raise ConfigurationError("joint names must not contain duplicates")
    return names


@dataclass(frozen=True)
class DemonstrationSample:
    time_from_start: float
    positions: dict[str, float]

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "DemonstrationSample":
        if not isinstance(data, Mapping):
            raise ConfigurationError("demonstration sample must be a mapping")
        if "time_from_start" in data:
            raw_time = data["time_from_start"]
        elif "t" in data:
            raw_time = data["t"]
        else:
            raise ConfigurationError("demonstration sample is missing time_from_start")
        raw_positions = data.get("positions")
        if not isinstance(raw_positions, Mapping):
            raise ConfigurationError("demonstration sample positions must be a mapping")
        return cls(
            time_from_start=_ensure_finite("sample time_from_start", raw_time),
            positions={
                str(name): _ensure_finite(f"sample position {name}", value)
                for name, value in raw_positions.items()
            },
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "time_from_start": self.time_from_start,
            "positions": dict(self.positions),
        }


@dataclass
class Demonstration:
    joints: list[str]
    samples: list[DemonstrationSample]
    metadata: dict[str, Any] = field(default_factory=dict)
    created_at: str = field(default_factory=_utc_now)

    def __post_init__(self) -> None:
        self.joints = [str(name) for name in self.joints]
        self.samples = list(self.samples)
        self.metadata = dict(self.metadata)
        self._validate()

    @property
    def duration(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1].time_from_start

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "Demonstration":
        if not isinstance(data, Mapping):
            raise ConfigurationError("demonstration file must contain a mapping")
        format_name = data.get("format", DEMONSTRATION_FORMAT)
        if format_name != DEMONSTRATION_FORMAT:
            raise ConfigurationError(
                f"Unsupported demonstration format {format_name!r}; "
                f"expected {DEMONSTRATION_FORMAT!r}"
            )
        raw_joints = data.get("joints")
        if not isinstance(raw_joints, list):
            raise ConfigurationError("demonstration joints must be a list")
        raw_samples = data.get("samples")
        if not isinstance(raw_samples, list):
            raise ConfigurationError("demonstration samples must be a list")
        raw_metadata = data.get("metadata") or {}
        if not isinstance(raw_metadata, Mapping):
            raise ConfigurationError("demonstration metadata must be a mapping")
        return cls(
            joints=[str(name) for name in raw_joints],
            samples=[DemonstrationSample.from_mapping(sample) for sample in raw_samples],
            metadata=dict(raw_metadata),
            created_at=str(data.get("created_at") or _utc_now()),
        )

    @classmethod
    def load(cls, path: str | Path) -> "Demonstration":
        return load_demonstration(path)

    def save(self, path: str | Path) -> None:
        save_demonstration(self, path)

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": DEMONSTRATION_FORMAT,
            "created_at": self.created_at,
            "metadata": dict(self.metadata),
            "joints": list(self.joints),
            "samples": [sample.to_dict() for sample in self.samples],
        }

    def window(
        self,
        *,
        start_at: float | None = None,
        end_at: float | None = None,
    ) -> list[DemonstrationSample]:
        if start_at is not None:
            start_at = _ensure_finite("start_at", start_at)
        if end_at is not None:
            end_at = _ensure_finite("end_at", end_at)
        if start_at is not None and start_at < 0:
            raise ConfigurationError("start_at must be non-negative")
        if end_at is not None and end_at < 0:
            raise ConfigurationError("end_at must be non-negative")
        if start_at is not None and end_at is not None and end_at < start_at:
            raise ConfigurationError("end_at must be greater than or equal to start_at")

        samples = self.samples
        if start_at is not None:
            samples = [sample for sample in samples if sample.time_from_start >= start_at]
        if end_at is not None:
            samples = [sample for sample in samples if sample.time_from_start <= end_at]
        if not samples:
            raise MotionError("demonstration window contains no samples")
        return samples

    def _validate(self) -> None:
        if not self.joints:
            raise ConfigurationError("demonstration must contain at least one joint")
        if len(self.joints) != len(set(self.joints)):
            raise ConfigurationError("demonstration joints must not contain duplicates")
        if not self.samples:
            raise ConfigurationError("demonstration must contain at least one sample")

        previous_time: float | None = None
        joint_set = set(self.joints)
        for index, sample in enumerate(self.samples):
            sample_time = _ensure_finite(f"sample {index} time_from_start", sample.time_from_start)
            if sample_time < 0:
                raise ConfigurationError("sample time_from_start must be non-negative")
            if previous_time is not None and sample_time < previous_time:
                raise ConfigurationError("demonstration sample times must be monotonic")
            previous_time = sample_time

            sample_joint_set = set(sample.positions)
            if sample_joint_set != joint_set:
                missing = joint_set - sample_joint_set
                extra = sample_joint_set - joint_set
                details: list[str] = []
                if missing:
                    details.append(f"missing: {', '.join(sorted(missing))}")
                if extra:
                    details.append(f"extra: {', '.join(sorted(extra))}")
                raise ConfigurationError(
                    f"sample {index} joint set does not match demonstration joints "
                    f"({'; '.join(details)})"
                )
            for name, value in sample.positions.items():
                _ensure_finite(f"sample {index} position {name}", value)


class DemonstrationRecorder:
    """Record joint-space demonstrations from a SOARM instance."""

    def __init__(
        self,
        arm: "SOARM",
        *,
        sample_hz: float = 20.0,
        joints: Iterable[str] | None = None,
    ) -> None:
        self.arm = arm
        self.sample_hz = _ensure_finite("sample_hz", sample_hz)
        if self.sample_hz <= 0:
            raise ConfigurationError("sample_hz must be positive")
        self.joints = _normalize_joints(arm.config, joints)

    def record(
        self,
        *,
        duration: float | None = None,
        output_path: str | Path | None = None,
        disable_torque: bool = True,
        announce: Callable[[str], None] | None = None,
    ) -> Demonstration:
        if duration is not None:
            duration = _ensure_finite("duration", duration)
            if duration < 0:
                raise ConfigurationError("duration must be non-negative")

        if not self.arm.connected:
            self.arm.connect()
        if disable_torque:
            if announce:
                announce("Disabling torque for manual teaching.")
            self.arm.disable()

        period = 1.0 / self.sample_hz
        samples: list[DemonstrationSample] = []
        start_time = time.monotonic()
        next_sample_time = start_time

        try:
            while True:
                positions = self.arm.get_joint_positions(unit="rad")
                elapsed = time.monotonic() - start_time
                samples.append(
                    DemonstrationSample(
                        time_from_start=elapsed,
                        positions={name: float(positions[name]) for name in self.joints},
                    )
                )
                if duration is not None and elapsed >= duration:
                    break

                next_sample_time += period
                sleep_time = next_sample_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif -sleep_time > period:
                    next_sample_time = time.monotonic()
        except KeyboardInterrupt:
            if not samples:
                raise
            if announce:
                announce("Recording stopped by keyboard interrupt.")

        demo = Demonstration(
            joints=list(self.joints),
            samples=samples,
            metadata={
                "arm_name": self.arm.config.arm.name,
                "sample_hz": self.sample_hz,
                "unit": "rad",
                "source": "soarm_sdk.demonstration",
            },
        )
        if output_path is not None:
            demo.save(output_path)
        return demo


class DemonstrationReplayer:
    """Replay joint-space demonstrations through the SOARM motion controller."""

    def __init__(self, arm: "SOARM") -> None:
        self.arm = arm

    def replay(
        self,
        demonstration: Demonstration,
        *,
        speed: float = 1.0,
        start_at: float | None = None,
        end_at: float | None = None,
        move_to_start: bool = True,
        lead_in_duration: float = 2.0,
        feedback_tolerance: float = _POSITION_TOLERANCE_RAD,
        output_hz: float | None = None,
        interpolation: InterpolationMode = "pchip",
        dry_run: bool = False,
        announce: Callable[[str], None] | None = None,
    ) -> int:
        speed = _ensure_finite("speed", speed)
        if speed <= 0:
            raise ConfigurationError("speed must be positive")
        lead_in_duration = _ensure_finite("lead_in_duration", lead_in_duration)
        if lead_in_duration < 0:
            raise ConfigurationError("lead_in_duration must be non-negative")
        feedback_tolerance = _ensure_finite("feedback_tolerance", feedback_tolerance)
        if feedback_tolerance <= 0:
            raise ConfigurationError("feedback_tolerance must be positive")
        replay_output_hz = (
            self.arm.config.arm.control_hz
            if output_hz is None
            else _ensure_finite("output_hz", output_hz)
        )
        if replay_output_hz <= 0:
            raise ConfigurationError("output_hz must be positive")
        if interpolation not in {"linear", "pchip"}:
            raise ConfigurationError("interpolation must be one of: linear, pchip")

        samples = demonstration.window(start_at=start_at, end_at=end_at)
        validate_demonstration_for_config(demonstration, self.arm.config)
        try:
            trajectory = TimedJointTrajectory(
                [
                    TrajectoryPoint(
                        time_from_start=sample.time_from_start,
                        positions=dict(sample.positions),
                    )
                    for sample in samples
                ],
                interpolation=interpolation,
            )
        except MotionError as exc:
            raise ConfigurationError(str(exc)) from exc
        if dry_run:
            return len(samples)

        if not self.arm.connected:
            self.arm.connect()
        if not self.arm.enabled:
            self.arm.enable()

        first = samples[0]
        if move_to_start:
            if announce:
                max_delta = self._max_target_delta(first.positions)
                announce(
                    "Moving to the first demonstration sample "
                    f"(max delta {max_delta:.4f} rad, "
                    f"step limit {self.arm.config.arm.max_step_rad:.4f} rad)."
                )
            self._move_with_step_limit(
                first.positions,
                duration=lead_in_duration,
                feedback_tolerance=feedback_tolerance,
            )

        if announce:
            announce(
                f"Replaying {len(samples)} timed samples at {speed:.3f}x "
                f"with {replay_output_hz:.1f} Hz {interpolation} output."
            )
        self.arm.follow_joint_trajectory(
            trajectory,
            speed=speed,
            output_hz=replay_output_hz,
        )
        return len(samples)

    def _max_target_delta(self, target: Mapping[str, float]) -> float:
        current = self.arm.motion.read_positions_rad()
        return _max_delta(current, target, set(target))

    def _move_with_step_limit(
        self,
        target: Mapping[str, float],
        *,
        duration: float,
        feedback_tolerance: float,
    ) -> None:
        moving_joints = set(target)
        step_limit = self.arm.config.arm.max_step_rad * _STEP_SAFETY_FACTOR
        initial = self.arm.motion.read_positions_rad()
        initial_delta = _max_delta(initial, target, moving_joints)
        max_iterations = max(1, int(math.ceil(initial_delta / max(step_limit, 1e-9)))) + 30
        remaining_duration = max(0.0, duration)

        for _index in range(max_iterations):
            current = self.arm.motion.read_positions_rad()
            max_delta = _max_delta(current, target, moving_joints)
            if max_delta <= feedback_tolerance:
                return

            alpha = min(1.0, step_limit / max_delta)
            waypoint_duration = remaining_duration * alpha
            waypoint = _interpolate_target(current, target, moving_joints, alpha=alpha)
            self._move_waypoint_with_retry(
                current,
                waypoint,
                moving_joints,
                duration=waypoint_duration,
                feedback_tolerance=feedback_tolerance,
            )
            remaining_duration = max(0.0, remaining_duration - waypoint_duration)
        current = self.arm.motion.read_positions_rad()
        raise MotionError(
            "Timed out while segmenting replay move to stay within max_step_rad; "
            f"remaining max delta {_max_delta(current, target, moving_joints):.4f} rad "
            f"({_format_largest_deltas(current, target, moving_joints)}). "
            "This usually means the servo did not physically follow the last Goal_Position write, "
            "or Present_Position readback is not updating. "
            + self._format_control_diagnostics(target, moving_joints)
        )

    def _move_waypoint_with_retry(
        self,
        current: Mapping[str, float],
        waypoint: Mapping[str, float],
        moving_joints: set[str],
        *,
        duration: float,
        feedback_tolerance: float,
    ) -> None:
        local_current = dict(current)
        local_waypoint = dict(waypoint)
        local_duration = duration
        for _attempt in range(_MAX_STEP_RETRIES):
            minimum = minimum_motion_duration(
                self.arm.config,
                local_current,
                local_waypoint,
                moving_joints,
            )
            try:
                self.arm.move_joints(local_waypoint, duration=max(local_duration, minimum), wait=True)
                return
            except LimitViolation as exc:
                if "max_step_rad" not in str(exc):
                    raise
                local_current = self.arm.motion.read_positions_rad()
                max_delta = _max_delta(local_current, local_waypoint, moving_joints)
                if max_delta <= feedback_tolerance:
                    return
                alpha = min(1.0, (self.arm.config.arm.max_step_rad * 0.5) / max_delta)
                local_waypoint = _interpolate_target(
                    local_current,
                    local_waypoint,
                    moving_joints,
                    alpha=alpha,
                )
                local_duration = local_duration * alpha
        raise MotionError("Failed to keep replay waypoint within max_step_rad after retries")

    def _format_control_diagnostics(
        self,
        target: Mapping[str, float],
        moving_joints: set[str],
    ) -> str:
        try:
            current = self.arm.motion.read_positions_rad()
        except Exception as exc:  # noqa: BLE001
            return f"Control diagnostics unavailable: failed to read current positions ({exc})."

        joints_by_delta = sorted(
            (
                (
                    name,
                    abs(float(target[name]) - float(current.get(name, target[name]))),
                )
                for name in moving_joints
            ),
            key=lambda item: item[1],
            reverse=True,
        )
        details: list[str] = []
        for name, _delta in joints_by_delta[:3]:
            joint = self.arm.config.joints[name]
            servo_id = int(joint.id)
            target_tick = joint.rad_to_tick(float(target[name]))
            current_rad = current.get(name)
            fields = [
                f"{name}(id={servo_id})",
                f"current_rad={_format_float(current_rad)}",
                f"target_rad={float(target[name]):.4f}",
                f"target_tick={target_tick}",
            ]
            for register in (
                "Present_Position",
                "Goal_Position",
                "Torque_Enable",
                "Operating_Mode",
                "Moving",
                "Goal_Velocity",
                "Acceleration",
                "Torque_Limit",
                "Max_Torque_Limit",
                "Protection_Current",
                "Present_Load",
                "Present_Current",
                "Present_Voltage",
                "Present_Temperature",
                "Status",
            ):
                try:
                    value = self.arm.bus.read_register(register, servo_id, raw=True)
                except Exception as exc:  # noqa: BLE001
                    fields.append(f"{register}=ERR:{exc}")
                else:
                    fields.append(f"{register}={value}")
            details.append("[" + ", ".join(fields) + "]")
        if not details:
            return "Control diagnostics: no moving joints to inspect."
        return "Control diagnostics: " + " ".join(details)

def _max_delta(
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: set[str],
) -> float:
    max_delta = 0.0
    for name in moving_joints:
        if name in current:
            max_delta = max(max_delta, abs(float(target[name]) - float(current[name])))
    return max_delta


def _interpolate_target(
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: set[str],
    *,
    alpha: float,
) -> dict[str, float]:
    return {
        name: float(current.get(name, target[name]))
        + (float(target[name]) - float(current.get(name, target[name]))) * alpha
        for name in moving_joints
    }


def _format_largest_deltas(
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: set[str],
    *,
    limit: int = 3,
) -> str:
    deltas = sorted(
        (
            (
                name,
                abs(float(target[name]) - float(current[name])),
                float(current[name]),
                float(target[name]),
            )
            for name in moving_joints
            if name in current
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    if not deltas:
        return "no readable joints"
    return ", ".join(
        f"{name}: current={current_value:.4f}, target={target_value:.4f}, "
        f"delta={delta:.4f}"
        for name, delta, current_value, target_value in deltas[:limit]
    )


def _format_float(value: float | None) -> str:
    if value is None:
        return "None"
    return f"{float(value):.4f}"


def validate_demonstration_for_config(
    demonstration: Demonstration,
    config: SOARMConfig,
) -> None:
    unknown = set(demonstration.joints) - set(config.joints)
    if unknown:
        raise ConfigurationError(
            f"demonstration contains joints not present in config: {', '.join(sorted(unknown))}"
        )
    for sample in demonstration.samples:
        for name, position in sample.positions.items():
            config.joints[name].check_limit(float(position))


def load_demonstration(path: str | Path) -> Demonstration:
    path = Path(path)
    try:
        data = json.loads(path.read_text())
    except OSError as exc:
        raise ConfigurationError(f"Failed to read demonstration file {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigurationError(f"Failed to parse demonstration JSON {path}: {exc}") from exc
    return Demonstration.from_mapping(data)


def save_demonstration(demonstration: Demonstration, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        path.write_text(json.dumps(demonstration.to_dict(), indent=2) + "\n")
    except OSError as exc:
        raise ConfigurationError(f"Failed to write demonstration file {path}: {exc}") from exc


def record_demonstration(
    arm: "SOARM",
    output_path: str | Path,
    *,
    duration: float | None = None,
    sample_hz: float = 20.0,
    joints: Iterable[str] | None = None,
    disable_torque: bool = True,
    announce: Callable[[str], None] | None = None,
) -> Demonstration:
    recorder = DemonstrationRecorder(arm, sample_hz=sample_hz, joints=joints)
    return recorder.record(
        duration=duration,
        output_path=output_path,
        disable_torque=disable_torque,
        announce=announce,
    )


def replay_demonstration(
    arm: "SOARM",
    input_path: str | Path,
    *,
    speed: float = 1.0,
    start_at: float | None = None,
    end_at: float | None = None,
    move_to_start: bool = True,
    lead_in_duration: float = 2.0,
    feedback_tolerance: float = _POSITION_TOLERANCE_RAD,
    output_hz: float | None = None,
    interpolation: InterpolationMode = "pchip",
    dry_run: bool = False,
    announce: Callable[[str], None] | None = None,
) -> int:
    demonstration = load_demonstration(input_path)
    replayer = DemonstrationReplayer(arm)
    return replayer.replay(
        demonstration,
        speed=speed,
        start_at=start_at,
        end_at=end_at,
        move_to_start=move_to_start,
        lead_in_duration=lead_in_duration,
        feedback_tolerance=feedback_tolerance,
        output_hz=output_hz,
        interpolation=interpolation,
        dry_run=dry_run,
        announce=announce,
    )
