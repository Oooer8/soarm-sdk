from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Mapping

from ..config import SOARMConfig
from ..errors import ConfigurationError, MotionError
from ..safety import SafetyGuard

if TYPE_CHECKING:
    from .controller import MotionController


JointStreamMode = Literal["arrival", "tracking", "direct"]


@dataclass(frozen=True)
class JointStreamSnapshot:
    running: bool
    mode: str
    output_hz: float
    target_timeout_s: float
    writes: int
    overruns: int
    stale: bool
    target_age_s: float | None
    output: dict[str, float]
    target: dict[str, float] | None
    target_velocities: dict[str, float]
    velocities: dict[str, float]
    error: str | None = None


class JointStreamingController:
    """Online joint-space streaming controller for teleoperation.

    The controller accepts low-frequency target updates and owns the fixed-rate
    output loop. Smooth modes advance a velocity/acceleration-limited reference;
    direct mode writes the latest target each output tick for lowest latency.
    """

    def __init__(
        self,
        *,
        config: SOARMConfig,
        motion: "MotionController",
        safety: SafetyGuard,
        output_hz: float | None = None,
        target_timeout_s: float = 0.15,
        joint_names: list[str] | None = None,
        mode: JointStreamMode = "arrival",
        tracking_kp: float = 8.0,
        tracking_feedforward: float = 1.0,
    ) -> None:
        self.config = config
        self.motion = motion
        self.safety = safety
        self.output_hz = float(output_hz or config.arm.control_hz)
        self.target_timeout_s = float(target_timeout_s)
        self.joint_names = list(joint_names or config.joint_names)
        self.mode = mode
        self.tracking_kp = float(tracking_kp)
        self.tracking_feedforward = float(tracking_feedforward)
        self._validate_settings()

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._output: dict[str, float] = {}
        self._velocities: dict[str, float] = {name: 0.0 for name in self.joint_names}
        self._target: dict[str, float] | None = None
        self._target_velocities: dict[str, float] = {name: 0.0 for name in self.joint_names}
        self._target_at: float | None = None
        self._writes = 0
        self._overruns = 0
        self._stale = True
        self._error: str | None = None

    def start(self) -> "JointStreamingController":
        with self._lock:
            if self.running:
                return self
            current = self.motion.read_positions_rad()
            self._output = {
                name: float(current[name])
                for name in self.joint_names
                if name in current
            }
            missing = set(self.joint_names) - set(self._output)
            if missing:
                names = ", ".join(sorted(missing))
                raise MotionError(f"Cannot start joint stream; missing current joints: {names}")
            self._velocities = {name: 0.0 for name in self.joint_names}
            self._target = None
            self._target_velocities = {name: 0.0 for name in self.joint_names}
            self._target_at = None
            self._writes = 0
            self._overruns = 0
            self._stale = True
            self._error = None
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="soarm-joint-stream",
                daemon=True,
            )
            self._thread.start()
        return self

    def stop(self, *, timeout: float = 1.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout)))
        with self._lock:
            if self._thread is thread:
                self._thread = None

    def update_target(self, targets: Mapping[str, float]) -> None:
        self._raise_if_failed()
        clean = self._clean_targets(targets)
        now = time.monotonic()
        with self._lock:
            target_velocities = self._estimate_target_velocities(clean, now)
            self._target = clean
            self._target_velocities = target_velocities
            self._target_at = now
            self._stale = False

    def snapshot(self) -> JointStreamSnapshot:
        with self._lock:
            target_age = None if self._target_at is None else time.monotonic() - self._target_at
            return JointStreamSnapshot(
                running=self.running,
                mode=self.mode,
                output_hz=self.output_hz,
                target_timeout_s=self.target_timeout_s,
                writes=self._writes,
                overruns=self._overruns,
                stale=self._stale,
                target_age_s=target_age,
                output=dict(self._output),
                target=None if self._target is None else dict(self._target),
                target_velocities=dict(self._target_velocities),
                velocities=dict(self._velocities),
                error=self._error,
            )

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def __enter__(self) -> "JointStreamingController":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def _run(self) -> None:
        period = 1.0 / self.output_hz
        next_deadline = time.monotonic()
        last_tick = next_deadline
        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = max(1e-6, min(now - last_tick, period * 5.0))
            last_tick = now

            try:
                output, names = self._next_output(now, dt)
                if names:
                    self.safety.check_not_emergency_stopped()
                    self.motion.write_joint_setpoint(output, joint_names=names)
                    with self._lock:
                        self._writes += 1
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._error = str(exc)
                self._stop_event.set()
                break

            next_deadline += period
            sleep_time = next_deadline - time.monotonic()
            if sleep_time <= 0:
                with self._lock:
                    self._overruns += 1
                next_deadline = time.monotonic() + period
                sleep_time = period
            self._stop_event.wait(sleep_time)

    def _next_output(self, now: float, dt: float) -> tuple[dict[str, float], set[str]]:
        with self._lock:
            target = None if self._target is None else dict(self._target)
            target_velocities = dict(self._target_velocities)
            target_at = self._target_at
            output = dict(self._output)
            velocities = dict(self._velocities)

        if target is None or target_at is None:
            with self._lock:
                self._stale = True
            return output, set()

        stale = now - target_at > self.target_timeout_s
        if stale:
            with self._lock:
                for name in target:
                    self._velocities[name] = 0.0
                    self._target_velocities[name] = 0.0
                self._stale = True
            return output, set()

        names = set(target)
        next_output = dict(output)
        next_velocities = dict(velocities)
        write_names: set[str] = set()
        for name in names:
            position = float(output[name])
            wanted = float(target[name])
            velocity = float(velocities.get(name, 0.0))
            target_velocity = float(target_velocities.get(name, 0.0))
            next_position, next_velocity = self._step_joint(
                name,
                position,
                velocity,
                wanted,
                target_velocity,
                dt,
            )
            next_output[name] = next_position
            next_velocities[name] = next_velocity
            if abs(next_position - position) > 1e-12:
                write_names.add(name)

        with self._lock:
            self._output = next_output
            self._velocities = next_velocities
            self._stale = False
        return next_output, write_names

    def _step_joint(
        self,
        name: str,
        position: float,
        velocity: float,
        target: float,
        target_velocity: float,
        dt: float,
    ) -> tuple[float, float]:
        if self.mode == "direct":
            self.config.joints[name].check_limit(target)
            return target, 0.0
        if self.mode == "tracking":
            return self._step_tracking_joint(name, position, velocity, target, target_velocity, dt)
        return self._step_arrival_joint(name, position, velocity, target, dt)

    def _step_arrival_joint(
        self,
        name: str,
        position: float,
        velocity: float,
        target: float,
        dt: float,
    ) -> tuple[float, float]:
        joint = self.config.joints[name]
        error = target - position
        if abs(error) <= 1e-9:
            return target, 0.0

        direction = 1.0 if error > 0 else -1.0
        stop_limited_velocity = math.sqrt(max(0.0, 2.0 * joint.max_acc_rad_s2 * abs(error)))
        desired_velocity = direction * min(joint.max_vel_rad_s, stop_limited_velocity)
        max_delta_velocity = joint.max_acc_rad_s2 * dt
        next_velocity = _move_toward(velocity, desired_velocity, max_delta_velocity)
        if abs(next_velocity) > joint.max_vel_rad_s:
            next_velocity = math.copysign(joint.max_vel_rad_s, next_velocity)

        step = next_velocity * dt
        if abs(step) >= abs(error) or step * error < 0:
            return target, 0.0

        next_position = position + step
        limited_position = max(joint.min_rad, min(joint.max_rad, next_position))
        if limited_position != next_position:
            next_position = limited_position
            next_velocity = 0.0
        joint.check_limit(next_position)
        return next_position, next_velocity

    def _step_tracking_joint(
        self,
        name: str,
        position: float,
        velocity: float,
        target: float,
        target_velocity: float,
        dt: float,
    ) -> tuple[float, float]:
        joint = self.config.joints[name]
        error = target - position
        if abs(error) <= 1e-9 and abs(target_velocity) <= 1e-9:
            return target, 0.0

        target_velocity = max(
            -joint.max_vel_rad_s,
            min(joint.max_vel_rad_s, float(target_velocity)),
        )
        desired_velocity = (
            self.tracking_feedforward * target_velocity
            + self.tracking_kp * error
        )
        desired_velocity = max(
            -joint.max_vel_rad_s,
            min(joint.max_vel_rad_s, desired_velocity),
        )

        max_delta_velocity = joint.max_acc_rad_s2 * dt
        next_velocity = _move_toward(velocity, desired_velocity, max_delta_velocity)
        if abs(next_velocity) > joint.max_vel_rad_s:
            next_velocity = math.copysign(joint.max_vel_rad_s, next_velocity)

        step = next_velocity * dt
        if abs(target_velocity) <= 1e-6 and (abs(step) >= abs(error) or step * error < 0):
            return target, 0.0

        next_position = position + step
        joint.check_limit(next_position)
        return next_position, next_velocity

    def _estimate_target_velocities(self, clean: dict[str, float], now: float) -> dict[str, float]:
        target_velocities = {name: 0.0 for name in self.joint_names}
        if self.mode != "tracking" or self._target is None or self._target_at is None:
            return target_velocities

        dt = now - self._target_at
        if dt <= 1e-6:
            return target_velocities

        for name, target in clean.items():
            previous = self._target.get(name)
            if previous is None:
                continue
            joint = self.config.joints[name]
            velocity = (float(target) - float(previous)) / dt
            target_velocities[name] = max(
                -joint.max_vel_rad_s,
                min(joint.max_vel_rad_s, velocity),
            )
        return target_velocities

    def _clean_targets(self, targets: Mapping[str, float]) -> dict[str, float]:
        if not targets:
            raise MotionError("joint stream target must contain at least one joint")
        unknown = set(targets) - set(self.config.joints)
        if unknown:
            names = ", ".join(sorted(unknown))
            raise ConfigurationError(f"Unknown joints: {names}")
        outside_stream = set(targets) - set(self.joint_names)
        if outside_stream:
            names = ", ".join(sorted(outside_stream))
            raise ConfigurationError(f"Joints are not part of this stream: {names}")
        clean: dict[str, float] = {}
        for name, value in targets.items():
            position = float(value)
            if not math.isfinite(position):
                raise MotionError(f"joint stream target {name} must be finite")
            self.config.joints[name].check_limit(position)
            clean[str(name)] = position
        return clean

    def _validate_settings(self) -> None:
        if self.output_hz <= 0:
            raise MotionError("output_hz must be positive")
        if self.target_timeout_s <= 0:
            raise MotionError("target_timeout_s must be positive")
        if self.mode not in {"arrival", "tracking", "direct"}:
            raise MotionError("joint stream mode must be one of: arrival, tracking, direct")
        if self.tracking_kp < 0:
            raise MotionError("tracking_kp must be non-negative")
        if self.tracking_feedforward < 0:
            raise MotionError("tracking_feedforward must be non-negative")
        unknown = set(self.joint_names) - set(self.config.joints)
        if unknown:
            names = ", ".join(sorted(unknown))
            raise ConfigurationError(f"Unknown stream joints: {names}")

    def _raise_if_failed(self) -> None:
        with self._lock:
            error = self._error
        if error is not None:
            raise MotionError(f"joint stream failed: {error}")


def _move_toward(current: float, target: float, step: float) -> float:
    if step <= 0:
        return target
    if current < target:
        return min(target, current + step)
    return max(target, current - step)
