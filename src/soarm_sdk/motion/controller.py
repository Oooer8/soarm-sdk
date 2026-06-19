from __future__ import annotations

import time
from typing import Mapping, Sequence

from ..config import SOARMConfig
from ..errors import HardwareError, LimitViolation, MotionError
from ..motion.trajectory import InterpolationMode, TimedJointTrajectory, TrajectoryPoint, linear_trajectory
from ..safety import SafetyGuard
from ..safety.limits import minimum_motion_duration


class MotionController:
    def __init__(self, *, config: SOARMConfig, bus, safety: SafetyGuard) -> None:
        self.config = config
        self.bus = bus
        self.safety = safety

    def read_positions_rad(self) -> dict[str, float]:
        ticks = self.bus.read_positions()
        return {
            name: joint.tick_to_rad(ticks[joint.id])
            for name, joint in self.config.joints.items()
            if ticks.get(joint.id) is not None
        }

    def move_joints(
        self,
        targets: Mapping[str, float],
        *,
        duration: float = 1.0,
        wait: bool = True,
    ) -> None:
        """Move joints using either blocking interpolation or a single setpoint write.

        ``wait=True`` is the point-to-point path: it interpolates from the
        current pose to the target at ``arm.control_hz`` and returns after all
        setpoints are written.

        ``wait=False`` is the legacy streaming path: it validates the command
        using ``duration`` as the caller's intended command interval, writes one
        setpoint, and returns immediately.
        """
        current = self.read_positions_rad()
        merged = dict(current)
        for name, position in targets.items():
            merged[name] = float(position)

        voltages = self.bus.read_voltages(strict=False)
        self.safety.validate_motion(
            current=current,
            target=merged,
            moving_joints=set(targets),
            duration=duration,
            voltages=voltages,
        )

        moving_joints = set(targets)

        if not wait or duration == 0:
            self._write_joint_positions(
                merged,
                joint_names=moving_joints,
            )
            return

        points = linear_trajectory(
            current,
            merged,
            duration=duration,
            hz=self.config.arm.control_hz,
        )
        start_time = time.monotonic()
        for point in points:
            self._write_joint_positions(
                point.positions,
                joint_names=moving_joints,
            )
            deadline = start_time + point.time_from_start
            sleep_time = deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stream_joints(self, targets: Mapping[str, float], *, dt: float | None = None) -> None:
        """Write one streaming joint target.

        ``dt`` represents the caller's control period and is used for safety
        checks. It does not block, interpolate, or set Feetech ``Goal_Time``.
        """
        control_dt = 1.0 / float(self.config.arm.control_hz) if dt is None else float(dt)
        self.move_joints(targets, duration=max(control_dt, 1e-6), wait=False)

    def follow_timed_trajectory(
        self,
        points: Sequence[TrajectoryPoint] | TimedJointTrajectory,
        *,
        speed: float = 1.0,
        output_hz: float | None = None,
        interpolation: InterpolationMode = "pchip",
    ) -> int:
        """Replay sparse timed joint points at a fixed controller output rate.

        The sparse input points can come from a 20 Hz demonstration, a 30 Hz
        teleop loop, or a 60 Hz policy. This method evaluates that trajectory at
        ``output_hz`` and writes the interpolated setpoints at that fixed rate.
        """
        speed = float(speed)
        if speed <= 0:
            raise MotionError("speed must be positive")
        write_hz = float(output_hz or self.config.arm.control_hz)
        if write_hz <= 0:
            raise MotionError("output_hz must be positive")

        if isinstance(points, TimedJointTrajectory):
            trajectory = points
        else:
            trajectory = TimedJointTrajectory(points, interpolation=interpolation)
        output_points = trajectory.fixed_rate_points(speed=speed, output_hz=write_hz)
        self._validate_timed_trajectory(trajectory, output_points=output_points)

        moving_joints = set(trajectory.joint_names)
        start_time = time.monotonic()
        writes = 0
        for point in output_points:
            deadline = start_time + point.time_from_start
            sleep_time = deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            self.safety.check_not_emergency_stopped()
            self._write_joint_positions(point.positions, joint_names=moving_joints)
            writes += 1
        return writes

    def stop(self) -> None:
        ticks = self.bus.read_positions()
        positions = {servo_id: tick for servo_id, tick in ticks.items() if tick is not None}
        if positions:
            self.bus.write_positions(positions)

    def _write_joint_positions(
        self,
        positions_rad: Mapping[str, float],
        *,
        joint_names: set[str] | None = None,
    ) -> None:
        names = joint_names or set(positions_rad)
        ticks: dict[int, int] = {}
        for name in names:
            joint = self.config.joints[name]
            position = positions_rad[name]
            ticks[joint.id] = joint.rad_to_tick(float(position))
        if not ticks:
            return
        result = self.bus.write_positions(ticks)
        failed = [servo_id for servo_id, ok in result.items() if not ok]
        if failed:
            raise HardwareError(f"Position write failed for servos: {failed}")

    def _validate_timed_trajectory(
        self,
        trajectory: TimedJointTrajectory,
        *,
        output_points: Sequence[TrajectoryPoint],
    ) -> None:
        current = self.read_positions_rad()
        voltages = self.bus.read_voltages(strict=False)
        moving_joints = set(trajectory.joint_names)
        first_target = trajectory.points[0].positions
        first_duration = max(
            1.0 / float(self.config.arm.control_hz),
            minimum_motion_duration(self.config, current, first_target, moving_joints),
            1e-6,
        )
        self.safety.validate_motion(
            current=current,
            target=first_target,
            moving_joints=moving_joints,
            duration=first_duration,
            voltages=voltages,
        )

        for point in trajectory.points:
            for name, position in point.positions.items():
                self.config.joints[name].check_limit(float(position))

        for previous, point in zip(output_points, output_points[1:]):
            for name in moving_joints:
                delta = abs(float(point.positions[name]) - float(previous.positions[name]))
                if delta > self.config.arm.max_step_rad:
                    raise LimitViolation(
                        f"{name} interpolated step {delta:.4f} rad exceeds max_step_rad "
                        f"{self.config.arm.max_step_rad:.4f}; increase output_hz or reduce speed"
                    )
