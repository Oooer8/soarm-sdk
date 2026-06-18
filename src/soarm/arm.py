from __future__ import annotations

from pathlib import Path
from typing import Callable, Mapping, Sequence

from .calibration import (
    AllJointsRangeRecorder,
    build_joint_calibration_from_direction,
    capture_current_pose,
    capture_zero_ticks,
)
from .config import SOARMConfig
from .diagnostics import run_basic_checks
from .errors import CalibrationError, ConfigurationError, UnsupportedFeature
from .hardware import ServoBus
from .kinematics import forward_kinematics, home_positions, solve_position_ik
from .model import ArmState, JointState
from .hardware.motor_profile import (
    MotorProfileResult,
    apply_motor_profile,
    motor_profile_requires_torque_disabled,
)
from .motion import InterpolationMode, MotionController, TimedJointTrajectory, TrajectoryPoint
from .safety import SafetyGuard
from .testing import MockBus


class SOARM:
    """User-facing API for controlling a SOARM."""

    def __init__(
        self,
        config: SOARMConfig,
        *,
        bus: ServoBus | MockBus | None = None,
        mock: bool = False,
    ) -> None:
        self.config = config
        self._config_path: Path | None = None
        self.bus = bus or self._make_bus(config, mock=mock)
        self.safety = SafetyGuard(config)
        self.motion = MotionController(config=config, bus=self.bus, safety=self.safety)
        self._enabled = False
        self._motor_profile_applied = False

    @classmethod
    def from_config(cls, path: str | Path, *, mock: bool = False) -> "SOARM":
        config = SOARMConfig.from_file(path)
        arm = cls(config, mock=mock)
        arm._config_path = Path(path)
        return arm

    @classmethod
    def mock(cls, config: SOARMConfig | str | Path = "configs/soarm.yaml") -> "SOARM":
        if not isinstance(config, SOARMConfig):
            config = SOARMConfig.from_file(config)
        return cls(config, mock=True)

    def _make_bus(self, config: SOARMConfig, *, mock: bool) -> ServoBus | MockBus:
        if mock:
            return MockBus(config)
        return ServoBus(
            servo_ids=config.servo_ids,
            port=config.arm.port,
            baudrate=config.arm.baudrate,
            auto_disable=config.arm.auto_disable,
        )

    def __enter__(self) -> "SOARM":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    @property
    def connected(self) -> bool:
        return self.bus.connected

    @property
    def enabled(self) -> bool:
        return self._enabled

    def connect(self) -> None:
        self.bus.connect()

    def disconnect(self) -> None:
        self.bus.disconnect()
        self._enabled = False
        self._motor_profile_applied = False

    def enable(self) -> None:
        if not self.connected:
            self.connect()
        self.safety.check_not_emergency_stopped()
        if self.config.motor_profile.apply_on_enable and not self._motor_profile_applied:
            self.apply_motor_profile()
        self.bus.read_positions()
        self._enabled = True

    def apply_motor_profile(
        self,
        *,
        force: bool = False,
        check_only: bool = False,
    ) -> list[MotorProfileResult]:
        if not self.connected:
            self.connect()
        if motor_profile_requires_torque_disabled(
            self.config.motor_profile,
            force=force,
            check_only=check_only,
        ):
            self.bus.disable_torque()
            self._enabled = False
        results = apply_motor_profile(
            self.bus,
            profile=self.config.motor_profile,
            joints=self.config.joints,
            force=force,
            check_only=check_only,
        )
        if not check_only:
            self._motor_profile_applied = True
        return results

    def disable(self) -> None:
        if self.connected:
            self.bus.disable_all()
        self._enabled = False

    def stop(self) -> None:
        self.motion.stop()

    def emergency_stop(self) -> None:
        self.safety.set_emergency_stop()
        if self.connected:
            self.bus.disable_all()
        self._enabled = False

    def reset_emergency_stop(self) -> None:
        self.safety.reset_emergency_stop()

    def scan(self, ids: list[int] | None = None) -> dict[int, bool]:
        if not self.connected:
            self.connect()
        return self.bus.scan(ids)

    def diagnostics(self) -> list[str]:
        if not self.connected:
            self.connect()
        return run_basic_checks(self.config, self.bus)

    def get_joint_positions(self, unit: str = "rad") -> dict[str, float | int]:
        ticks = self.bus.read_positions()
        positions: dict[str, float | int] = {}
        for name, joint in self.config.joints.items():
            tick = ticks[joint.id]
            if unit == "tick":
                positions[name] = tick
            elif unit == "rad":
                positions[name] = joint.tick_to_rad(tick)
            elif unit == "deg":
                positions[name] = joint.tick_to_deg(tick)
            else:
                raise ValueError("unit must be one of: rad, deg, tick")
        return positions

    def get_joint_states(self) -> dict[str, JointState]:
        ticks = self.bus.read_positions(strict=False)
        voltages = self.bus.read_voltages(strict=False)
        states: dict[str, JointState] = {}
        for name, joint in self.config.joints.items():
            tick = ticks.get(joint.id)
            voltage = voltages.get(joint.id)
            states[name] = JointState(
                name=name,
                id=joint.id,
                position_tick=tick,
                position_rad=None if tick is None else joint.tick_to_rad(tick),
                voltage=voltage,
                online=tick is not None,
            )
        return states

    def get_arm_state(self) -> ArmState:
        return ArmState(
            connected=self.connected,
            enabled=self.enabled,
            emergency_stopped=self.safety.emergency_stopped,
            joints=self.get_joint_states() if self.connected else {},
        )

    def forward_kinematics(self, positions: Mapping[str, float] | None = None) -> dict:
        return forward_kinematics(self.config, positions or home_positions(self.config))

    def solve_position_ik(self, target: Mapping[str, float], *, elbow: str = "down") -> dict:
        return solve_position_ik(self.config, target, elbow=elbow)

    def move_joint(
        self,
        name: str,
        position_rad: float,
        *,
        duration: float = 1.0,
        wait: bool = True,
    ) -> None:
        self.move_joints({name: position_rad}, duration=duration, wait=wait)

    def move_joints(
        self,
        targets: Mapping[str, float],
        *,
        duration: float = 1.0,
        wait: bool = True,
    ) -> None:
        if not self._enabled:
            self.enable()
        self.motion.move_joints(targets, duration=duration, wait=wait)

    def stream_joints(self, targets: Mapping[str, float], *, dt: float | None = None) -> None:
        if not self._enabled:
            self.enable()
        self.motion.stream_joints(targets, dt=dt)

    def follow_joint_trajectory(
        self,
        points: Sequence[TrajectoryPoint] | TimedJointTrajectory,
        *,
        speed: float = 1.0,
        output_hz: float | None = None,
        interpolation: InterpolationMode = "pchip",
    ) -> int:
        if not self._enabled:
            self.enable()
        return self.motion.follow_timed_trajectory(
            points,
            speed=speed,
            output_hz=output_hz,
            interpolation=interpolation,
        )

    def move_pose(self, name: str, *, duration: float = 1.0, wait: bool = True) -> None:
        pose = self.config.get_pose(name)
        self.move_joints(pose.joints, duration=duration, wait=wait)

    def move_home(self, *, duration: float = 1.0, wait: bool = True) -> None:
        self.move_pose("home", duration=duration, wait=wait)

    def set_gripper(self, position_rad: float, *, duration: float = 0.5, wait: bool = True) -> None:
        if "gripper" not in self.config.joints:
            raise ConfigurationError("Config does not define a gripper joint")
        self.move_joint("gripper", position_rad, duration=duration, wait=wait)

    def calibrate(
        self,
        *,
        output_path: str | Path | None = None,
        prompt: Callable[[str], None] | None = None,
        announce: Callable[[str], None] | None = None,
        margin_ticks: int = 0,
        motion_threshold: int = 100,
        filter_window: int = 3,
    ) -> SOARMConfig:
        """Run interactive joint calibration using a single dynamic sweep.

        Calibration flow (4 steps)
        --------------------------
        1. Disable servo torque so the arm can be back-driven freely.
        2. Place every joint at the physical zero pose; press Enter to record.
        3. Freely sweep every joint through its safe workspace; press Enter
           when done.
           - All joints are recorded **concurrently** in one pass.
           - A median filter (window=*filter_window*) rejects encoder glitches.
           - Existing joint directions are preserved from the config.
           - Optional margins (*margin_ticks*) can be subtracted from both extremes.
           - Under-excited joints (range < *motion_threshold* ticks) trigger a
             warning; calibration proceeds with whatever range was observed.
        4. Write the updated config to disk.

        Parameters
        ----------
        output_path:
            Destination YAML file.  Defaults to the file the arm was loaded from.
        prompt:
            Callable used to pause and wait for the user (e.g. :func:`input`).
        announce:
            Callable used to print progress messages (e.g. :func:`print`).
        margin_ticks:
            Optional padding applied to both soft-limit extremes (default 0 ticks).
        motion_threshold:
            Minimum observed tick range for a joint to be considered
            "well-excited" (default 100 ticks ≈ 8.8°).
        filter_window:
            Size of the median filter window used to reject encoder glitches
            (default 3; odd values recommended).
        """
        if not self.connected:
            self.connect()

        prompt = prompt or _default_calibration_prompt
        announce = announce or (lambda _message: None)

        # ------------------------------------------------------------------
        # Step 1: disable torque
        # ------------------------------------------------------------------
        announce("Calibration step 1/4: disabling servo torque for manual movement.")
        self.disable()

        # ------------------------------------------------------------------
        # Step 2: record zero pose
        # ------------------------------------------------------------------
        announce("Calibration step 2/4: recording the physical zero pose.")
        prompt(
            "Place every joint in the physical zero pose, then press Enter to record zero ticks: "
        )
        zero_ticks = capture_zero_ticks(self.config, self.bus)
        announce(
            "  Zero ticks: "
            + ", ".join(f"{n}={t}" for n, t in zero_ticks.items())
        )

        # ------------------------------------------------------------------
        # Step 3: dynamic sweep — all joints, single pass
        # ------------------------------------------------------------------
        announce("Calibration step 3/4: dynamic sweep of all joints.")
        announce(
            f"  Noise filter: median window={filter_window} | "
            f"Limit margin: {margin_ticks} ticks | "
            f"Motion threshold: {motion_threshold} ticks"
        )
        recorder = AllJointsRangeRecorder(
            self.config,
            self.bus,
            zero_ticks,
            filter_window=filter_window,
            motion_threshold=motion_threshold,
            margin_ticks=margin_ticks,
        )
        recorder.start()
        prompt(
            "Freely sweep the full range of motion for each joint. "
            "Press Enter when done: "
        )
        sweep_results = recorder.stop()

        # ------------------------------------------------------------------
        # Validate, report, and build calibrations
        # ------------------------------------------------------------------
        calibrations: dict = {}
        under_excited: list[str] = []

        for name, result in sweep_results.items():
            status = "" if result.well_excited else "  ⚠ UNDER-EXCITED"
            announce(
                f"  {name}: direction={result.inferred_direction:+d} | "
                f"raw=[{result.raw_min_tick}, {result.raw_max_tick}] ticks "
                f"(range={result.range_ticks}) | "
                f"safe=[{result.safe_min_rad:.4f}, {result.safe_max_rad:.4f}] rad"
                f"{status}"
            )
            if not result.well_excited:
                under_excited.append(name)

            calibration = build_joint_calibration_from_direction(
                name=name,
                zero_tick=result.zero_tick,
                direction=result.inferred_direction,
                first_limit_tick=result.safe_min_tick,
                second_limit_tick=result.safe_max_tick,
            )
            calibrations[name] = calibration.to_config_values()

        if under_excited:
            announce(
                "  WARNING: the following joints had insufficient motion during the sweep "
                f"(< {motion_threshold} ticks): {', '.join(under_excited)}. "
                "Consider re-calibrating those joints."
            )

        # ------------------------------------------------------------------
        # Step 4: apply and save
        # ------------------------------------------------------------------
        self.config = self.config.replace_joint_calibrations(calibrations)
        self.safety = SafetyGuard(self.config)
        self.motion = MotionController(config=self.config, bus=self.bus, safety=self.safety)
        output = Path(output_path) if output_path is not None else self._config_path
        if output is None:
            raise CalibrationError("No output path provided for calibrated config")
        self.config.save(output)
        announce(f"Calibration step 4/4: saved calibrated config to {output}.")
        return self.config

    def capture_pose(self, name: str, *, output_path: str | Path | None = None) -> SOARMConfig:
        if not self.connected:
            self.connect()
        positions = capture_current_pose(self.config, self.bus)
        self.config = self.config.replace_pose(name, positions)
        output = Path(output_path) if output_path is not None else self._config_path
        if output is None:
            raise CalibrationError("No output path provided for captured pose")
        self.config.save(output)
        return self.config

    def set_joint_torque(self, *_args, **_kwargs) -> None:
        raise UnsupportedFeature("SOARM SDK exposes position-based motion control only")


def _default_calibration_prompt(message: str) -> None:
    try:
        input(message)
    except EOFError as exc:
        raise CalibrationError("Calibration requires interactive stdin") from exc
