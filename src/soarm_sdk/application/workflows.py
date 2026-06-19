from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from ..calibration import build_joint_calibration_from_direction
from ..config import SOARMConfig
from ..kinematics import forward_kinematics, home_positions, solve_position_ik
from ..model import JointState
from ..safety.limits import minimum_motion_duration


def config_payload(config_path: Path, config: SOARMConfig) -> dict[str, Any]:
    frequencies = {
        **config.frequencies.to_dict(),
        "control_hz": config.arm.control_hz,
        "serial_write_hz": config.arm.control_hz,
    }
    return {
        "config_path": str(config_path),
        "config_files": {
            "main": str(config.source.main_path or config_path),
            "runtime": None if config.source.runtime_path is None else str(config.source.runtime_path),
            "motor_profile": None
            if config.source.motor_profile_path is None
            else str(config.source.motor_profile_path),
        },
        "arm": {
            "name": config.arm.name,
            "port": config.arm.port,
            "baudrate": config.arm.baudrate,
            "control_hz": config.arm.control_hz,
            "low_voltage": config.arm.low_voltage,
            "auto_disable": config.arm.auto_disable,
        },
        "frequencies": frequencies,
        "motor_profile": config.motor_profile.to_dict(),
        "calibration": config.calibration.to_dict(),
        "joints": [
            {
                "name": name,
                "id": joint.id,
                "zero_tick": joint.zero_tick,
                "direction": joint.direction,
                "min_rad": joint.min_rad,
                "max_rad": joint.max_rad,
                "max_vel_rad_s": joint.max_vel_rad_s,
                "max_acc_rad_s2": joint.max_acc_rad_s2,
            }
            for name, joint in config.joints.items()
        ],
        "poses": {name: pose.to_dict() for name, pose in config.poses.items()},
    }


def is_calibrated(config: SOARMConfig, *, session_calibrated: bool = False) -> bool:
    return bool(config.calibration.calibrated or session_calibrated)


def workflow_payload(
    config: SOARMConfig,
    *,
    status_passed: bool = False,
    calibration_ready: bool = False,
    session_calibrated: bool = False,
) -> dict[str, Any]:
    calibrated = is_calibrated(config, session_calibrated=session_calibrated)
    status_passed = bool(status_passed)
    calibration_ready = bool(status_passed or calibration_ready)
    control_ready = bool(calibrated and status_passed)
    if control_ready:
        phase = "control"
    elif calibration_ready:
        phase = "calibration"
    else:
        phase = "status"
    return {
        "phase": phase,
        "status_passed": status_passed,
        "calibration_ready": calibration_ready,
        "calibrated": calibrated,
        "control_ready": control_ready,
    }


def clamp_to_joint_limits(config: SOARMConfig, positions: Mapping[str, float]) -> dict[str, float]:
    clamped: dict[str, float] = {}
    for name, joint in config.joints.items():
        value = float(positions.get(name, 0.0))
        value = max(float(joint.min_rad), min(float(joint.max_rad), value))
        clamped[name] = value
    return clamped


def model_joint_positions(
    config: SOARMConfig,
    positions: Mapping[str, float] | None = None,
) -> dict[str, float]:
    """Convert SDK joint radians into the bundled SO-ARM101 URDF joint convention.

    SDK joint values are calibrated around each physical servo direction.  The
    bundled URDF is authored in the servo/CAD joint-axis convention, so joints
    whose calibrated direction is reversed need their sign flipped for visual
    playback.  Keeping this conversion in the SDK layer prevents front ends from
    duplicating robot-specific data rules.
    """
    merged = home_positions(config)
    if positions:
        merged.update({str(name): float(value) for name, value in positions.items()})

    visual: dict[str, float] = {}
    for name, value in merged.items():
        joint = config.joints.get(name)
        if joint is None:
            continue
        visual[name] = float(value) * joint.direction
    return visual


def recommended_move_duration(
    config: SOARMConfig,
    *,
    current: Mapping[str, float],
    target: Mapping[str, float],
    minimum: float = 0.2,
) -> float:
    current_for_targets = {
        str(name): float(current.get(name, 0.0))
        for name in target
    }
    return max(minimum, minimum_motion_duration(config, current_for_targets, target))


def apply_sweep_calibration(
    config: SOARMConfig,
    sweep_results: Any,
) -> tuple[SOARMConfig, list[dict[str, Any]]]:
    if not isinstance(sweep_results, list):
        raise ValueError("sweep_results must be a list")

    result_by_name: dict[str, Mapping[str, Any]] = {}
    for raw_result in sweep_results:
        if not isinstance(raw_result, Mapping):
            raise ValueError("Each sweep result must be an object")
        result_by_name[str(raw_result["name"])] = raw_result

    calibration_values: dict[str, dict[str, Any]] = {}
    summary: list[dict[str, Any]] = []
    for name in config.joints:
        result = result_by_name.get(name)
        if result is None:
            raise ValueError(f"Missing sweep result for joint {name!r}")

        calibration = build_joint_calibration_from_direction(
            name=name,
            zero_tick=int(result["zero_tick"]),
            direction=int(result.get("inferred_direction", config.joints[name].direction)),
            first_limit_tick=int(result["safe_min_tick"]),
            second_limit_tick=int(result["safe_max_tick"]),
        )
        calibration_values[name] = calibration.to_config_values()
        summary.append(
            {
                "name": name,
                "zero_tick": calibration.zero_tick,
                "positive_tick": calibration.positive_tick,
                "direction": calibration.direction,
                "min_tick": calibration.min_tick,
                "max_tick": calibration.max_tick,
                "min_rad": calibration.min_rad,
                "max_rad": calibration.max_rad,
            }
        )

    return config.replace_joint_calibrations(calibration_values), summary


def robot_model_state(config: SOARMConfig) -> dict[str, Any]:
    home = home_positions(config)
    positions = clamp_to_joint_limits(config, home)
    return {
        "name": "so_arm101",
        "joint_order": config.joint_names,
        "model_joints": model_joint_positions(config, positions),
        "fk": forward_kinematics(config, positions),
    }


def joint_state_payload(
    config: SOARMConfig,
    states: Mapping[str, JointState],
    *,
    mock: bool,
) -> dict[str, Any]:
    fk_positions = home_positions(config)
    fk_positions.update(
        {
            name: state.position_rad
            for name, state in states.items()
            if state.position_rad is not None
        }
    )
    return {
        "mock": mock,
        "joints": {
            name: {
                "id": state.id,
                "online": state.online,
                "position_tick": state.position_tick,
                "position_rad": state.position_rad,
                "voltage": state.voltage,
            }
            for name, state in states.items()
        },
        "model_joints": model_joint_positions(config, fk_positions),
        "fk": forward_kinematics(config, clamp_to_joint_limits(config, fk_positions)),
    }


def move_response_payload(
    config: SOARMConfig,
    *,
    mock: bool,
    synced: bool,
    duration: float | None,
    targets: Mapping[str, float],
    states: Mapping[str, JointState],
    lines: list[str],
) -> dict[str, Any]:
    positions = {
        name: state.position_rad
        for name, state in states.items()
        if state.position_rad is not None
    }
    display_positions = home_positions(config)
    display_positions.update(positions)
    if not synced or not positions:
        display_positions.update({str(name): float(value) for name, value in targets.items()})
    fk = forward_kinematics(config, display_positions)
    return {
        "mock": mock,
        "synced": synced,
        "duration": duration,
        "targets": dict(targets),
        "joints": {
            name: {
                "id": state.id,
                "online": state.online,
                "position_tick": state.position_tick,
                "position_rad": state.position_rad,
                "voltage": state.voltage,
            }
            for name, state in states.items()
        },
        "model_joints": model_joint_positions(config, display_positions),
        "fk": fk,
        "lines": lines,
    }


def fk_payload(config: SOARMConfig, positions: Mapping[str, float] | None = None) -> dict[str, Any]:
    resolved = positions or home_positions(config)
    payload = forward_kinematics(config, resolved)
    payload["model_joints"] = model_joint_positions(config, resolved)
    return payload


def ik_payload(
    config: SOARMConfig,
    target: Mapping[str, float],
    *,
    elbow: str = "down",
) -> dict[str, Any]:
    payload = solve_position_ik(config, target, elbow=elbow)
    payload["model_joints"] = model_joint_positions(config, payload.get("positions", {}))
    return payload
