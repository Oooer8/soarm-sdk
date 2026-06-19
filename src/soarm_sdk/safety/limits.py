from __future__ import annotations

from typing import Iterable, Mapping

from ..config import SOARMConfig
from ..errors import ConfigurationError, LimitViolation


def minimum_rest_to_rest_duration(distance: float, max_vel: float, max_acc: float) -> float:
    distance = abs(float(distance))
    max_vel = float(max_vel)
    max_acc = float(max_acc)
    if distance <= 0:
        return 0.0
    if max_acc <= 0:
        return distance / max_vel if max_vel > 0 else 0.0
    if max_vel <= 0:
        return 2.0 * (distance / max_acc) ** 0.5

    accel_decel_distance = max_vel * max_vel / max_acc
    if distance <= accel_decel_distance:
        return 2.0 * (distance / max_acc) ** 0.5
    return distance / max_vel + max_vel / max_acc


def minimum_motion_duration(
    config: SOARMConfig,
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: Iterable[str] | None = None,
) -> float:
    """Return the longest rest-to-rest duration required by moving joints.

    This helper intentionally only computes a duration. Limit checks and
    unknown-joint errors stay in the callers that can report the best context.
    """
    duration = 0.0
    names = target.keys() if moving_joints is None else moving_joints
    for name in names:
        if name not in config.joints or name not in current or name not in target:
            continue
        joint = config.joints[name]
        duration = max(
            duration,
            minimum_rest_to_rest_duration(
                float(target[name]) - float(current[name]),
                joint.max_vel_rad_s,
                joint.max_acc_rad_s2,
            ),
        )
    return duration


def ensure_known_joints(config: SOARMConfig, targets: Mapping[str, float]) -> None:
    unknown = set(targets) - set(config.joints)
    if unknown:
        names = ", ".join(sorted(unknown))
        raise ConfigurationError(f"Unknown joints: {names}")


def ensure_joint_limits(config: SOARMConfig, targets: Mapping[str, float]) -> None:
    ensure_known_joints(config, targets)
    for name, position in targets.items():
        config.joints[name].check_limit(float(position))


def ensure_step_limits(
    config: SOARMConfig,
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: set[str],
) -> None:
    for name in moving_joints:
        if name not in current:
            continue
        delta = abs(float(target[name]) - float(current[name]))
        if delta > config.arm.max_step_rad:
            raise LimitViolation(
                f"{name} step {delta:.4f} rad exceeds max_step_rad "
                f"{config.arm.max_step_rad:.4f}; use a closer intermediate pose"
            )


def ensure_velocity_feasible(
    config: SOARMConfig,
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: set[str],
    duration: float,
) -> None:
    if duration <= 0:
        return
    for name in moving_joints:
        if name not in current:
            continue
        joint = config.joints[name]
        required = abs(float(target[name]) - float(current[name])) / duration
        if required > joint.max_vel_rad_s:
            raise LimitViolation(
                f"{name} requires {required:.4f} rad/s over {duration:.3f}s, "
                f"above max_vel_rad_s {joint.max_vel_rad_s:.4f}"
            )


def ensure_acceleration_feasible(
    config: SOARMConfig,
    current: Mapping[str, float],
    target: Mapping[str, float],
    moving_joints: set[str],
    duration: float,
) -> None:
    if duration <= 0:
        return
    for name in moving_joints:
        if name not in current:
            continue
        joint = config.joints[name]
        delta = abs(float(target[name]) - float(current[name]))
        minimum = minimum_rest_to_rest_duration(delta, joint.max_vel_rad_s, joint.max_acc_rad_s2)
        if duration + 1e-9 < minimum:
            raise LimitViolation(
                f"{name} requires at least {minimum:.3f}s for {delta:.4f} rad "
                f"with max_vel_rad_s {joint.max_vel_rad_s:.4f} and "
                f"max_acc_rad_s2 {joint.max_acc_rad_s2:.4f}; requested {duration:.3f}s"
            )
