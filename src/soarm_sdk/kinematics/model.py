from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from ..config import SOARMConfig
from ..errors import LimitViolation, MotionError


SOARM_JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@dataclass(frozen=True)
class LinkGeometry:
    base_height: float = 0.075
    shoulder_height: float = 0.125
    upper_arm: float = 0.160
    forearm: float = 0.160
    wrist: float = 0.075
    tool: float = 0.065


GEOMETRY = LinkGeometry()
Matrix4 = list[list[float]]


def home_positions(config: SOARMConfig) -> dict[str, float]:
    if "home" in config.poses:
        return dict(config.poses["home"].joints)
    return {name: 0.0 for name in config.joints}


def _identity() -> Matrix4:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply(a: Matrix4, b: Matrix4) -> Matrix4:
    return [
        [sum(a[row][inner] * b[inner][col] for inner in range(4)) for col in range(4)]
        for row in range(4)
    ]


def _translate(x: float, y: float, z: float) -> Matrix4:
    matrix = _identity()
    matrix[0][3] = x
    matrix[1][3] = y
    matrix[2][3] = z
    return matrix


def _rotate_x(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotate_y(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotate_z(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _position(matrix: Matrix4) -> list[float]:
    return [matrix[0][3], matrix[1][3], matrix[2][3]]


def _rounded_matrix(matrix: Matrix4) -> Matrix4:
    return [[round(value, 6) for value in row] for row in matrix]


def _clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def _validate_targets(config: SOARMConfig, positions: Mapping[str, float]) -> dict[str, float]:
    merged = home_positions(config)
    for name, value in positions.items():
        if name not in config.joints:
            raise MotionError(f"Unknown joint {name!r}")
        joint = config.joints[name]
        joint.check_limit(float(value))
        merged[name] = float(value)
    return merged


def forward_kinematics(
    config: SOARMConfig,
    positions: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    q = _validate_targets(config, positions or home_positions(config))
    frames: list[dict[str, Any]] = []

    def record(name: str, matrix: Matrix4) -> None:
        frames.append(
            {
                "name": name,
                "position": [round(value, 6) for value in _position(matrix)],
                "matrix": _rounded_matrix(matrix),
            }
        )

    transform = _identity()
    record("world", transform)

    transform = _multiply(transform, _translate(0.0, 0.0, GEOMETRY.base_height))
    transform = _multiply(transform, _rotate_z(q.get("shoulder_pan", 0.0)))
    record("shoulder_pan", transform)

    transform = _multiply(transform, _translate(0.0, 0.0, GEOMETRY.shoulder_height))
    transform = _multiply(transform, _rotate_y(q.get("shoulder_lift", 0.0)))
    record("shoulder_lift", transform)

    transform = _multiply(transform, _translate(GEOMETRY.upper_arm, 0.0, 0.0))
    transform = _multiply(transform, _rotate_y(q.get("elbow_flex", 0.0)))
    record("elbow_flex", transform)

    transform = _multiply(transform, _translate(GEOMETRY.forearm, 0.0, 0.0))
    transform = _multiply(transform, _rotate_y(q.get("wrist_flex", 0.0)))
    record("wrist_flex", transform)

    transform = _multiply(transform, _translate(GEOMETRY.wrist, 0.0, 0.0))
    transform = _multiply(transform, _rotate_x(q.get("wrist_roll", 0.0)))
    record("wrist_roll", transform)

    transform = _multiply(transform, _translate(GEOMETRY.tool, 0.0, 0.0))
    record("tool0", transform)

    return {
        "joint_order": [name for name in SOARM_JOINT_ORDER if name in config.joints],
        "positions": {name: round(q[name], 6) for name in config.joints},
        "frames": frames,
        "end_effector": {
            "frame": "tool0",
            "position": frames[-1]["position"],
            "matrix": frames[-1]["matrix"],
        },
    }


def solve_position_ik(
    config: SOARMConfig,
    target: Mapping[str, float],
    *,
    elbow: str = "down",
) -> dict[str, Any]:
    try:
        x = float(target["x"])
        y = float(target["y"])
        z = float(target["z"])
    except KeyError as exc:
        raise MotionError("IK target must include x, y, and z") from exc

    pan = math.atan2(y, x)
    reach_xy = math.hypot(x, y)
    wrist_offset = GEOMETRY.wrist + GEOMETRY.tool
    planar_x = max(0.0, reach_xy - wrist_offset)
    planar_z = z - GEOMETRY.base_height - GEOMETRY.shoulder_height

    l1 = GEOMETRY.upper_arm
    l2 = GEOMETRY.forearm
    distance = math.hypot(planar_x, planar_z)
    max_reach = l1 + l2
    min_reach = abs(l1 - l2)
    reachable = min_reach <= distance <= max_reach
    safe_distance = _clamp(distance, min_reach + 1e-6, max_reach - 1e-6)

    cos_elbow = _clamp((safe_distance**2 - l1**2 - l2**2) / (2.0 * l1 * l2), -1.0, 1.0)
    elbow_angle = math.acos(cos_elbow)
    if elbow == "up":
        elbow_angle = -elbow_angle

    shoulder = math.atan2(planar_z, planar_x) - math.atan2(
        l2 * math.sin(elbow_angle),
        l1 + l2 * math.cos(elbow_angle),
    )
    wrist = -(shoulder + elbow_angle)

    solution = home_positions(config)
    candidates = {
        "shoulder_pan": pan,
        "shoulder_lift": shoulder,
        "elbow_flex": elbow_angle,
        "wrist_flex": wrist,
    }
    violations: list[str] = []
    for name, value in candidates.items():
        if name not in config.joints:
            continue
        try:
            config.joints[name].check_limit(value)
            solution[name] = value
        except LimitViolation as exc:
            violations.append(str(exc))
            joint = config.joints[name]
            solution[name] = _clamp(value, joint.min_rad, joint.max_rad)

    return {
        "reachable": reachable and not violations,
        "violations": violations,
        "positions": {name: round(solution[name], 6) for name in config.joints},
        "fk": forward_kinematics(config, solution),
    }
