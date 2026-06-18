from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, Literal, Mapping, Sequence

from ..errors import MotionError

InterpolationMode = Literal["linear", "pchip"]


@dataclass(frozen=True)
class TrajectoryPoint:
    time_from_start: float
    positions: dict[str, float]


def linear_trajectory(
    start: Mapping[str, float],
    target: Mapping[str, float],
    *,
    duration: float,
    hz: int,
) -> list[TrajectoryPoint]:
    if duration < 0:
        raise MotionError("duration must be non-negative")
    if hz <= 0:
        raise MotionError("hz must be positive")
    if set(start) != set(target):
        raise MotionError("start and target must contain the same joints")
    if duration == 0:
        return [TrajectoryPoint(time_from_start=0.0, positions=dict(target))]

    steps = max(1, int(math.ceil(duration * hz)))
    points: list[TrajectoryPoint] = []
    for index in range(1, steps + 1):
        alpha = index / steps
        positions = {
            name: float(start[name]) + (float(target[name]) - float(start[name])) * alpha
            for name in start
        }
        points.append(TrajectoryPoint(time_from_start=duration * alpha, positions=positions))
    return points


class TimedJointTrajectory:
    """Evaluate sparse joint targets as a continuous, time-indexed trajectory."""

    def __init__(
        self,
        points: Sequence[TrajectoryPoint],
        *,
        interpolation: InterpolationMode = "pchip",
    ) -> None:
        if not points:
            raise MotionError("timed trajectory must contain at least one point")
        if interpolation not in {"linear", "pchip"}:
            raise MotionError("interpolation must be one of: linear, pchip")
        self.points = tuple(
            TrajectoryPoint(
                time_from_start=float(point.time_from_start),
                positions={name: float(value) for name, value in point.positions.items()},
            )
            for point in points
        )
        self.interpolation = interpolation
        self._times = tuple(point.time_from_start for point in self.points)
        self._joint_names = tuple(self.points[0].positions)
        self._validate()
        self._pchip_slopes = self._build_pchip_slopes() if interpolation == "pchip" else {}

    @classmethod
    def from_mappings(
        cls,
        samples: Iterable[tuple[float, Mapping[str, float]]],
        *,
        interpolation: InterpolationMode = "pchip",
    ) -> "TimedJointTrajectory":
        return cls(
            [
                TrajectoryPoint(
                    time_from_start=float(time_from_start),
                    positions={name: float(value) for name, value in positions.items()},
                )
                for time_from_start, positions in samples
            ],
            interpolation=interpolation,
        )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    @property
    def start_time(self) -> float:
        return self._times[0]

    @property
    def end_time(self) -> float:
        return self._times[-1]

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    def evaluate(self, time_from_start: float) -> dict[str, float]:
        query_time = float(time_from_start)
        if query_time <= self.start_time or len(self.points) == 1:
            return dict(self.points[0].positions)
        if query_time >= self.end_time:
            return dict(self.points[-1].positions)

        upper = bisect.bisect_right(self._times, query_time)
        lower = max(0, upper - 1)
        start = self.points[lower]
        end = self.points[upper]
        span = end.time_from_start - start.time_from_start
        if span <= 0:
            return dict(end.positions)
        alpha = (query_time - start.time_from_start) / span
        if self.interpolation == "pchip":
            return self._evaluate_pchip(lower, span, alpha)
        return {
            name: start.positions[name] + (end.positions[name] - start.positions[name]) * alpha
            for name in self._joint_names
        }

    def fixed_rate_points(self, *, speed: float, output_hz: float) -> list[TrajectoryPoint]:
        speed = float(speed)
        output_hz = float(output_hz)
        if speed <= 0:
            raise MotionError("speed must be positive")
        if output_hz <= 0:
            raise MotionError("output_hz must be positive")
        if self.duration <= 0:
            return [TrajectoryPoint(time_from_start=0.0, positions=dict(self.points[-1].positions))]

        playback_duration = self.duration / speed
        steps = max(1, int(math.ceil(playback_duration * output_hz)))
        points: list[TrajectoryPoint] = []
        for index in range(steps + 1):
            real_time = min(playback_duration, index / output_hz)
            demo_time = min(self.end_time, self.start_time + real_time * speed)
            points.append(
                TrajectoryPoint(
                    time_from_start=real_time,
                    positions=self.evaluate(demo_time),
                )
            )
        if points[-1].time_from_start < playback_duration:
            points.append(
                TrajectoryPoint(
                    time_from_start=playback_duration,
                    positions=self.evaluate(self.end_time),
                )
            )
        return points

    def _validate(self) -> None:
        previous_time: float | None = None
        joint_set = set(self._joint_names)
        if not joint_set:
            raise MotionError("timed trajectory points must contain at least one joint")
        for index, point in enumerate(self.points):
            if not math.isfinite(point.time_from_start):
                raise MotionError(f"trajectory point {index} time must be finite")
            if previous_time is not None and point.time_from_start < previous_time:
                raise MotionError("timed trajectory point times must be monotonic")
            if (
                self.interpolation == "pchip"
                and previous_time is not None
                and point.time_from_start <= previous_time
            ):
                raise MotionError("pchip interpolation requires strictly increasing point times")
            previous_time = point.time_from_start
            if set(point.positions) != joint_set:
                raise MotionError("all timed trajectory points must contain the same joints")
            for name, value in point.positions.items():
                if not math.isfinite(float(value)):
                    raise MotionError(f"trajectory point {index} position {name} must be finite")

    def _build_pchip_slopes(self) -> dict[str, list[float]]:
        return {
            name: _pchip_slopes(
                self._times,
                [point.positions[name] for point in self.points],
            )
            for name in self._joint_names
        }

    def _evaluate_pchip(self, lower: int, span: float, alpha: float) -> dict[str, float]:
        start = self.points[lower]
        end = self.points[lower + 1]
        alpha2 = alpha * alpha
        alpha3 = alpha2 * alpha
        h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0
        h10 = alpha3 - 2.0 * alpha2 + alpha
        h01 = -2.0 * alpha3 + 3.0 * alpha2
        h11 = alpha3 - alpha2
        return {
            name: (
                h00 * start.positions[name]
                + h10 * span * self._pchip_slopes[name][lower]
                + h01 * end.positions[name]
                + h11 * span * self._pchip_slopes[name][lower + 1]
            )
            for name in self._joint_names
        }


def _pchip_slopes(times: Sequence[float], values: Sequence[float]) -> list[float]:
    count = len(times)
    if count == 1:
        return [0.0]
    intervals = [float(times[index + 1]) - float(times[index]) for index in range(count - 1)]
    if any(interval <= 0 for interval in intervals):
        raise MotionError("pchip interpolation requires strictly increasing point times")
    deltas = [
        (float(values[index + 1]) - float(values[index])) / intervals[index]
        for index in range(count - 1)
    ]
    if count == 2:
        return [deltas[0], deltas[0]]

    slopes = [0.0] * count
    slopes[0] = _pchip_endpoint_slope(intervals[0], intervals[1], deltas[0], deltas[1])
    slopes[-1] = _pchip_endpoint_slope(intervals[-1], intervals[-2], deltas[-1], deltas[-2])

    for index in range(1, count - 1):
        previous_delta = deltas[index - 1]
        next_delta = deltas[index]
        if previous_delta == 0.0 or next_delta == 0.0 or _sign(previous_delta) != _sign(next_delta):
            slopes[index] = 0.0
            continue
        previous_interval = intervals[index - 1]
        next_interval = intervals[index]
        w1 = 2.0 * next_interval + previous_interval
        w2 = next_interval + 2.0 * previous_interval
        slopes[index] = (w1 + w2) / (w1 / previous_delta + w2 / next_delta)
    return slopes


def _pchip_endpoint_slope(
    first_interval: float,
    second_interval: float,
    first_delta: float,
    second_delta: float,
) -> float:
    numerator = (2.0 * first_interval + second_interval) * first_delta
    numerator -= first_interval * second_delta
    slope = numerator / (first_interval + second_interval)
    if slope == 0.0 or first_delta == 0.0 or _sign(slope) != _sign(first_delta):
        return 0.0
    if _sign(first_delta) != _sign(second_delta) and abs(slope) > abs(3.0 * first_delta):
        return 3.0 * first_delta
    return slope


def _sign(value: float) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0
