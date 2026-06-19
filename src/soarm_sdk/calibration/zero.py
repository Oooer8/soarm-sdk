from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from statistics import median
from typing import Any

from ..constants import MAX_POSITION_TICK, MIN_POSITION_TICK
from ..config import SOARMConfig
from ..errors import CalibrationError
from ..hardware.units import ticks_delta_to_rad


@dataclass(frozen=True)
class JointCalibration:
    name: str
    zero_tick: int
    positive_tick: int
    direction: int
    min_tick: int
    max_tick: int
    min_rad: float
    max_rad: float

    def to_config_values(self) -> dict[str, Any]:
        return {
            "zero_tick": self.zero_tick,
            "direction": self.direction,
            "min_rad": self.min_rad,
            "max_rad": self.max_rad,
        }


@dataclass(frozen=True)
class JointRangeResult:
    """Calibration sweep result for one joint produced by :class:`AllJointsRangeRecorder`.

    Attributes
    ----------
    name:
        Joint name.
    zero_tick:
        Encoder tick at the physical zero pose.
    raw_min_tick / raw_max_tick:
        Observed extremes during the sweep, *before* applying safety margins.
    safe_min_tick / safe_max_tick:
        Limits after applying ``margin_ticks``; zero tick is always contained.
    inferred_direction:
        Direction used to convert ticks into calibrated radians.  The dynamic
        sweep uses the configured joint direction so the user can move freely
        during range recording.
    range_ticks:
        ``raw_max_tick - raw_min_tick``; the total observed excursion.
    well_excited:
        ``True`` when ``range_ticks >= motion_threshold``.
    safe_min_rad / safe_max_rad:
        Safe limits in radians (always satisfies ``min_rad <= 0 <= max_rad``).
    """

    name: str
    zero_tick: int
    raw_min_tick: int
    raw_max_tick: int
    safe_min_tick: int
    safe_max_tick: int
    inferred_direction: int
    range_ticks: int
    well_excited: bool
    safe_min_rad: float
    safe_max_rad: float


def capture_zero_ticks(config: SOARMConfig, bus) -> dict[str, int]:
    ticks = bus.read_positions()
    zero_ticks: dict[str, int] = {}
    missing: list[str] = []
    for name, joint in config.joints.items():
        tick = ticks.get(joint.id)
        if tick is None:
            missing.append(name)
        else:
            zero_ticks[name] = int(tick)
    if missing:
        raise CalibrationError(f"Cannot capture zero ticks; missing joints: {', '.join(missing)}")
    return zero_ticks


def _validate_direction(direction: int) -> int:
    direction = int(direction)
    if direction not in (-1, 1):
        raise CalibrationError("direction must be 1 or -1")
    return direction


def tick_to_calibrated_rad(tick: int, *, zero_tick: int, direction: int) -> float:
    tick = _validate_tick("joint", tick, "position")
    zero_tick = _validate_tick("joint", zero_tick, "zero")
    direction = _validate_direction(direction)
    return direction * ticks_delta_to_rad(tick - zero_tick)


def _adjacent_tick_for_direction(zero_tick: int, direction: int) -> int:
    adjacent = zero_tick + direction
    if MIN_POSITION_TICK <= adjacent <= MAX_POSITION_TICK:
        return adjacent
    return zero_tick


def build_joint_calibration_from_direction(
    *,
    name: str,
    zero_tick: int,
    direction: int,
    first_limit_tick: int,
    second_limit_tick: int,
    positive_tick: int | None = None,
) -> JointCalibration:
    zero_tick = _validate_tick(name, zero_tick, "zero")
    direction = _validate_direction(direction)
    if positive_tick is None:
        positive_tick = _adjacent_tick_for_direction(zero_tick, direction)
    else:
        positive_tick = _validate_tick(name, positive_tick, "positive")

    first_limit_tick = _validate_tick(name, first_limit_tick, "first limit")
    second_limit_tick = _validate_tick(name, second_limit_tick, "second limit")
    if first_limit_tick == second_limit_tick:
        raise CalibrationError(
            f"Cannot set soft limits for {name!r}; both limit samples are {first_limit_tick}."
        )

    limits = [
        (
            tick_to_calibrated_rad(first_limit_tick, zero_tick=zero_tick, direction=direction),
            first_limit_tick,
        ),
        (
            tick_to_calibrated_rad(second_limit_tick, zero_tick=zero_tick, direction=direction),
            second_limit_tick,
        ),
    ]
    limits.sort(key=lambda item: item[0])
    min_rad, min_tick = limits[0]
    max_rad, max_tick = limits[1]
    if min_rad >= max_rad:
        raise CalibrationError(f"Cannot set soft limits for {name!r}; min_rad >= max_rad")
    if not min_rad <= 0.0 <= max_rad:
        raise CalibrationError(
            f"Soft limits for {name!r} must contain the calibrated zero pose; "
            f"computed [{min_rad:.4f}, {max_rad:.4f}] rad."
        )

    return JointCalibration(
        name=name,
        zero_tick=int(zero_tick),
        positive_tick=int(positive_tick),
        direction=direction,
        min_tick=min_tick,
        max_tick=max_tick,
        min_rad=min_rad,
        max_rad=max_rad,
    )


@dataclass
class AllJointsRangeRecorder:
    """Records min/max encoder limits for **all** joints in a single free-form sweep.

    State machine
    -------------
    The caller is expected to use this class as follows::

        recorder = AllJointsRangeRecorder(config, bus, zero_ticks)
        recorder.start()
        # user freely back-drives the arm …
        results: dict[str, JointRangeResult] = recorder.stop()

    Noise filtering
    ---------------
    Each joint maintains a sliding window of the last *filter_window* raw
    readings.  The **median** of that window is used to update the running
    min/max, which rejects single-frame encoder glitches without introducing
    the lag of a moving average.

    Direction handling
    -------------------
    The sweep preserves each joint's configured ``direction``.  It does not
    infer direction from the first movement, so the user can freely move each
    joint through its available range while recording.

    Optional margins
    --------------
    After recording, both extremes can be contracted inward by *margin_ticks*::

        safe_min_tick = raw_min_tick + margin_ticks
        safe_max_tick = raw_max_tick - margin_ticks

    The default margin is 0, so the saved range matches the observed raw
    min/max.  The zero tick is always guaranteed to remain within
    ``[safe_min, safe_max]``.

    Under-excitation detection
    --------------------------
    A joint is flagged as *not well-excited* when its total observed range
    (``raw_max_tick - raw_min_tick``) is less than *motion_threshold* ticks.
    """

    config: SOARMConfig
    bus: Any
    zero_ticks: dict[str, int]
    poll_interval: float = 0.02          # 50 Hz
    filter_window: int = 3               # median filter; odd values recommended
    motion_threshold: int = 100          # minimum total excursion to be "well excited"
    margin_ticks: int = 0                # optional padding ticks applied to both limits

    # -- internal per-joint state (initialised in __post_init__) -------------
    _min_ticks: dict[str, int] = field(init=False, default_factory=dict)
    _max_ticks: dict[str, int] = field(init=False, default_factory=dict)
    _buffers: dict[str, deque[int]] = field(init=False, default_factory=dict)
    _stop_event: threading.Event = field(init=False, default_factory=threading.Event)
    _thread: threading.Thread | None = field(init=False, default=None)

    def __post_init__(self) -> None:
        if self.poll_interval <= 0:
            raise CalibrationError("poll_interval must be positive")
        if self.filter_window <= 0:
            raise CalibrationError("filter_window must be positive")
        if self.motion_threshold < 0:
            raise CalibrationError("motion_threshold must be non-negative")
        if self.margin_ticks < 0:
            raise CalibrationError("margin_ticks must be non-negative")

        missing = [name for name in self.config.joints if name not in self.zero_ticks]
        if missing:
            raise CalibrationError(
                "Cannot start range recording; missing zero ticks for: "
                + ", ".join(missing)
            )

        normalized_zero_ticks: dict[str, int] = {}
        for name in self.config.joints:
            zero = _validate_tick(name, self.zero_ticks[name], "zero")
            normalized_zero_ticks[name] = zero
            self._min_ticks[name] = zero
            self._max_ticks[name] = zero
            self._buffers[name] = deque(maxlen=max(1, self.filter_window))
        self.zero_ticks = normalized_zero_ticks

    def start(self) -> None:
        """Start the background polling thread (non-blocking)."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self) -> None:
        joint_ids: dict[str, int] = {
            name: joint.id for name, joint in self.config.joints.items()
        }

        while not self._stop_event.is_set():
            try:
                raw = self.bus.read_positions()
            except Exception:  # noqa: BLE001
                time.sleep(self.poll_interval)
                continue

            for name, joint_id in joint_ids.items():
                tick = raw.get(joint_id)
                if tick is None:
                    continue
                tick = int(tick)

                # ---- Noise filter: median of sliding window -----------------
                buf = self._buffers[name]
                buf.append(tick)
                filtered = int(median(buf))

                # ---- Peak tracking ------------------------------------------
                if filtered < self._min_ticks[name]:
                    self._min_ticks[name] = filtered
                if filtered > self._max_ticks[name]:
                    self._max_ticks[name] = filtered

            time.sleep(self.poll_interval)

    def stop(self) -> dict[str, JointRangeResult]:
        """Stop recording and return a :class:`JointRangeResult` per joint.

        Individual joints that are under-excited are reported via
        :attr:`JointRangeResult.well_excited` rather than raising.
        """
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        results: dict[str, JointRangeResult] = {}
        for name in self.config.joints:
            zero_tick = self.zero_ticks[name]
            raw_min = self._min_ticks[name]
            raw_max = self._max_ticks[name]
            range_ticks = raw_max - raw_min

            # ---- Direction handling -----------------------------------------
            direction = self.config.joints[name].direction

            # ---- Optional margins (always keep zero_tick inside range) --------
            safe_min = max(MIN_POSITION_TICK, raw_min + self.margin_ticks)
            safe_max = min(MAX_POSITION_TICK, raw_max - self.margin_ticks)
            # Guarantee zero containment regardless of how tight the margin is
            safe_min = min(safe_min, zero_tick)
            safe_max = max(safe_max, zero_tick)

            # ---- Convert safe limits to radians (always sorted) -------------
            rad_a = tick_to_calibrated_rad(safe_min, zero_tick=zero_tick, direction=direction)
            rad_b = tick_to_calibrated_rad(safe_max, zero_tick=zero_tick, direction=direction)
            safe_min_rad = min(rad_a, rad_b)
            safe_max_rad = max(rad_a, rad_b)

            # ---- Under-excitation check -------------------------------------
            well_excited = range_ticks >= self.motion_threshold

            results[name] = JointRangeResult(
                name=name,
                zero_tick=zero_tick,
                raw_min_tick=raw_min,
                raw_max_tick=raw_max,
                safe_min_tick=safe_min,
                safe_max_tick=safe_max,
                inferred_direction=direction,
                range_ticks=range_ticks,
                well_excited=well_excited,
                safe_min_rad=safe_min_rad,
                safe_max_rad=safe_max_rad,
            )

        return results


def capture_current_pose(config: SOARMConfig, bus) -> dict[str, float]:
    ticks = bus.read_positions()
    positions: dict[str, float] = {}
    missing: list[str] = []
    for name, joint in config.joints.items():
        tick = ticks.get(joint.id)
        if tick is None:
            missing.append(name)
        else:
            positions[name] = joint.tick_to_rad(int(tick))
    if missing:
        raise CalibrationError(f"Cannot capture pose; missing joints: {', '.join(missing)}")
    return positions


def _validate_tick(name: str, tick: int, label: str) -> int:
    tick = int(tick)
    if not MIN_POSITION_TICK <= tick <= MAX_POSITION_TICK:
        raise CalibrationError(
            f"{name!r} {label} tick {tick} is outside "
            f"[{MIN_POSITION_TICK}, {MAX_POSITION_TICK}]"
        )
    return tick
