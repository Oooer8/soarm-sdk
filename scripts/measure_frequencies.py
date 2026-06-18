from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path
from typing import Any, Mapping

from soarm.config import SOARMConfig
from soarm.hardware import ServoBus
from soarm.motion import MotionController
from soarm.safety import SafetyGuard
from soarm.testing import MockBus


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * q)))
    return ordered[index]


def summarize_duration(label: str, seconds: list[float]) -> None:
    if not seconds:
        print(f"{label}: no samples")
        return
    mean = statistics.fmean(seconds)
    median = statistics.median(seconds)
    p95 = percentile(seconds, 0.95)
    print(f"{label}: samples={len(seconds)}")
    print(f"  mean:   {mean * 1000:.3f} ms  ({1 / mean:.1f} Hz)")
    print(f"  median: {median * 1000:.3f} ms  ({1 / median:.1f} Hz)")
    print(f"  p95:    {p95 * 1000:.3f} ms  ({1 / p95:.1f} Hz)")
    print(f"  min:    {min(seconds) * 1000:.3f} ms")
    print(f"  max:    {max(seconds) * 1000:.3f} ms")


def summarize_intervals(label: str, starts: list[float]) -> None:
    intervals = [b - a for a, b in zip(starts, starts[1:])]
    summarize_duration(label, intervals)


def drain_serial(bus: Any) -> None:
    port_handler = getattr(bus, "_port_handler", None)
    serial_port = getattr(port_handler, "ser", None)
    if serial_port is not None and hasattr(serial_port, "flush"):
        serial_port.flush()


class RecordingBus:
    def __init__(self, bus: Any, *, drain: bool = False) -> None:
        self._bus = bus
        self.drain = drain
        self.write_starts: list[float] = []
        self.write_durations: list[float] = []

    def __getattr__(self, name: str) -> Any:
        return getattr(self._bus, name)

    def write_positions(self, positions: Mapping[int, int], **kwargs) -> dict[int, bool]:
        start = time.perf_counter()
        result = self._bus.write_positions(positions, **kwargs)
        if self.drain:
            drain_serial(self._bus)
        end = time.perf_counter()
        self.write_starts.append(start)
        self.write_durations.append(end - start)
        return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Measure SOARM bus/control frequencies.")
    parser.add_argument("--config", default="configs/soarm.yaml")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--controller-duration", type=float, default=2.0)
    parser.add_argument(
        "--drain",
        action="store_true",
        help="Call pyserial flush() after write calls to include output-buffer drain time.",
    )
    return parser.parse_args()


def make_bus(config: SOARMConfig, *, mock: bool):
    if mock:
        return MockBus(config)
    return ServoBus(
        servo_ids=config.servo_ids,
        port=config.arm.port,
        baudrate=config.arm.baudrate,
        auto_disable=config.arm.auto_disable,
    )


def current_rad_positions(config: SOARMConfig, ticks: Mapping[int, int | None]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for name, joint in config.joints.items():
        tick = ticks.get(joint.id)
        if tick is not None:
            positions[name] = joint.tick_to_rad(tick)
    return positions


def main() -> int:
    args = parse_args()
    if args.iterations <= 0:
        raise SystemExit("--iterations must be positive")
    if args.warmup < 0:
        raise SystemExit("--warmup must be non-negative")
    if args.controller_duration <= 0:
        raise SystemExit("--controller-duration must be positive")

    config = SOARMConfig.from_file(Path(args.config))
    bus = make_bus(config, mock=args.mock)
    print(f"config: {args.config}")
    print(f"mode: {'mock' if args.mock else 'hardware'}")
    print(f"servo_ids: {config.servo_ids}")
    print(f"baudrate: {config.arm.baudrate}")
    print(f"control_hz: {config.arm.control_hz}")
    print(f"iterations: {args.iterations}, warmup: {args.warmup}, drain={args.drain}")
    if not args.mock:
        print("safety: writes send current ticks back to the servos; controller test uses zero displacement.")

    bus.connect()
    try:
        current = bus.read_positions()
        positions = {servo_id: tick for servo_id, tick in current.items() if tick is not None}
        if not positions:
            raise SystemExit("No servo positions available")
        print("positions:", ", ".join(f"{servo_id}={tick}" for servo_id, tick in positions.items()))

        for _ in range(args.warmup):
            bus.write_positions(positions)
            if args.drain:
                drain_serial(bus)

        write_times: list[float] = []
        for _ in range(args.iterations):
            start = time.perf_counter()
            result = bus.write_positions(positions)
            if args.drain:
                drain_serial(bus)
            write_times.append(time.perf_counter() - start)
            if not all(result.values()):
                raise SystemExit(f"write_positions failed: {result}")
        summarize_duration("raw trajectory point input / serial write_positions", write_times)

        for _ in range(args.warmup):
            bus.read_positions()
        read_times: list[float] = []
        for _ in range(args.iterations):
            start = time.perf_counter()
            bus.read_positions()
            read_times.append(time.perf_counter() - start)
        summarize_duration("feedback read_positions", read_times)

        target_rad = current_rad_positions(config, bus.read_positions())
        recording_bus = RecordingBus(bus, drain=args.drain)
        controller = MotionController(
            config=config,
            bus=recording_bus,
            safety=SafetyGuard(config),
        )
        expected_points = max(1, round(args.controller_duration * config.arm.control_hz))
        print(
            "controller zero-displacement trajectory: "
            f"duration={args.controller_duration:.3f}s, expected_points~={expected_points}"
        )
        start = time.perf_counter()
        controller.move_joints(target_rad, duration=args.controller_duration, wait=True)
        elapsed = time.perf_counter() - start
        print(f"controller elapsed: {elapsed:.3f}s")
        print(f"controller writes: {len(recording_bus.write_starts)}")
        summarize_intervals("controller interpolation write interval", recording_bus.write_starts)
        summarize_duration("controller write_positions call time", recording_bus.write_durations)
    finally:
        bus.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
