from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path

from soarm.config import SOARMConfig
from soarm.hardware import ServoBus
from soarm.testing import MockBus


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * q)))
    return ordered[index]


def summarize(label: str, seconds: list[float]) -> None:
    if not seconds:
        print(f"{label}: no samples")
        return
    mean = statistics.fmean(seconds)
    median = statistics.median(seconds)
    p95 = percentile(seconds, 0.95)
    print(f"{label}: samples={len(seconds)}")
    print(f"  mean:   {mean * 1000:.3f} ms  ({1.0 / mean:.1f} Hz)")
    print(f"  median: {median * 1000:.3f} ms  ({1.0 / median:.1f} Hz)")
    print(f"  p95:    {p95 * 1000:.3f} ms  ({1.0 / p95:.1f} Hz)")
    print(f"  min:    {min(seconds) * 1000:.3f} ms")
    print(f"  max:    {max(seconds) * 1000:.3f} ms")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Benchmark one SOARM ServoBus.write_positions() call by repeatedly "
            "writing the current servo ticks back to the same servos."
        )
    )
    parser.add_argument("--config", default="configs/soarm.yaml")
    parser.add_argument("--mock", action="store_true", help="Use the in-memory mock bus")
    parser.add_argument("--iterations", type=int, default=50)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0,
        help="Optional sleep between measured writes, in seconds",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.iterations <= 0:
        raise SystemExit("--iterations must be positive")
    if args.warmup < 0:
        raise SystemExit("--warmup must be non-negative")
    if args.sleep < 0:
        raise SystemExit("--sleep must be non-negative")

    config = SOARMConfig.from_file(Path(args.config))
    bus = MockBus(config) if args.mock else ServoBus(
        servo_ids=config.servo_ids,
        port=config.arm.port,
        baudrate=config.arm.baudrate,
        auto_disable=config.arm.auto_disable,
    )

    print(f"config: {args.config}")
    print(f"mode: {'mock' if args.mock else 'hardware'}")
    print(f"servo_ids: {config.servo_ids}")
    print(f"baudrate: {config.arm.baudrate}")
    print(f"iterations: {args.iterations}, warmup: {args.warmup}")
    if not args.mock:
        print("safety: writing current ticks back to the servos; no new target is generated.")

    bus.connect()
    try:
        current = bus.read_positions()
        positions = {
            servo_id: tick
            for servo_id, tick in current.items()
            if tick is not None
        }
        if not positions:
            raise SystemExit("No servo positions available to write")
        print("positions:", ", ".join(f"{servo_id}={tick}" for servo_id, tick in positions.items()))

        warmup_times: list[float] = []
        for _ in range(args.warmup):
            start = time.perf_counter()
            result = bus.write_positions(positions)
            warmup_times.append(time.perf_counter() - start)
            if not all(result.values()):
                raise SystemExit(f"Warmup write failed: {result}")

        measured: list[float] = []
        for _ in range(args.iterations):
            start = time.perf_counter()
            result = bus.write_positions(positions)
            measured.append(time.perf_counter() - start)
            if not all(result.values()):
                raise SystemExit(f"Measured write failed: {result}")
            if args.sleep:
                time.sleep(args.sleep)
    finally:
        bus.disconnect()

    summarize("warmup write_positions", warmup_times)
    summarize("measured write_positions", measured)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
