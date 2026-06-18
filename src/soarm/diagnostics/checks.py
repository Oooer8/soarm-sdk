from __future__ import annotations

from ..config import SOARMConfig


GREEN = "\033[32m"
RED = "\033[31m"
RESET = "\033[0m"


def _pass(text: str) -> str:
    return f"{GREEN}[PASS] {text}{RESET}"


def _fail(text: str) -> str:
    return f"{RED}[FAIL] {text}{RESET}"


def calibration_ready_from_report(lines: list[str]) -> bool:
    for line in lines:
        clean = line.replace(GREEN, "").replace(RED, "").replace(RESET, "")
        if clean.startswith("calibration readiness:"):
            return "[PASS]" in clean
    return not any("[FAIL]" in line for line in lines)


def run_basic_checks(config: SOARMConfig, bus) -> list[str]:
    lines: list[str] = []

    positions = bus.read_positions(config.servo_ids, strict=False)
    voltages = bus.read_voltages(config.servo_ids, strict=False)

    missing: list[str] = []
    low_voltage: list[str] = []
    voltage_unavailable: list[str] = []
    invalid_position: list[str] = []
    soft_limit_violations: list[str] = []

    lines.append("SOARM status report")
    lines.append("what it does:")
    lines.append("  1. Connect to the configured servo bus.")
    lines.append("  2. Read each configured servo ID once to confirm communication.")
    lines.append("  3. Read current position ticks and convert them to radians.")
    lines.append("  4. Check current positions against configured soft limits.")
    lines.append("  5. Read voltage and compare it with the low-voltage threshold.")
    lines.append("  6. Close the bus; auto_disable=true disables torque during shutdown.")
    lines.append("safety: this command reads state only; it does not enable torque or send movement targets.")
    lines.append("phase note: the SDK phase check runs during connection; phase 0 is the expected value.")
    lines.append(
        "config: "
        f"joints={len(config.joints)}, "
        f"baudrate={config.arm.baudrate}, "
        f"low_voltage={config.arm.low_voltage:.2f}V, "
        f"auto_disable={str(config.arm.auto_disable).lower()}"
    )
    lines.append("meaning: online means the servo returned position data for its configured ID.")
    lines.append("joint checks:")

    for joint_name, joint in config.joints.items():
        tick = positions.get(joint.id)
        voltage = voltages.get(joint.id)

        if tick is None:
            communication = _fail("offline, no position response from this servo ID")
            position = _fail("unavailable")
            limit = _fail("not checked because position is unavailable")
            missing.append(joint_name)
        else:
            communication = _pass("online, position response received")
            try:
                position_rad = joint.tick_to_rad(tick)
            except ValueError as exc:
                position = f"{tick} ticks"
                limit = _fail(f"invalid raw position: {exc}")
                invalid_position.append(f"{joint_name}=invalid raw tick {tick}")
            else:
                position = _pass(f"{tick} ticks / {position_rad:.4f} rad")
                if joint.min_rad <= position_rad <= joint.max_rad:
                    limit = _pass(f"within [{joint.min_rad:.4f}, {joint.max_rad:.4f}] rad")
                else:
                    limit = _fail(f"outside [{joint.min_rad:.4f}, {joint.max_rad:.4f}] rad")
                    soft_limit_violations.append(
                        f"{joint_name}={position_rad:.4f} rad outside "
                        f"[{joint.min_rad:.4f}, {joint.max_rad:.4f}]"
                    )

        if voltage is None:
            voltage_text = _fail("unavailable")
            voltage_unavailable.append(joint_name)
        else:
            if voltage < config.arm.low_voltage:
                voltage_text = _fail(
                    f"{voltage:.2f}V below threshold {config.arm.low_voltage:.2f}V"
                )
                low_voltage.append(f"{joint_name}={voltage:.2f}V")
            else:
                voltage_text = _pass(
                    f"{voltage:.2f}V at or above threshold {config.arm.low_voltage:.2f}V"
                )

        lines.append(f"- {joint_name} (id={joint.id})")
        lines.append(f"  communication: {communication}")
        lines.append(f"  position: {position}")
        lines.append(f"  soft_limit: {limit}")
        lines.append(f"  voltage: {voltage_text}")

    if missing or low_voltage or voltage_unavailable or invalid_position or soft_limit_violations:
        lines.append("summary: " + _fail("one or more checks failed"))
        if missing:
            lines.append("summary: missing joints: " + ", ".join(missing))
        if voltage_unavailable:
            lines.append("summary: voltage unavailable: " + ", ".join(voltage_unavailable))
        if low_voltage:
            lines.append("summary: low voltage: " + ", ".join(low_voltage))
        if invalid_position:
            lines.append("summary: invalid raw positions: " + "; ".join(invalid_position))
        if soft_limit_violations:
            lines.append(
                "summary: position outside configured soft limits: "
                + "; ".join(soft_limit_violations)
            )
    else:
        lines.append("summary: " + _pass("all checks passed"))
    if missing or low_voltage or voltage_unavailable or invalid_position:
        lines.append(
            "calibration readiness: "
            + _fail("hardware is not ready for calibration; fix communication, voltage, or raw position errors first")
        )
    else:
        if soft_limit_violations:
            message = (
                "communication, position reads, and voltage are OK; "
                "soft-limit failures can be fixed by calibration"
            )
        else:
            message = "all required hardware checks are ready for calibration"
        lines.append("calibration readiness: " + _pass(message))
    lines.append(
        "shutdown: after this report the bus is closed; "
        "with auto_disable=true the SDK disables servo torque once before closing the port."
    )

    return lines
