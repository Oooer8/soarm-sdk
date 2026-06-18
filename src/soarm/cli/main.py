from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from ..arm import SOARM
from ..config import SOARMConfig
from ..demonstration import (
    DemonstrationRecorder,
    DemonstrationReplayer,
    load_demonstration,
)
from ..diagnostics import calibration_ready_from_report
from ..errors import SOARMError
from ..hardware import ServoBus
from ..kinematics import forward_kinematics, home_positions, solve_position_ik


SOARM_URDF_PATH = Path(__file__).resolve().parents[1] / "assets" / "soarm101" / "urdf" / "so_arm101.urdf"


def _add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", default="configs/soarm.yaml", help="Path to SOARM YAML config")
    parser.add_argument("--mock", action="store_true", help="Use the in-memory mock bus")


def _parse_targets(values: list[str]) -> dict[str, float]:
    targets: dict[str, float] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"Target must be JOINT=RAD, got {value!r}")
        name, raw = value.split("=", 1)
        targets[name] = float(raw)
    return targets


def _diagnostics_passed(lines: list[str]) -> bool:
    return not any("[FAIL]" in line for line in lines)


def _print_report(title: str, lines: list[str]) -> None:
    print(title)
    for line in lines:
        print(line)


def _joint_register_snapshot(arm: SOARM, joint_name: str, *, target_rad: float | None = None) -> dict:
    joint = arm.config.joints[joint_name]
    servo_id = int(joint.id)
    try:
        current_rad = arm.get_joint_positions(unit="rad")[joint_name]
    except Exception as exc:  # noqa: BLE001
        current_rad = f"ERR:{exc}"
    payload = {
        "joint": joint_name,
        "id": servo_id,
        "current_rad": current_rad,
        "target_rad": target_rad,
        "target_tick": None if target_rad is None else joint.rad_to_tick(float(target_rad)),
        "registers": {},
    }
    for register in (
        "Present_Position",
        "Goal_Position",
        "Torque_Enable",
        "Operating_Mode",
        "Moving",
        "Goal_Velocity",
        "Acceleration",
        "Torque_Limit",
        "Max_Torque_Limit",
        "Protection_Current",
        "Present_Load",
        "Present_Current",
        "Present_Voltage",
        "Present_Temperature",
        "Status",
    ):
        try:
            payload["registers"][register] = arm.bus.read_register(register, servo_id, raw=True)
        except Exception as exc:  # noqa: BLE001
            payload["registers"][register] = f"ERR:{exc}"
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="soarm")
    sub = parser.add_subparsers(dest="command", required=True)

    ports = sub.add_parser("ports", help="List candidate servo bus ports and update config")
    ports.add_argument("--config", default="configs/soarm.yaml", help="Path to SOARM YAML config to update")
    ports.add_argument("--port", default=None, help="Port to write when more than one candidate is found")
    ports.add_argument("--no-update", action="store_true", help="List ports without updating config")

    web = sub.add_parser("web", help="Serve the browser-based setup and status checker")
    web.add_argument("--config", default="configs/soarm.yaml", help="Path to SOARM YAML config")
    web.add_argument("--host", default="127.0.0.1", help="Host interface to bind")
    web.add_argument("--port", type=int, default=8765, help="HTTP port to bind")
    web.add_argument("--mock", action="store_true", help="Default status checks to the in-memory mock bus")

    urdf = sub.add_parser("urdf", help="Print the bundled SOARM URDF path or contents")
    urdf.add_argument("--print", action="store_true", help="Print the URDF XML instead of the path")

    status = sub.add_parser("status", help="Read joint state and run health checks")
    _add_common(status)

    configure_motors = sub.add_parser(
        "configure-motors",
        help="Check or apply the configured Feetech motor profile",
    )
    _add_common(configure_motors)
    configure_motors.add_argument(
        "--force",
        action="store_true",
        help="Write all configured motor profile registers instead of drift-only writes",
    )
    configure_motors.add_argument(
        "--check-only",
        action="store_true",
        help="Only report register drift; do not write anything",
    )

    fk = sub.add_parser("fk", help="Compute SOARM forward kinematics from JOINT=RAD targets")
    fk.add_argument("--config", default="configs/soarm.yaml", help="Path to SOARM YAML config")
    fk.add_argument("targets", nargs="*")

    ik = sub.add_parser("ik", help="Compute an approximate SOARM IK solution for a target point")
    ik.add_argument("--config", default="configs/soarm.yaml", help="Path to SOARM YAML config")
    ik.add_argument("x", type=float)
    ik.add_argument("y", type=float)
    ik.add_argument("z", type=float)
    ik.add_argument("--elbow", choices=["down", "up"], default="down")

    read = sub.add_parser("read", help="Read current joint positions")
    _add_common(read)
    read.add_argument("--unit", choices=["rad", "deg", "tick"], default="rad")

    home = sub.add_parser("home", help="Move to the home pose")
    _add_common(home)
    home.add_argument("--duration", type=float, default=1.5)

    move = sub.add_parser("move", help="Move joints using JOINT=RAD targets")
    _add_common(move)
    move.add_argument("targets", nargs="+")
    move.add_argument("--duration", type=float, default=1.0)
    move.add_argument("--no-wait", action="store_true")

    pose = sub.add_parser("pose", help="Move to a named pose")
    _add_common(pose)
    pose.add_argument("name")
    pose.add_argument("--duration", type=float, default=1.0)
    pose.add_argument("--no-wait", action="store_true")

    probe_joint = sub.add_parser(
        "probe-joint",
        help="Move one joint by a small amount and print low-level register snapshots",
    )
    _add_common(probe_joint)
    probe_joint.add_argument("name", help="Joint name to probe")
    probe_joint.add_argument("--delta", type=float, default=None, help="Relative move in radians")
    probe_joint.add_argument("--target", type=float, default=None, help="Absolute target in radians")
    probe_joint.add_argument("--duration", type=float, default=1.0, help="Move duration in seconds")
    probe_joint.add_argument("--settle", type=float, default=0.5, help="Seconds to wait after the move")

    record_demo = sub.add_parser(
        "record-demo",
        aliases=["record"],
        help="Record a manual joint-space demonstration",
    )
    _add_common(record_demo)
    record_demo.add_argument("output", help="Output demonstration JSON path")
    record_demo.add_argument("--duration", type=float, default=None, help="Recording duration in seconds")
    record_demo.add_argument("--hz", type=float, default=20.0, help="Joint sample frequency")
    record_demo.add_argument("--joints", nargs="+", default=None, help="Subset of joints to record")
    record_demo.add_argument(
        "--keep-torque",
        action="store_true",
        help="Do not disable torque before recording",
    )

    replay_demo = sub.add_parser(
        "replay-demo",
        aliases=["replay"],
        help="Replay a recorded joint-space demonstration",
    )
    _add_common(replay_demo)
    replay_demo.add_argument("input", help="Input demonstration JSON path")
    replay_demo.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    replay_demo.add_argument("--start-at", type=float, default=None, help="Replay from this timestamp")
    replay_demo.add_argument("--end-at", type=float, default=None, help="Replay through this timestamp")
    replay_demo.add_argument(
        "--lead-in-duration",
        type=float,
        default=2.0,
        help="Seconds used to move to the first replay sample",
    )
    replay_demo.add_argument(
        "--feedback-tolerance",
        type=float,
        default=0.03,
        help="Allowed readback error per joint in radians during replay",
    )
    replay_demo.add_argument(
        "--output-hz",
        type=float,
        default=None,
        help="Interpolated setpoint output frequency; defaults to arm.control_hz",
    )
    replay_demo.add_argument(
        "--interpolation",
        choices=["pchip", "linear"],
        default="pchip",
        help="Interpolation method used between demonstration samples",
    )
    replay_demo.add_argument(
        "--no-lead-in",
        action="store_true",
        help="Start replay immediately instead of moving to the first sample first",
    )
    replay_demo.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate the demonstration file without moving hardware",
    )

    calibrate = sub.add_parser("calibrate", help="Run full interactive joint calibration")
    _add_common(calibrate)
    calibrate.add_argument("--output", default=None, help="Output config path")

    capture = sub.add_parser("capture-pose", help="Save current joint positions as a named pose")
    _add_common(capture)
    capture.add_argument("name")
    capture.add_argument("--output", default=None, help="Output config path")

    disable = sub.add_parser("disable", help="Disable all servos")
    _add_common(disable)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        if args.command == "ports":
            found_ports = ServoBus.list_ports()
            if found_ports:
                for port in found_ports:
                    print(port)
            else:
                print("No candidate servo ports found")
                return 1

            if args.no_update:
                return 0

            selected_port = args.port
            if selected_port is None:
                if len(found_ports) != 1:
                    print(
                        "Multiple candidate servo ports found; "
                        "rerun with `--port <device>` to update the config.",
                        file=sys.stderr,
                    )
                    return 1
                selected_port = found_ports[0]
            elif selected_port not in found_ports:
                available = ", ".join(found_ports)
                print(
                    f"Port {selected_port!r} was not found. Available ports: {available}",
                    file=sys.stderr,
                )
                return 1

            config_path = Path(args.config)
            config = SOARMConfig.from_file(config_path).replace_arm_port(selected_port)
            config.save(config_path)
            print(f"Updated {config_path}: arm.port={selected_port}")
            return 0

        if args.command == "web":
            from ..web import run_web

            return run_web(config_path=args.config, host=args.host, port=args.port, mock=args.mock)

        if args.command == "urdf":
            if args.print:
                print(SOARM_URDF_PATH.read_text())
            else:
                print(SOARM_URDF_PATH)
            return 0

        if args.command == "fk":
            config = SOARMConfig.from_file(Path(args.config))
            positions = home_positions(config)
            positions.update(_parse_targets(args.targets))
            print(json.dumps(forward_kinematics(config, positions), indent=2))
            return 0

        if args.command == "ik":
            config = SOARMConfig.from_file(Path(args.config))
            result = solve_position_ik(
                config,
                {"x": args.x, "y": args.y, "z": args.z},
                elbow=args.elbow,
            )
            print(json.dumps(result, indent=2))
            return 0

        arm = SOARM.from_config(Path(args.config), mock=args.mock)
        if args.command in {"replay-demo", "replay"} and args.dry_run:
            demo = load_demonstration(args.input)
            replayer = DemonstrationReplayer(arm)
            count = replayer.replay(
                demo,
                speed=args.speed,
                start_at=args.start_at,
                end_at=args.end_at,
                move_to_start=not args.no_lead_in,
                lead_in_duration=args.lead_in_duration,
                feedback_tolerance=args.feedback_tolerance,
                output_hz=args.output_hz,
                interpolation=args.interpolation,
                dry_run=True,
                announce=print,
            )
            print(f"Validated {count} samples from {args.input}")
            return 0

        with arm:
            if args.command == "status":
                for line in arm.diagnostics():
                    print(line)
            elif args.command == "configure-motors":
                results = arm.apply_motor_profile(
                    force=args.force,
                    check_only=args.check_only,
                )
                if not results:
                    print("motor profile: no changes needed")
                for result in results:
                    print(
                        "motor profile: "
                        f"{result.action} {result.joint} "
                        f"(id={result.servo_id}) {result.register}: "
                        f"{result.current} -> {result.target}"
                    )
            elif args.command == "read":
                for name, value in arm.get_joint_positions(unit=args.unit).items():
                    print(f"{name}: {value}")
            elif args.command == "home":
                arm.enable()
                arm.move_home(duration=args.duration)
            elif args.command == "move":
                arm.enable()
                arm.move_joints(_parse_targets(args.targets), duration=args.duration, wait=not args.no_wait)
            elif args.command == "pose":
                arm.enable()
                arm.move_pose(args.name, duration=args.duration, wait=not args.no_wait)
            elif args.command == "probe-joint":
                if args.name not in arm.config.joints:
                    raise ValueError(f"Unknown joint: {args.name}")
                if args.target is None and args.delta is None:
                    raise ValueError("probe-joint requires --target or --delta")
                if args.target is not None and args.delta is not None:
                    raise ValueError("probe-joint accepts only one of --target or --delta")
                before = _joint_register_snapshot(arm, args.name)
                current_rad = arm.get_joint_positions(unit="rad")[args.name]
                target_rad = float(args.target) if args.target is not None else current_rad + float(args.delta)
                arm.config.joints[args.name].check_limit(target_rad)
                print(json.dumps({"phase": "before_enable", **before}, indent=2))
                arm.enable()
                print(
                    json.dumps(
                        {
                            "phase": "after_enable",
                            **_joint_register_snapshot(arm, args.name, target_rad=target_rad),
                        },
                        indent=2,
                    )
                )
                arm.move_joint(args.name, target_rad, duration=args.duration, wait=True)
                if args.settle > 0:
                    time.sleep(args.settle)
                print(
                    json.dumps(
                        {
                            "phase": "after_move",
                            **_joint_register_snapshot(arm, args.name, target_rad=target_rad),
                        },
                        indent=2,
                    )
                )
            elif args.command in {"record-demo", "record"}:
                recorder = DemonstrationRecorder(
                    arm,
                    sample_hz=args.hz,
                    joints=args.joints,
                )
                if args.duration is None:
                    print("Recording demonstration. Press Ctrl+C to stop.")
                else:
                    print(f"Recording demonstration for {args.duration:.3f}s.")
                demo = recorder.record(
                    duration=args.duration,
                    output_path=args.output,
                    disable_torque=not args.keep_torque,
                    announce=print,
                )
                print(
                    f"Recorded {len(demo.samples)} samples over "
                    f"{demo.duration:.3f}s to {args.output}"
                )
            elif args.command in {"replay-demo", "replay"}:
                demo = load_demonstration(args.input)
                replayer = DemonstrationReplayer(arm)
                count = replayer.replay(
                    demo,
                    speed=args.speed,
                    start_at=args.start_at,
                    end_at=args.end_at,
                    move_to_start=not args.no_lead_in,
                    lead_in_duration=args.lead_in_duration,
                    feedback_tolerance=args.feedback_tolerance,
                    output_hz=args.output_hz,
                    interpolation=args.interpolation,
                    dry_run=args.dry_run,
                    announce=print,
                )
                action = "Validated" if args.dry_run else "Replayed"
                print(f"{action} {count} samples from {args.input}")
            elif args.command == "calibrate":
                preflight = arm.diagnostics()
                _print_report("Pre-calibration status:", preflight)
                if not calibration_ready_from_report(preflight):
                    print(
                        "soarm: hardware is not ready for calibration; fix the status failures first.",
                        file=sys.stderr,
                    )
                    return 1
                arm.calibrate(output_path=args.output, announce=print)
                postflight = arm.diagnostics()
                _print_report("Post-calibration status:", postflight)
                if not _diagnostics_passed(postflight):
                    print(
                        "soarm: calibration was saved, but the post-calibration status check failed.",
                        file=sys.stderr,
                    )
                    return 1
            elif args.command == "capture-pose":
                arm.capture_pose(args.name, output_path=args.output)
            elif args.command == "disable":
                arm.disable()
    except (SOARMError, ValueError) as exc:
        print(f"soarm: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
