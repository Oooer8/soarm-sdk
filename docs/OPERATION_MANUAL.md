# SOARM Operation Manual

This manual is the day-to-day operating guide for the SOARM SDK. It keeps the README readable while preserving the details needed for bring-up, calibration, scripted control, Web control, teaching replay, and debugging.

## Hardware Scope

SOARM SDK currently targets a SOARM / SO-ARM101 style arm using Feetech STS3215 servos through `scservo_sdk`, wrapped by `soarm.hardware.ServoBus`.

The SDK expects servo IDs to already match `configs/soarm.yaml`. It does not infer joint order from the bus. If several new servos still share the factory default ID, connect and rename one servo at a time with the vendor or low-level Feetech tooling before using this SDK.

Default joint IDs:

| Joint | Servo ID |
| --- | ---: |
| `shoulder_pan` | 1 |
| `shoulder_lift` | 2 |
| `elbow_flex` | 3 |
| `wrist_flex` | 4 |
| `wrist_roll` | 5 |
| `gripper` | 6 |

## Repository Map

| Path | Purpose |
| --- | --- |
| `configs/soarm.yaml` | Main arm instance config: serial port, joint IDs, calibration, limits, named poses. |
| `configs/runtime.yaml` | Runtime frequencies, voltage thresholds, motion step limits, disconnect behavior. |
| `configs/motors/feetech_sts3215.yaml` | Feetech motor profile, PID, acceleration, mode, and gripper protection policy. |
| `src/soarm/arm.py` | User-facing `SOARM` facade. |
| `src/soarm/hardware/` | Servo register I/O, unit conversion, sync reads and sync writes. |
| `src/soarm/motion/` | Motion controller, trajectory interpolation, safety-checked target writes. |
| `src/soarm/calibration/` | Zero capture, range sweep, soft-limit calculation. |
| `src/soarm/diagnostics/` | Read-only status and health checks. |
| `src/soarm/kinematics/` | FK/IK and SOARM geometry. |
| `src/soarm/demonstration.py` | Joint-space teaching record/replay format and helpers. |
| `src/soarm/workflows.py` | Shared CLI/Web workflow payloads and rules. |
| `src/soarm/web.py` | Local HTTP API for the static Web GUI. |
| `src/soarm/webapp/` | Browser setup, calibration, visualization, and control UI. |
| `examples/` | Small Python API examples. |
| `scripts/` | Bench and frequency measurement scripts. |

## Setup

Install the package in editable mode:

```bash
pip install -e .
```

The local development environment for this workspace is the `soarm-sdk` conda environment:

```bash
conda run -n soarm-sdk python -m pip install -e .
conda run -n soarm-sdk python -m soarm.cli status --config configs/soarm.yaml --mock
```

The mock bus is useful for CLI, Web, and documentation checks. It does not replace real checks for wiring, servo IDs, voltage, mechanical range, or torque behavior.

## Configuration Model

`configs/soarm.yaml` is the single entry point for CLI, Web, and Python API usage. It includes runtime and motor profile files:

```yaml
includes:
  runtime: runtime.yaml
  motor_profile: motors/feetech_sts3215.yaml
```

Keep each physical idea in one place:

- Joint identity, calibration, soft limits, named poses, serial port, and baud rate live in `configs/soarm.yaml`.
- Process and control-loop frequencies live in `configs/runtime.yaml`.
- Feetech register policy lives in `configs/motors/feetech_sts3215.yaml`.
- `joints.*.max_vel_rad_s` and `joints.*.max_acc_rad_s2` are joint-space safety/planning constraints, not direct per-move `Goal_Velocity` or `Acceleration` register values.

Motor profile writes are not part of the 200 Hz control loop. Normal motion keeps the real-time path focused on the Feetech STS position command block: `Acceleration`, `Goal_Position`, `Goal_Time=0`, and `Goal_Velocity`.

## First Bring-Up Checklist

1. Inspect wiring, power, linkage travel, and gripper clearance with power off.
2. Assign servo IDs to match `configs/soarm.yaml`.
3. Install the SDK and run the mock status command.
4. Scan and save the serial port.
5. Run real `status` and inspect communication, voltage, raw position, and soft-limit results.
6. If calibration is missing or stale, run calibration.
7. Save calibration and immediately run post-calibration status.
8. Check the Feetech motor profile.
9. Try a small low-speed move or named pose.
10. Open the Web GUI for visual feedback and slider control.

Recommended starting commands:

```bash
conda run -n soarm-sdk python -m soarm.cli ports --config configs/soarm.yaml
conda run -n soarm-sdk python -m soarm.cli status --config configs/soarm.yaml
conda run -n soarm-sdk python -m soarm.cli calibrate --config configs/soarm.yaml --output configs/soarm.yaml
conda run -n soarm-sdk python -m soarm.cli configure-motors --config configs/soarm.yaml --check-only
conda run -n soarm-sdk python -m soarm.cli home --config configs/soarm.yaml --duration 2.0
```

## CLI Workflows

Port discovery:

```bash
soarm ports --config configs/soarm.yaml
soarm ports --config configs/soarm.yaml --port /dev/cu.usbmodem5A7C1190351
soarm ports --config configs/soarm.yaml --no-update
```

Health and calibration:

```bash
soarm status --config configs/soarm.yaml
soarm calibrate --config configs/soarm.yaml --output configs/soarm.yaml
```

Joint reads and motion:

```bash
soarm read --config configs/soarm.yaml --unit rad
soarm home --config configs/soarm.yaml --duration 1.5
soarm move --config configs/soarm.yaml shoulder_pan=0.0 elbow_flex=0.5 --duration 1.0
soarm pose --config configs/soarm.yaml ready --duration 1.0
```

Maintenance and debugging:

```bash
soarm configure-motors --config configs/soarm.yaml --check-only
soarm configure-motors --config configs/soarm.yaml
soarm configure-motors --config configs/soarm.yaml --force
soarm probe-joint shoulder_pan --config configs/soarm.yaml --delta -0.1 --duration 1
soarm capture-pose --config configs/soarm.yaml ready --output configs/soarm.yaml
soarm disable --config configs/soarm.yaml
```

Kinematics:

```bash
soarm fk --config configs/soarm.yaml shoulder_lift=-0.3 elbow_flex=0.5
soarm ik --config configs/soarm.yaml 0.42 0.0 0.20
```

## Web GUI Workflow

Start the local Web GUI:

```bash
conda run -n soarm-sdk python -m soarm.cli web --config configs/soarm.yaml --mock
```

Without `--mock`, the Web server talks to the configured serial bus. The default URL is:

```text
http://127.0.0.1:8765/
```

The Web GUI has three practical phases:

| Phase | Purpose |
| --- | --- |
| Detect | Select port, save arm name/port, edit runtime frequencies, run status. |
| Calibrate | Disable torque, capture zero, sweep all joints, review new limits, save calibration. |
| Visualize/control | Load URDF, stream real joint feedback, enable torque, send slider targets through SDK safety checks. |

The Web GUI only enters control when `control_ready=true`: a calibration exists, the current status passes, and current joint positions are inside saved soft limits.

## Teaching Record And Replay

Record a joint-space demonstration:

```bash
soarm record-demo demos/pick_place.json --config configs/soarm.yaml --duration 10 --hz 20
soarm record-demo demos/elbow_only.json --config configs/soarm.yaml --joints shoulder_pan elbow_flex
```

Replay safely:

```bash
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --dry-run
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --speed 0.5
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --output-hz 200 --interpolation pchip
```

`record-demo` writes `soarm.demonstration.v1` JSON with `created_at`, `metadata`, `joints`, and time-stamped radian samples. `replay-demo` validates the file, moves through a safety-checked lead-in, interpolates sparse samples to output setpoints, and reuses SDK limit checks. The default interpolation is PCHIP; use `linear` for conservative debugging.

## Python API

```python
from soarm import SOARM

with SOARM.from_config("configs/soarm.yaml") as arm:
    arm.enable()
    arm.move_home(duration=1.5)
    arm.move_joints(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": -0.4,
            "elbow_flex": 0.8,
        },
        duration=2.0,
    )
    print(arm.get_joint_positions())
```

Demonstration helpers can also be used directly:

```python
from soarm import SOARM, record_demonstration, replay_demonstration

with SOARM.from_config("configs/soarm.yaml") as arm:
    record_demonstration(arm, "demos/pick_place.json", duration=10, sample_hz=20)

with SOARM.from_config("configs/soarm.yaml") as arm:
    replay_demonstration(arm, "demos/pick_place.json", speed=1.0)
```

## Safety Gates

Use `status` as the read-only gate before real motion. It checks:

- configured servo IDs are online
- current positions can be read
- current positions are inside soft limits
- voltage is above the configured threshold

Calibration readiness is slightly different from control readiness. Old soft limits may fail while communication, voltage, and raw position reads are still healthy enough to recalibrate. Control requires a saved calibration and a passing current status.

## Troubleshooting

Status fails on communication:

- Confirm serial port, baud rate, cabling, power, and servo IDs.
- Run `soarm ports --config configs/soarm.yaml --no-update` and verify the selected device.

Status fails only on soft limits:

- Move the arm back into the saved range and rerun status.
- If the saved limits are stale, recalibrate.

`Goal_Position` changes but the servo does not move:

- Run `probe-joint` and inspect `Torque_Enable`, `Operating_Mode`, `Moving`, `Present_Load`, `Present_Current`, `Status`, `Acceleration`, and `Goal_Velocity`.
- Confirm the motor profile was applied. STS3215 position moves should use the full 7-byte position command block, not only a standalone `Goal_Position` write.

Replay lead-in reports a large remaining delta:

- The joint may not be following position commands.
- The present-position readback may be stale or wrong.
- Re-run `probe-joint` on the failing joint and slow the lead-in with `--lead-in-duration`.

Voltage fails:

- Do not move until power is corrected.
- Re-run status after stabilizing the supply.

Joint direction or zero looks wrong:

- Stop motion.
- Re-run calibration and verify zero ticks, sweep ranges, direction, and radian limits before enabling torque.

## Development And Validation

Use mock checks before touching hardware:

```bash
conda run -n soarm-sdk python -m soarm.cli status --config configs/soarm.yaml --mock
conda run -n soarm-sdk python -m soarm.cli fk --config configs/soarm.yaml shoulder_lift=-0.3 elbow_flex=0.5
conda run -n soarm-sdk python -m soarm.cli ik --config configs/soarm.yaml 0.42 0.0 0.20
conda run -n soarm-sdk python -m soarm.cli record-demo /tmp/soarm-demo.json --config configs/soarm.yaml --mock --duration 1
conda run -n soarm-sdk python -m soarm.cli replay-demo /tmp/soarm-demo.json --config configs/soarm.yaml --dry-run
```

When modifying code, keep these ownership boundaries:

- `ServoBus` owns register I/O only.
- `SOARM` exposes the public SDK facade.
- Motion planning and safety live outside the hardware bus layer.
- CLI and Web should call shared workflow/API code instead of duplicating calibration math, safety gates, FK/IK, or config writes.

## GitHub Publishing Checklist

Before the first GitHub push:

1. Confirm whether the repository should be public or private.
2. Confirm the target remote, for example `git@github.com:Oooer8/soarm.git`.
3. Re-authenticate GitHub CLI if needed with `gh auth login -h github.com`.
4. Ensure generated files such as `__pycache__`, `.pyc`, `.DS_Store`, build artifacts, and virtualenvs are ignored.
5. Run the mock validation commands.
6. Commit documentation, source, configs, examples, and required assets.
7. Push the initial branch and optionally open a draft PR if the project uses PR review.
