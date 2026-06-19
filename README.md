<p align="center">
  <img src="docs/assets/brand/soarm_horizontal_lockup.svg" alt="SOARM SDK logo" width="560">
</p>

# SOARM SDK

🦾 Python SDK and local web console for controlling a SOARM / SO-ARM101 style robotic arm with Feetech STS3215 servos.

SOARM SDK keeps hardware access, calibration, safety checks, kinematics, motion control, teaching replay, CLI tools, and the browser control console behind one shared Python package.

## Features

- Feetech STS3215 servo bus wrapper built on `scservo_sdk`
- YAML-based arm, runtime, calibration, and motor-profile configuration
- Read-only health checks before calibration or motion
- Interactive calibration and soft-limit generation
- Joint-space motion with safety checks
- FK/IK helpers for the SOARM geometry
- Joint-space teaching record and replay
- Local Web GUI for setup, calibration, URDF visualization, feedback, and slider control
- Mock bus for development without hardware

## Install

```bash
pip install -e .
```

This workspace uses the `soarm-sdk` conda environment for local validation:

```bash
conda run -n soarm-sdk soarm-sdk status --config configs/soarm-sdk.yaml --mock
```

## Quick Start

```bash
soarm-sdk ports --config configs/soarm-sdk.yaml
soarm-sdk status --config configs/soarm-sdk.yaml
soarm-sdk calibrate --config configs/soarm-sdk.yaml --output configs/soarm-sdk.yaml
soarm-sdk home --config configs/soarm-sdk.yaml --duration 2.0
soarm-sdk web --config configs/soarm-sdk.yaml
```

For no-hardware checks, add `--mock` where supported:

```bash
soarm-sdk status --config configs/soarm-sdk.yaml --mock
soarm-sdk web --config configs/soarm-sdk.yaml --mock
```

## Python API

```python
from soarm_sdk import SOARM

with SOARM.from_config("configs/soarm-sdk.yaml") as arm:
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

## Documentation

The detailed documentation is a single static page:

- [Documentation site](https://oooer8.github.io/soarm-sdk/)
- [Source: docs/index.html](docs/index.html)

GitHub Actions uploads `docs/` as a Pages artifact and deploys it to GitHub Pages. No generated site files are committed.

## Repository Layout

```text
configs/       Example SOARM, runtime, and motor-profile configs
docs/          Single-page documentation site
examples/      Small Python API examples
scripts/       Bench and measurement utilities
src/soarm_sdk/     Python package
  application/ Web/CLI payload and workflow helpers
  calibration/ Calibration capture and sweep logic
  cli/         Command-line entry point and command dispatch
  config/      YAML schema, split-config loading, and persistence
  demonstration/
               Demonstration data, validation, recording, and replay
  hardware/    Feetech bus, registers, units, and motor profile writes
  kinematics/  SOARM FK/IK helpers
  model/       Shared dataclasses for joints, poses, and state
  motion/      Trajectory generation and motion controller
  safety/      Joint, step, velocity, acceleration, and voltage guards
  testing/     Mock bus for no-hardware validation
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for the package layering and import rules.

## License

MIT
