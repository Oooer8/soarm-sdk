<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="src/soarm/assets/brand/soarm-logo-dark.svg">
    <img src="src/soarm/assets/brand/soarm-logo.svg" alt="SOARM SDK logo" width="460">
  </picture>
</p>

# SOARM SDK

Python SDK and local web console for controlling a SOARM / SO-ARM101 style robotic arm with Feetech STS3215 servos.

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
conda run -n soarm-sdk python -m soarm.cli status --config configs/soarm.yaml --mock
```

## Quick Start

```bash
soarm ports --config configs/soarm.yaml
soarm status --config configs/soarm.yaml
soarm calibrate --config configs/soarm.yaml --output configs/soarm.yaml
soarm home --config configs/soarm.yaml --duration 2.0
soarm web --config configs/soarm.yaml
```

For no-hardware checks, add `--mock` where supported:

```bash
soarm status --config configs/soarm.yaml --mock
soarm web --config configs/soarm.yaml --mock
```

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

## Documentation

The detailed documentation is a single static page:

- [Documentation site](https://oooer8.github.io/soarm-sdk/)
- [Source: docs/index.html](docs/index.html)

The documentation site is published from `docs/index.html` to the `gh-pages` branch for GitHub Pages.

## Repository Layout

```text
configs/       Example SOARM, runtime, and motor-profile configs
docs/          Single-page documentation site
examples/      Small Python API examples
scripts/       Bench and measurement utilities
src/soarm/     SDK, CLI, web server, web app, assets, and tests helpers
```

## License

MIT
