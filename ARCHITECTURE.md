# SOARM SDK Architecture

SOARM SDK is organized as a layered Python package. Lower layers should stay
free of imports from higher layers so the SDK can be reused from CLI, Web,
scripts, tests, and future application surfaces.

## Current Capabilities

- Feetech STS/SMS servo bus access through `scservo_sdk`
- Packaged canonical YAML config plus generated split user config copies
- Read-only diagnostics for communication, position, soft limits, and voltage
- Interactive calibration with zero capture and one-pass all-joint sweep
- Joint-space safety checks for limits, step size, velocity, acceleration, and voltage
- Position motion, one-shot streaming setpoints, online joint streaming, and timed trajectory replay
- Forward/inverse kinematics helpers for the bundled SO-ARM101-style geometry
- Demonstration JSON recording, validation, windowing, and replay
- Local Web console for setup, status, calibration, URDF visualization, feedback, and slider control
- Mock bus for no-hardware CLI and development validation

## Layers

1. `soarm_sdk.constants`, `soarm_sdk.errors`, `soarm_sdk.model`
   define SDK-wide constants, exceptions, and pure dataclasses.
2. `soarm_sdk.config`
   owns packaged defaults, user-copy generation, and YAML load/merge/save logic,
   including split files.
3. `soarm_sdk.hardware`
   owns Feetech register definitions, unit conversion, serial bus I/O, torque,
   operating mode, PID, and motor-profile register writes.
4. `soarm_sdk.safety`
   owns reusable safety rules: known joints, soft limits, max step, velocity,
   acceleration, voltage, emergency stop, and minimum feasible motion duration.
5. `soarm_sdk.motion`, `soarm_sdk.kinematics`, `soarm_sdk.calibration`,
   and `soarm_sdk.diagnostics`
   implement robot-domain behavior on top of config, safety, and hardware
   interfaces.
6. `soarm_sdk.arm.SOARM`
   is the public Python facade. It wires config, bus, safety, motion,
   calibration, diagnostics, kinematics, motor profile, and demonstration flows.
7. `soarm_sdk.demonstration`
   owns demonstration JSON data, validation, recording, and replay services.
8. `soarm_sdk.application`
   contains shared payload and workflow helpers used by the Web server and any
   future UI surface.
9. `soarm_sdk.cli`, `soarm_sdk.web`, `soarm_sdk.webapp`, examples, and scripts
   are entry points and adapters.

## Import Rules

- `hardware` must not import `motion`, `calibration`, `diagnostics`, `arm`,
  `application`, `cli`, or `web`.
- `safety` may depend on config/model/errors, but should not know about serial
  transport or HTTP.
- `motion`, `calibration`, and `diagnostics` may operate on a bus-like object,
  which allows both `ServoBus` and `MockBus`.
- `SOARM` is allowed to compose lower-level services, but lower-level services
  should not import `SOARM` except under `TYPE_CHECKING`.
- CLI and Web should parse inputs, enforce session/workflow gates, and call SDK
  or `application` helpers instead of duplicating calibration math, FK/IK, config
  writes, or safety rules.

## Runtime Flow

Status:

```text
CLI/Web -> SOARM -> diagnostics -> bus.read_positions/read_voltages
                         -> config joint conversions and threshold checks
```

Calibration:

```text
CLI/Web -> SOARM/calibration -> disable torque
                         -> capture zero ticks
                         -> AllJointsRangeRecorder reads all joints concurrently
                         -> build calibrated directions and soft limits
                         -> SOARMConfig.save()
```

Motion:

```text
CLI/Web/Python -> SOARM -> MotionController
                         -> read current positions and voltage
                         -> SafetyGuard validates limits, step, velocity, acceleration
                         -> ServoBus.write_positions() writes Feetech position command blocks
```

Motion APIs:

```text
move_joints(wait=True)
  -> point-to-point move with rest-to-rest feasibility checks

stream_joints(targets, dt=...)
  -> one-shot setpoint write for simple caller-driven streams

start_joint_stream()
  -> long-lived online controller
  -> latest-target overwrite slot
  -> fixed-rate velocity/acceleration-limited output
  -> timeout hold when target updates stop

follow_joint_trajectory()
  -> fixed-rate replay of an already-timestamped trajectory
```

Teaching replay:

```text
Demonstration JSON -> DemonstrationReplayer
                   -> validate sample joints and limits
                   -> segment lead-in if needed
                   -> TimedJointTrajectory fixed-rate interpolation
                   -> MotionController writes setpoints
```

Web:

```text
HTTP handler -> application.workflow payload helpers
             -> long-lived control arm guarded by a lock
             -> feedback sampler caches state at configured feedback_hz
             -> browser receives cached state at web_state_hz
```

## Configuration Model

`src/soarm_sdk/configs/` is the single tracked canonical config template and is
included in the Python package. It contains the main config:

- `soarm-sdk.yaml` for identity, bus settings, calibration, joints, and poses.
- `runtime.yaml` for control frequency, feedback frequency, Web update
  rates, voltage threshold, max step, and auto-disable behavior.
- `motors/feetech_sts3215.yaml` for motor-profile policy and register
  values.

The repository-root `configs/` directory is not a second source of truth. It is
gitignored and exists as the default location for generated user working copies.
`soarm-sdk init-config`, write-capable CLI commands, and the Web UI can copy the
packaged split config there before saving port, calibration, identity, runtime,
or motor-profile edits.

When a split config is loaded and saved back to the same main path,
`SOARMConfig.save()` writes runtime values back to the runtime file and motor
profile values back to the motor-profile file. Saving to a different path writes
one fully merged YAML file.

## Efficiency Notes

- `ServoBus.sync_read()` caches group sync readers by register and servo ID tuple.
- `ServoBus.prepare_position_control()` avoids repeatedly writing torque/mode for
  servos already prepared for position control.
- `ServoBus.write_positions()` writes the Feetech STS/SMS seven-byte position
  command block starting at `Acceleration`, because writing only `Goal_Position`
  does not reliably trigger STS3215 movement.
- `SOARM` passes one reentrant I/O lock into `MotionController`, so online
  streaming writes and foreground state reads serialize on the same bus.
- `JointStreamingController` only writes when its output reference advances.
  Once a target is reached or target updates time out, it holds locally instead
  of sending redundant bus writes.
- Web feedback reads are sampled in a background thread and cached; voltage is
  refreshed at a slower cadence than position.
- Trajectory replay converts sparse samples into fixed-rate setpoints once before
  sending them to the controller loop.

## Compatibility Paths

These thin modules are kept for import compatibility and should remain small:

- `soarm_sdk.motor_profile` re-exports `soarm_sdk.hardware.motor_profile`
- `soarm_sdk.workflows` re-exports `soarm_sdk.application.workflows`

New internal code should import the layered module directly.

## CLI Entrypoints

The installed command is:

```bash
soarm-sdk status --mock
```

The module form is:

```bash
python -m soarm_sdk.cli status --mock
```

## Development Checks

Use the project conda environment for local validation:

```bash
PYTHONPYCACHEPREFIX=/private/tmp/soarm-sdk-pycache conda run -n soarm-sdk python -m compileall src tests
conda run -n soarm-sdk python -m unittest discover -s tests -v
conda run -n soarm-sdk soarm-sdk status --mock
conda run -n soarm-sdk soarm-sdk fk shoulder_lift=-0.3 elbow_flex=0.5
conda run -n soarm-sdk soarm-sdk ik 0.42 0.0 0.20
```

Focused unit tests cover the online streaming controller. CLI mock paths and
compile checks remain the broader lightweight regression checks.
