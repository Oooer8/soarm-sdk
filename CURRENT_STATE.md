# SOARM Current State

Last updated: 2026-06-18

This file records the current control, motor-profile, and visualization state so
future changes have a concrete baseline.

## Hardware Layer

- The active bus implementation is `soarm.hardware.ServoBus`, backed directly by
  `scservo_sdk`.
- The supported target is the SOARM / SO-ARM101 style arm with Feetech STS3215
  servos at 1 Mbps.
- `ServoBus` owns register I/O only: sync read, sync write, PID helpers,
  torque, operating mode, and profile-cache mechanics.
- High-level motion planning should stay out of `ServoBus`.

## Motor Profile Policy

- Motor profile configuration lives in `configs/motors/feetech_sts3215.yaml`.
- The profile is aligned with LeRobot's SO101 Feetech setup:
  - `Return_Delay_Time = 0`
  - `Maximum_Acceleration = 254`
  - startup/runtime `Acceleration = 254`
  - PID `P=16, I=0, D=32`
  - gripper protection overrides
- `SOARM.enable()` applies the motor profile once per connection when
  `apply_on_enable: true`.
- Runtime position commands use the same packet shape as Feetech STS
  `SyncWritePosEx`: a 7-byte SRAM block starting at `Acceleration`, containing
  `Acceleration`, `Goal_Position`, `Goal_Time=0`, and `Goal_Velocity`.
  Hardware probing showed that writing only `Goal_Position` updates the
  register but does not reliably trigger motion on STS3215 servos.
- `joints.*.max_vel_rad_s` and `joints.*.max_acc_rad_s2` are joint-space
  safety/planning limits. They are not converted into Feetech
  `Goal_Velocity` or `Acceleration` on each move.

## Motion Control

- Blocking CLI/Python moves with `wait=True` and `duration>0` use
  `linear_trajectory()`.
- `linear_trajectory()` currently provides position-only linear interpolation at
  `config.arm.control_hz` (default 200 Hz).
- Safety checks now include:
  - known joints
  - joint soft limits
  - max step
  - average velocity feasibility
  - rest-to-rest velocity/acceleration duration feasibility
  - voltage threshold
- The trajectory shape itself is still linear; it is not yet trapezoidal,
  S-curve, or jerk-limited.

## Runtime Frequencies

- High-level target input: 60 Hz default
- Controller interpolation: 200 Hz default
- Serial position command block write target: 200 Hz in blocking interpolated moves
- Feedback sampler: 100 Hz default
- Web state sync: 30 Hz default
- Web motion command throttle: 30 Hz default
- Web rendering: 60 Hz default

## Web Control Semantics

- URDF/2D visualization and numeric joint readouts always follow hardware
  feedback.
- With joint enable off, the user can back-drive the real arm; sliders follow
  the feedback stream and do not act as controls.
- With joint enable on, sliders represent command targets and are sent through
  `/api/move` with `wait=false`.
- Web slider control does not currently run the 200 Hz interpolation loop. It
  is a throttled stream of Feetech STS position command blocks plus motor-side
  position control.
- Enabling joints aligns the frontend target sliders to the current feedback
  sample so stale slider targets are not reused.

## Known Next Step

The next control improvement should be a motion-layer servo loop or trajectory
generator that supports velocity/acceleration-limited retargeting:

- For point-to-point scripted moves: trapezoidal or S-curve interpolation.
- For dynamic grasping: a persistent 200 Hz command loop that accepts 30-60 Hz
  target updates and preserves at least position and velocity continuity.
- `ServoBus` should remain a low-level register bus.
