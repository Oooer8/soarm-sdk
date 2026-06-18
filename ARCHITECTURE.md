# SOARM Package Architecture

SOARM is organized around a small set of layers. Lower layers should not import
from higher layers.

## Layers

1. `soarm.model`, `soarm.config`, `soarm.constants`, `soarm.errors`
   define pure data, configuration loading, and SDK-wide exceptions.
2. `soarm.hardware` owns Feetech serial bus access, register definitions, unit
   conversions, and motor profile writes.
3. `soarm.safety`, `soarm.motion`, `soarm.kinematics`, and `soarm.calibration`
   implement robot-domain behavior on top of config and hardware interfaces.
4. `soarm.arm.SOARM` is the public Python facade that wires config, hardware,
   safety, motion, calibration, diagnostics, and teaching workflows together.
5. `soarm.demonstration` owns demonstration JSON data, validation, recording,
   and replay services.
6. `soarm.application` contains payload and workflow helpers shared by the web
   server and future app surfaces.
7. `soarm.cli`, `soarm.web`, examples, and scripts are entry points.

## Compatibility Paths

The old public modules remain import-compatible:

- `soarm.config`
- `soarm.demonstration`
- `soarm.motor_profile`
- `soarm.workflows`

New code should prefer the layered locations when it is reaching into internals,
for example `soarm.hardware.motor_profile` and `soarm.application.workflows`.

## CLI Entrypoints

Both entry points are supported:

```bash
soarm status --config configs/soarm.yaml --mock
python -m soarm.cli status --config configs/soarm.yaml --mock
```
