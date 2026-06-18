# SOARM SDK

SOARM SDK 是一个面向 SOARM 机械臂的 Python 控制包，负责串口总线、配置加载、健康检查、标定、软限位、关节空间运动、FK/IK、关节空间示教记录/回放，以及两个前端入口：

- CLI：适合脚本、维护、批处理和无界面环境。
- Web GUI：适合首次带机、标定引导、URDF 可视化、实时反馈和交互式点动。

两条路共用同一套 SDK 逻辑。安全检查、标定计算、FK/IK、运动限位和配置写入不应该在前端重复实现。

## Documentation

README 保持为项目入口和常用命令速查。更完整的带机、标定、CLI/Web 使用、示教回放、安全门禁和排障流程见 [docs/OPERATION_MANUAL.md](docs/OPERATION_MANUAL.md)。

当前控制层、电机 profile、Web 语义和后续改进方向见 [CURRENT_STATE.md](CURRENT_STATE.md)。

## Hardware

本 SDK 目前只面向 SOARM 机械臂，使用 Feetech ST-3215 舵机，并通过 `soarm.hardware.ServoBus` 封装 Feetech `scservo_sdk`。

其他舵机型号不是当前设计目标。

## Install

```bash
pip install -e .
```

本仓库本地验证推荐使用 `soarm-sdk` conda 环境：

```bash
conda run -n soarm-sdk python -m soarm.cli status --config configs/soarm.yaml --mock
```

## Configuration Layout

`configs/soarm.yaml` 是 CLI、Web GUI 和 SDK API 的统一入口。它通过 include 拆出运行时配置和电机寄存器策略：

```yaml
includes:
  runtime: runtime.yaml
  motor_profile: motors/feetech_sts3215.yaml
```

- `configs/soarm.yaml`：机械臂实例配置，包括名称、串口、波特率、关节 ID、标定、软限位和命名姿态。
- `configs/runtime.yaml`：进程运行参数，包括控制频率、反馈频率、Web 同步/渲染频率、电压阈值、最大单步和断开时是否关闭扭矩。
- `configs/motors/feetech_sts3215.yaml`：Feetech 电机寄存器策略，包括 PID、返回延迟、启动加速度、位置模式和夹爪保护限制。

避免在多个文件里重复表达同一个物理含义。比如 `joints.*.max_vel_rad_s` 和 `joints.*.max_acc_rad_s2` 仍然是 `configs/soarm.yaml` 里的关节空间安全和规划限制，不再直接下沉为 Feetech `Goal_Velocity` 和 `Acceleration`。电机侧 profile 对齐 LeRobot 的 SO101 做法：`Maximum_Acceleration=254`、`Acceleration=254`。运行时位置命令按 Feetech STS SDK 的 `SyncWritePosEx` 形状写入一个 7 字节控制块：`Acceleration + Goal_Position + Goal_Time=0 + Goal_Velocity`。

## Capability Map

| 能力 | CLI | Web GUI | 说明 |
| --- | --- | --- | --- |
| 启动 Web 控制台 | Yes | N/A | `soarm web` 启动本地页面 |
| 扫描候选串口 | Yes | Yes | CLI 可自动或指定写入 `arm.port`；GUI 可下拉选择或手填 |
| 保存机械臂名称 | No | Yes | GUI 可编辑 `arm.name`；CLI 目前没有专门命令 |
| 保存实时频率 | No | Yes | GUI 可写入 runtime 频率；CLI 可手动编辑 YAML |
| 健康检查/status | Yes | Yes | 两者都调用同一套 diagnostics |
| 标定 readiness 门禁 | Yes | Yes | 通信、电压、原始位置读数必须健康；旧软限位失败仍可进入重标定 |
| 交互式标定 | Yes | Yes | CLI 用终端提示；GUI 用分步按钮、表格和报告 |
| 标定后复检 | Yes | Yes | 保存新限位后立刻重新跑 status；通过后才允许控制 |
| 已有标定重开检查 | Manual | Automatic | CLI 用户手动跑 `status`；GUI 打开后自动检查当前状态是否仍在限位内 |
| 读取当前关节 | Yes | Yes | CLI 打印；GUI 刷新面板和模型 |
| URDF/3D 可视化 | Print path/XML only | Yes | GUI 加载并驱动 SO-ARM101 URDF |
| 实机反馈可视化 | No | Yes | GUI 持续读取实机；模型和实际值只显示反馈 |
| 滑条目标控制 | No | Yes | 关节失能时滑条跟随实机；使能后滑条目标下发到 SDK |
| 命令式运动 | Yes | Partial | CLI 支持 `home`、`move`、`pose`；GUI 支持滑条点动 |
| 示教记录/回放 | Yes | No | `record-demo` 记录关节空间 JSON，`replay-demo` 复用 SDK 安全层回放 |
| FK/IK | Yes | Yes | CLI 输出 JSON；GUI 可把 IK 结果写入目标滑条 |
| 捕获命名姿态 | Yes | No | `capture-pose` 写入 YAML |
| 电机 profile 检查/应用 | Yes | No | `configure-motors` 是 CLI 维护入口 |
| 禁用所有舵机 | Yes | Calibration-only | CLI 有 `disable`；GUI 会在标定开始时禁用扭矩 |
| Mock bus | Yes | Yes | 用于无硬件开发和 UI 验证 |

## Bring-Up Overview

SOARM 有两条实际使用路线：

1. CLI 路线：适合 SSH、脚本化验证、批处理动作、CI/mock 检查和电机维护。
2. Web GUI 路线：适合第一次接线调试、人工标定、观察模型、实时读状态、用滑条点动。

两条路线的安全门禁一致：

1. 先确认舵机 ID 已经正确分配。
2. 再确认串口、配置和硬件状态。
3. 未标定或旧限位不可信时，先标定。
4. 标定保存后必须立刻用新限位复检。
5. 已有标定文件重新打开时，只需要检查当前硬件状态是否仍在保存的限位内；不需要默认重标定。
6. 只有“已有标定 + 当前 status 通过”才进入控制阶段。

## Step 0: Assign Servo IDs

舵机 ID 分配是所有 SDK 操作的前置条件。SDK 按 `configs/soarm.yaml` 中的 ID 寻址，不会从总线自动推断机械臂关节顺序。

默认配置期望的 ID：

| Joint | Servo ID |
| --- | ---: |
| `shoulder_pan` | 1 |
| `shoulder_lift` | 2 |
| `elbow_flex` | 3 |
| `wrist_flex` | 4 |
| `wrist_roll` | 5 |
| `gripper` | 6 |

这一步既不是 CLI 也不是 Web GUI 的职责。请用舵机厂商工具或底层 Feetech SDK 流程分配 ID。如果多个新舵机还共享出厂默认 ID，应一次只连接一个舵机进行改号。

## Route A: CLI Bring-Up

CLI 路线更直接，也更适合可复现的脚本、维护任务和示教数据采集。

### 1. 扫描并保存串口

```bash
soarm ports --config configs/soarm.yaml
```

如果只发现一个候选串口，该命令会写入 `arm.port`。如果有多个候选串口，请显式指定：

```bash
soarm ports --config configs/soarm.yaml --port /dev/cu.usbmodem5A7C1190351
```

只查看候选串口、不修改配置：

```bash
soarm ports --config configs/soarm.yaml --no-update
```

### 2. 运行健康检查

```bash
soarm status --config configs/soarm.yaml
```

`status` 是只读命令。它会检查配置的舵机 ID 是否在线、当前位置能否读取、当前位置是否在当前软限位内、电压是否高于阈值。

标定前最重要的是 `calibration readiness`。如果通信、电压、原始位置读取都正常，但软限位失败，通常说明旧标定或占位标定不可信，此时可以进入重新标定。

### 3. 标定

```bash
soarm calibrate --config configs/soarm.yaml --output configs/soarm.yaml
```

CLI 标定会：

1. 先运行一次只读 status，并确认硬件达到 calibration-ready。
2. 禁用舵机扭矩。
3. 提示你把机械臂放到物理零位并采集 zero ticks。
4. 提示你自由扫动每个关节，记录全臂范围。
5. 把 sweep 结果转换成零点、方向和软限位。
6. 保存 YAML。
7. 立刻用新标定运行 post-calibration status。

如果标定已经保存，但 post-check 失败，不要直接运动。先看报告：机械臂可能停在新软限位外、电压过低，或某个舵机不再响应。

### 4. 读取和运动

```bash
soarm read --config configs/soarm.yaml --unit rad
soarm home --config configs/soarm.yaml
soarm move --config configs/soarm.yaml shoulder_pan=0.0 elbow_flex=0.5
soarm pose --config configs/soarm.yaml ready
```

运动命令会通过 `SOARM.enable()` 启用连接，并在 `apply_on_enable: true` 时按需应用电机 profile。SDK 会在写目标前检查关节限位、单步限制、电压、速度可行性和加速度可行性。Web 推荐移动时间也使用同一套速度/加速度约束。

如果某个关节写入目标后没有跟随，可以用单关节 probe 打印底层寄存器闭环：

```bash
soarm probe-joint shoulder_pan --config configs/soarm.yaml --delta -0.1 --duration 1
```

重点看 `after_move` 里的 `Goal_Position`、`Present_Position`、`Torque_Enable`、`Operating_Mode`、`Moving`、`Present_Load`、`Present_Current`、`Status`。如果 `Goal_Position` 已经等于 `target_tick`，但 `Present_Position` 没变化，说明写入成功但舵机没有实际跟随。

### 5. CLI-only 维护能力

检查或应用 Feetech 电机 profile：

```bash
soarm configure-motors --config configs/soarm.yaml --check-only
soarm configure-motors --config configs/soarm.yaml
soarm configure-motors --config configs/soarm.yaml --force
```

捕获当前姿态为命名 pose：

```bash
soarm capture-pose --config configs/soarm.yaml ready --output configs/soarm.yaml
```

禁用所有舵机：

```bash
soarm disable --config configs/soarm.yaml
```

打印 URDF 路径或 XML：

```bash
soarm urdf
soarm urdf --print
```

### 6. 示教记录和回放

示教分成两个动作：

1. `record-demo`：读取当前关节角度并保存为 JSON 文件。
2. `replay-demo`：读取 JSON 文件，通过 SDK 的运动控制和安全检查回放。

记录默认会先关闭舵机扭矩，方便你手动拖动机械臂完成示教。记录前建议先确认当前配置和硬件状态：

```bash
soarm status --config configs/soarm.yaml
```

记录 10 秒、20 Hz 的全关节示教：

```bash
soarm record-demo demos/pick_place.json --config configs/soarm.yaml --duration 10 --hz 20
```

如果不传 `--duration`，记录会持续到你按 `Ctrl+C`。只记录部分关节：

```bash
soarm record-demo demos/elbow_only.json --config configs/soarm.yaml --joints shoulder_pan elbow_flex
```

如果你不想在记录前关闭扭矩，可以加 `--keep-torque`。无硬件验证可以加 `--mock`：

```bash
soarm record-demo /tmp/soarm-demo.json --config configs/soarm.yaml --mock --duration 1
```

回放前建议先 dry-run，只检查 JSON、关节名称、软限位和所选插值方法是否可构造，不打开硬件连接：

```bash
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --dry-run
```

真实回放会读取示教 JSON，先安全移动到第一帧，再把稀疏样本按时间戳插值为固定频率 setpoint 输出。默认输出频率使用 `arm.control_hz`，当前配置是 200 Hz，因此 20 Hz 的示教文件不会直接变成 20 Hz 舵机命令。默认插值方法是 shape-preserving cubic Hermite (`pchip`)；如果需要保守对照，可以切回逐关节线性插值：

```bash
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --speed 1.0
```

常用回放参数：

```bash
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --speed 0.5
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --start-at 2.0 --end-at 5.0
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --lead-in-duration 3.0
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --feedback-tolerance 0.03
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --output-hz 200
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --interpolation linear
```

`record-demo` 和 `replay-demo` 也可以简写为 `record` 和 `replay`。

示教 JSON 使用 `soarm.demonstration.v1` 格式，保存 `created_at`、`metadata`、`joints` 和 `samples`。每个 sample 记录 `time_from_start` 和以 rad 为单位的关节位置。这个格式适合人工检查，也方便后续转换成数据集或策略输入。

示教功能也可以作为模块复用。高层快捷函数：

```python
from soarm import SOARM, record_demonstration, replay_demonstration

with SOARM.from_config("configs/soarm.yaml") as arm:
    record_demonstration(
        arm,
        "demos/pick_place.json",
        duration=10,
        sample_hz=20,
    )

with SOARM.from_config("configs/soarm.yaml") as arm:
    replay_demonstration(arm, "demos/pick_place.json", speed=1.0)
```

如果需要更细的控制，可以直接使用 recorder/replayer 类：

```python
from soarm import SOARM, DemonstrationRecorder, DemonstrationReplayer, load_demonstration

with SOARM.from_config("configs/soarm.yaml") as arm:
    recorder = DemonstrationRecorder(arm, sample_hz=20)
    recorder.record(duration=10, output_path="demos/pick_place.json")

with SOARM.from_config("configs/soarm.yaml") as arm:
    demo = load_demonstration("demos/pick_place.json")
    DemonstrationReplayer(arm).replay(demo, speed=1.0)
```

回放仍然会复用 SDK 安全层；示教文件不会绕过软限位、插值后最大单步和急停检查。`lead-in` 阶段还会做电压、速度/加速度可行性检查，默认反馈到位容差是 `0.03 rad`，用于吸收编码器 tick、舵机保持误差和读回抖动。正式回放阶段使用 `SOARM.follow_joint_trajectory()`：`--speed` 只缩放示教时间轴，`--output-hz` 决定插值后的舵机 setpoint 下发频率，`--interpolation` 可选 `pchip` 或 `linear`。如果 replay 在 lead-in 阶段报告某个关节 remaining delta 很大，通常说明该关节没有跟随位置命令，或 `Present_Position` 读回没有更新。

## Route B: Web GUI Bring-Up

Web GUI 路线更适合首次带机和交互式调试。

```bash
soarm web --config configs/soarm.yaml
```

打开终端打印的本地 URL。默认地址类似：

```text
http://127.0.0.1:8765/
```

GUI 工作流分三页：

1. 检测：选择串口、保存 `arm.name`/`arm.port`、调整 runtime 频率、运行 status。
2. 标定：禁用扭矩、采集零点、开始/停止 sweep、查看每个关节的 raw ticks 和新限位、保存标定。
3. 可视化/控制：加载 SO-ARM101 URDF，持续读取实机状态；关节失能时滑条跟随实机，使能后滑条作为目标输入下发。

GUI 的控制阶段需要 `control_ready=true`，也就是：

- 配置里已有标定。
- 当前 status 全部通过。
- 当前关节位置在保存的软限位内。

如果你重新打开 GUI 时已经有标定文件，页面会自动跑一次只读 status。通过后进入可视化/控制；失败时保持在检测或标定流程中，提示你修复硬件状态、把机械臂手动带回限位内，或重新标定。

### GUI-only 交互能力

Web GUI 提供这些 CLI 没有的交互体验：

- 串口下拉选择、手动串口输入和机械臂名称编辑。
- runtime 频率表单，包括控制插值、反馈读取、Web 下发、Web 状态同步和渲染频率。
- 标定表格，显示每个关节的 zero、direction、raw tick range、safe tick limit 和 rad limit。
- URDF 3D 可视化和简化 2D fallback。
- 连续反馈采样，将实机关节状态刷新到模型和只读实际值。
- 关节失能时：可手动摆动实机，模型、实际值和滑条持续跟随反馈。
- 关节使能时：滑条作为目标输入，经 SDK 安全检查后下发到实机；模型和实际值仍持续跟随反馈。
- IK 表单：输入目标点，关节使能时把求解结果写入目标滑条；不会自动下发实机。

## Existing Calibration Rule

已有标定文件时，不需要默认重新标定。正确流程是：

```bash
soarm status --config configs/soarm.yaml
```

如果 status 通过，说明当前硬件状态和保存的软限位一致，可以进入运动或 GUI 控制。

如果 status 失败：

- 通信、电压或 raw position 失败：先修硬件。
- 只有软限位失败：可能是机械臂当前姿态在保存限位外，也可能是标定不适合当前装配。可以手动带回限位内后重跑 status，或重新标定。

GUI 会自动执行这次判断；CLI 用户需要显式运行 `status`。

## Kinematics

FK/IK 由 SDK 层实现，CLI 和 Web GUI 共用。

CLI 示例：

```bash
soarm fk --config configs/soarm.yaml shoulder_lift=-0.3 elbow_flex=0.5
soarm ik --config configs/soarm.yaml 0.42 0.0 0.20
```

Web GUI 中 FK/IK 用于计算工具端位置；IK 在关节使能时可把求解结果写入目标滑条。模型姿态和关节实际值始终来自实机反馈，IK 本身不会自动下发实机。

## Runtime Frequencies

| Layer | Default | Purpose |
| --- | ---: | --- |
| Vision / policy / trajectory input | 60 Hz | 产生高层目标 |
| Controller interpolation | 200 Hz | 把目标插值为 5 ms 一个点 |
| Serial position writes | 200 Hz | 向总线下发插值点 |
| Feedback sampling | 100 Hz | 读取/缓存真实关节状态 |
| Web state sync | 30 Hz | 把缓存状态同步到页面 |
| Web motion commands | 30 Hz | 限制滑条到硬件的命令频率 |
| Web rendering | 60 Hz | 绘制可视化模型 |

Web GUI 可以直接编辑这些 runtime 频率并保存到配置。CLI 目前没有专门的频率编辑命令；需要手动修改 `configs/runtime.yaml`。

`record-demo --hz` 是示教采样频率，默认 20 Hz。`replay-demo` 会把这些稀疏样本按 `--speed` 缩放后的时间轴插值为 `--output-hz` setpoint；不传 `--output-hz` 时使用 `arm.control_hz`。默认 `pchip` 插值比线性更平滑，同时避免普通 cubic spline 的典型过冲；`linear` 保留用于排查和保守回放。

当前 CLI replay 不启动独立的 100 Hz 反馈缓存线程；它在 lead-in 和安全检查处按需读取 `Present_Position`，正式回放按固定输出频率写插值后的 `Goal_Position`。

## Motor Profile Write Policy

电机 profile 寄存器不会在 200 Hz 控制循环里写。SDK 把实时路径保持在 `Goal_Position`，较慢或持久化的电机设置只在需要时应用。

| Register class | Examples | When to write |
| --- | --- | --- |
| Provisioning | `ID`, `Baud_Rate` | 一次性配置，SDK bring-up 不负责 |
| Calibration | `Homing_Offset`, position limits if enabled later | 只在标定变化后 |
| Motor profile | PID, `Return_Delay_Time`, `Maximum_Acceleration`, `Acceleration`, `Operating_Mode`, gripper protection | enable 时检查；只在漂移时写 |
| Enable state | `Torque_Enable` | 每次 enable/disable |
| Motion command block | `Acceleration`, `Goal_Position`, `Goal_Time`, `Goal_Velocity` | 每次位置命令按 Feetech STS 的 7 字节控制块一起写 |
| Feedback | `Present_Position` | 按配置的反馈频率 |

`status` 保持只读，不应用 motor profile。正常运动命令会调用 `SOARM.enable()`，在 `apply_on_enable: true` 时每个连接只应用一次 profile。

## Mock Mode

CLI：

```bash
conda run -n soarm-sdk python -m soarm.cli status --config configs/soarm.yaml --mock
conda run -n soarm-sdk python -m soarm.cli fk --config configs/soarm.yaml shoulder_lift=-0.3 elbow_flex=0.5
conda run -n soarm-sdk python -m soarm.cli ik --config configs/soarm.yaml 0.42 0.0 0.20
conda run -n soarm-sdk python -m soarm.cli record-demo /tmp/soarm-demo.json --config configs/soarm.yaml --mock --duration 1
conda run -n soarm-sdk python -m soarm.cli replay-demo /tmp/soarm-demo.json --config configs/soarm.yaml --dry-run
```

Web GUI：

```bash
conda run -n soarm-sdk python -m soarm.cli web --config configs/soarm.yaml --mock
```

Mock bus 用于无硬件开发、UI 验证和文档示例。它不能替代真实机械臂的舵机 ID 分配、供电、电缆和机械限位检查。

## SDK Quick Start

```python
from soarm import SOARM

arm = SOARM.from_config("configs/soarm.yaml")

with arm:
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

## CLI Reference

```bash
soarm ports --config configs/soarm.yaml
soarm web --config configs/soarm.yaml
soarm status --config configs/soarm.yaml
soarm configure-motors --config configs/soarm.yaml --check-only
soarm read --config configs/soarm.yaml --unit rad
soarm home --config configs/soarm.yaml
soarm move --config configs/soarm.yaml shoulder_pan=0.0 elbow_flex=0.5
soarm pose --config configs/soarm.yaml ready
soarm probe-joint shoulder_pan --config configs/soarm.yaml --delta -0.1 --duration 1
soarm record-demo demos/pick_place.json --config configs/soarm.yaml --duration 10 --hz 20
soarm record-demo demos/elbow_only.json --config configs/soarm.yaml --joints shoulder_pan elbow_flex
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --dry-run
soarm replay-demo demos/pick_place.json --config configs/soarm.yaml --speed 1.0 --output-hz 200 --interpolation pchip
soarm calibrate --config configs/soarm.yaml --output configs/soarm.yaml
soarm capture-pose --config configs/soarm.yaml ready --output configs/soarm.yaml
soarm disable --config configs/soarm.yaml
soarm urdf
soarm fk --config configs/soarm.yaml
soarm ik --config configs/soarm.yaml 0.42 0.0 0.20
```

## Architecture Boundaries

- `soarm.arm.SOARM`：用户和示例代码使用的 SDK facade。
- `soarm.hardware`：唯一直接调用 Feetech servo SDK 的层。
- `soarm.motion`：轨迹生成、运动安全验证和目标写入。
- `soarm.calibration`：零点、sweep 记录和软限位计算。
- `soarm.diagnostics`：只读硬件/config 健康检查。
- `soarm.kinematics`：SOARM FK/IK 和几何模型。
- `soarm.demonstration`：关节空间示教记录、JSON 文件读写和安全回放。
- `soarm.workflows`：CLI/Web 共享的应用层 payload 和流程函数。
- `soarm.cli`：命令行适配器。
- `soarm.web`：本地 HTTP API 适配器。
- `soarm.webapp`：静态 Web GUI，不拥有标定数学、运动安全或硬件规则。

## Package Layout

```text
src/soarm/
  arm.py            User-facing SOARM API
  config.py         YAML config loading and validation
  motor_profile.py  Drift-aware Feetech motor profile application
  model/            Arm, joint, pose, and state data models
  hardware/         SOARM servo bus wrapper and unit conversions
  motion/           Trajectory generation and motion controller
  kinematics/       SOARM-specific FK/IK and geometry model
  demonstration.py  Joint-space teaching record/replay utilities
  workflows.py      Reusable CLI/Web workflow payloads and operations
  web.py            Thin local HTTP adapter for the static Web console
  webapp/           Static visualization and interaction UI
  assets/soarm101/  Bundled SO-ARM101 URDF used by the web console
  safety/           Limits, voltage checks, and emergency-stop state
  calibration/      Zero and pose capture helpers
  diagnostics/      Hardware/config health checks
  testing/          Mock bus for no-hardware development
```
