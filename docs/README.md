# SOARM Documentation

README 是项目入口，适合快速了解 SDK 的用途、安装方式和主要命令。更长的流程、排障和开发约定放在这里，避免 README 继续变成所有内容的集合。

## Documents

- [Operation Manual](OPERATION_MANUAL.md)：带机、标定、CLI/Web 使用、示教回放、安全门禁和排障手册。
- [Current State](../CURRENT_STATE.md)：当前硬件控制策略、电机 profile、Web 语义和后续控制层改进方向。
- [Project README](../README.md)：项目概览、能力地图和常用命令速查。

## Suggested Reading Paths

首次接线调试：

1. 读 [Operation Manual](OPERATION_MANUAL.md) 的 Hardware Scope、Configuration Model 和 First Bring-Up Checklist。
2. 用 mock 命令确认本地环境能跑。
3. 分配舵机 ID，再执行串口、status、calibration、post-check。

日常开发：

1. 读 [Current State](../CURRENT_STATE.md) 了解当前控制边界。
2. 用 [Operation Manual](OPERATION_MANUAL.md) 的 Development And Validation 命令做本地校验。
3. 修改硬件、运动、安全、Web 时保持 README 中的 Architecture Boundaries。

准备发布到 GitHub：

1. 检查 `.gitignore`，避免提交缓存和本机临时文件。
2. 跑 mock 验证命令。
3. 确认目标仓库、默认分支、公开/私有策略和 GitHub 认证。
