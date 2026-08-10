# ArduSub 4.7.0 Minimal Pixhawk1 Lab

这是从官方 `Sub-4.7.0` 派生的**实验教学分支**。它不是主线固件，也不是完整 ArduSub 4.7.0：本分支只保留理解“手动输入—姿态闭环—6DOF 混控—Pixhawk1 输出”所必需的代码。

## 固定边界

| 项目 | 本分支保留 | 本分支不提供 |
|---|---|---|
| 飞行模式 | `MANUAL`、`STABILIZE` | AltHold、PosHold、Guided、Auto、Surface、Motor Detect 等 |
| 飞控板 | `Pixhawk1`（内部复用 `fmuv3` hwdef） | Pixhawk4、Cube、SITL、Linux 及其他板卡 |
| 控制主线 | 驾驶输入、AHRS/EKF3、姿态/角速度控制、spool 状态机、`AP_Motors6DOF`、SRV/HAL/IOMCU | 位置/航点/任务/轨迹控制、自动导航和避障 |
| 基本外设 | Pixhawk1 板载双 IMU、HMC5843/LSM303D、MS5611、模拟电池、u-blox GPS、IOMCU RC/PWM、漏水检测、MAVLink | CAN、DShot/ESC telemetry、Lua、OSD、camera/mount、rangefinder、terrain、optical flow、external AHRS 等 |
| 工程资料 | `Sub_learning/` 架构说明和默认不编译的动手实验 | 通用车辆、通用 SITL/autotest 平台和其他产品能力 |

根目录 Waf 只递归 ArduSub，`./waf list_boards` 只应列出 `Pixhawk1`。`modules/` 仍是上游记录的 Git submodule，不在本分支内修改。

## WSL 构建

```bash
./waf configure --board Pixhawk1 --out build_minimal_pixhawk1 --no-submodule-update
./waf sub -j4
```

预期产物：

```text
build_minimal_pixhawk1/Pixhawk1/bin/ardusub.apj
build_minimal_pixhawk1/Pixhawk1/bin/ardusub_with_bl.hex
```

禁止使用 `sudo ./waf`。本分支没有 SITL 目标，不要把 Pixhawk1 编译成功解释为控制算法、硬件 IO 或水下航行已经验证。

## 安全限制

- 这是架构阅读和台架实验代码，**不得直接作为主线产品固件发布**。
- 只保留两种模式后，原本请求 Surface/Hold 的部分 failsafe 已退化为 disarm；部署前必须逐项重新审计。
- 参数和日志集合已经随功能裁剪，不能假定与官方完整 Sub-4.7.0 二进制兼容。
- 任何 armed/推进器测试必须拆桨或物理隔离推进器，确认 safety、interlock、failsafe，并准备独立断电。
- 尚未完成 Pixhawk1 实机烧录、传感器枚举、RC、PWM 方向、断链和漏水触发验证。

## 分支关系

- `master`：团队长期维护的完整、审慎准入基线；实验代码不得直接合入。
- 本分支：极简阅读/教学实验，仅用于理解最短控制链。
- 新模式、新传感器、新参数或新控制器实验应从合适基线建立单独分支，取得构建、台架与回退证据后再讨论进入 `master`。

学习入口见 [`Sub_learning/README.md`](Sub_learning/README.md)。其中 `Sub_learning/labs/` 的参考 `.cpp/.h` 不在默认编译链中；学习者必须在自己的实验分支手工接入。
