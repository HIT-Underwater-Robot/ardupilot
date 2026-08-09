# ArduSub 极简系统架构与源码地图

本文只描述本实验分支的真实代码。它不再枚举完整官方 ArduSub 的 Auto、Guided、位置控制和全部板卡，而是沿 Pixhawk1 上 `Manual / Stabilize` 的实际数据流阅读。

## 1. 总图

| 一级层 | 二级职责 | 当前源码入口 | 向下游交付 |
|---|---|---|---|
| 生命周期 | HAL 回调、车辆对象、实时调度 | `ArduSub/Sub.cpp`、`Sub.h`、`system.cpp` | 初始化后的对象和按频率运行的任务 |
| 输入 | RC/IOMCU、MAVLink Manual Control、joystick 映射 | `radio.cpp`、`RC_Channel_Sub.*`、`joystick.cpp`、`GCS_MAVLink_Sub.*` | 归一化 roll/pitch/yaw/throttle/forward/lateral 与模式请求 |
| 估计 | IMU、罗盘、压力、AHRS/EKF3 | `sensors.cpp`、`AP_AHRS`、`AP_NavEKF3` | 姿态、角速度、航向和传感器健康 |
| 模式 | 选择、初始化、运行时虚函数分派 | `mode.h`、`mode.cpp`、`mode_manual.cpp`、`mode_stabilize.cpp` | 直接六轴力请求，或姿态/角速度目标 |
| 控制 | 姿态外环、rate PID、驾驶目标换算 | `Attitude.cpp`、`AC_AttitudeControl_Sub.*`、`AC_PID` | roll/pitch/yaw 控制量 |
| 推进器 | armed/interlock/safety、spool、6DOF 混控 | `motors.cpp`、`AP_MotorsMulticopter`、`AP_Motors6DOF` | 每个 motor 的受限归一化输出 |
| 硬件输出 | servo function、PWM、IOMCU、ChibiOS | `SRV_Channel`、`AP_IOMCU`、`AP_HAL_ChibiOS` | Pixhawk1 MAIN/AUX 输出 |
| 诊断安全 | arming、failsafe、日志、漏水/电池 | `AP_Arming_Sub.*`、`failsafe.cpp`、`Log.cpp` | 解锁许可、失效动作和可观测证据 |

## 2. 生命周期与 fast loop

`ArduSub/Sub.cpp` 中：

```cpp
Sub sub;
AP_Vehicle& vehicle = sub;
AP_HAL_MAIN_CALLBACKS(&sub);
```

`AP_HAL_MAIN_CALLBACKS` 把 `Sub` 接入 AP_Vehicle/HAL 生命周期。控制主线的 fast tasks 顺序是：

```text
AP_InertialSensor::update
→ Sub::run_rate_controller
→ Sub::motors_output
→ Sub::read_AHRS
→ Sub::check_ekf_yaw_reset
→ Sub::update_flight_mode
```

这是跨周期流水线，不是一次函数调用从输入立即走到同周期 PWM。分析时要结合调度频率、状态更新时间和日志。

## 3. 两种模式

### 3.1 Manual

`ModeManual::run()` 的最短链路：

```text
armed / spool request
→ channel roll/pitch/yaw/throttle/forward/lateral
→ motors.set_roll / set_pitch / set_yaw / set_throttle / set_forward / set_lateral
→ AP_Motors6DOF
```

Manual 不运行常规姿态 rate controller，但仍受 armed、interlock、safety、spool 状态机、输出限幅和 SRV/HAL 约束。

### 3.2 Stabilize

`ModeStabilize::run()` 的控制链：

```text
roll/pitch 驾驶输入 → 目标姿态
yaw 有输入 → 目标角速度
yaw 松杆 → 保持航向
→ AC_AttitudeControl 外环生成角速度目标
→ gyro 反馈进入 roll/pitch/yaw rate PID
→ AP_Motors6DOF → SRV/HAL
```

forward、lateral 和升沉仍由驾驶员控制。Stabilize 是理解“目标—反馈—控制量—混控”的最小闭环案例。

## 4. 模式分派与模式号

`Sub::update_flight_mode()` 只调用：

```cpp
flightmode->run();
```

实际执行哪个 `run()` 由 `flightmode` 指向的静态对象决定。当前 `Mode::Number` 只有：

| 模式 | 数值 | 类 |
|---|---:|---|
| Stabilize | 0 | `ModeStabilize` |
| Manual | 19 | `ModeManual` |

`mode_from_mode_num()`、`set_mode()`、RC 模式参数、GCS heartbeat/available modes 都必须与这两个身份一致。加入新模式不能只新增一个 `.cpp`。

## 5. 姿态控制器

| 文件 | 责任 |
|---|---|
| `ArduSub/Attitude.cpp` | 驾驶员角度/角速度目标换算，yaw reset 处理 |
| `AC_AttitudeControl/AC_AttitudeControl.*` | 姿态误差、输入整形、姿态外环与角速度目标 |
| `AC_AttitudeControl/AC_AttitudeControl_Sub.*` | Sub 的 roll/pitch/yaw rate 控制与 motors 写入 |
| `AC_PID/AC_PID.*`、`AC_P.*`、`AC_PI.*` | 实际保留的一维 PID/P/PI 基础实现 |

PID 输出不是 PWM。它先成为三个转矩方向的归一化控制请求，再与升沉和水平力一起进入 6DOF 混控。

## 6. Spool 与 6DOF 输出

模式层通过 `set_desired_spool_state()` 请求：

- `SHUT_DOWN`
- `GROUND_IDLE`
- `THROTTLE_UNLIMITED`

它是带 armed/interlock/safety 约束的状态机请求，不是一次布尔判断。真正输出发生在后续：

```text
Mode request
→ AP_MotorsMulticopter::output / output_logic
→ AP_Motors6DOF mixing
→ SRV_Channels motor functions
→ AP_IOMCU / AP_HAL_ChibiOS RCOutput
→ PWM
```

任何推进器构型或控制器修改都必须沿整条链审计限幅、方向和 failsafe。

## 7. Pixhawk1 硬件范围

对外唯一板卡名是 `Pixhawk1`。`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/hwdef.dat` 内部继承 `fmuv3/hwdef.dat`，所以仓库仍保留 `fmuv3` 作为实现基文件；它不是第二个对外支持目标。

当前探测的主要板载器件：

| 类别 | 保留 backend |
|---|---|
| IMU | MPU6000/Invensense、LSM9DS0 |
| Compass | HMC5843、LSM303D |
| Barometer | MS5611 |
| Battery | Analog |
| GPS | u-blox（单接收机） |
| RC/输出 | IOMCU、标准 PWM |
| Storage/log | FATFS 文件日志、RAMTRON/板载存储路径 |

CAN、DShot/ESC telemetry、SITL、Lua、OSD、rangefinder、terrain、optical flow、visual odometry、external AHRS、camera/mount 等均关闭并删除对应实现。

## 8. 公共库白名单

`ArduSub/wscript` 使用显式 `ap_libraries`，不再调用跨车辆公共依赖集合。保留库可按职责归组：

| 组 | 主要库 |
|---|---|
| 控制 | `AC_AttitudeControl`、`AC_PID`、`AP_Motors`、`Filter` |
| 估计/数学 | `AP_AHRS`、`AP_NavEKF`、`AP_NavEKF3`、`AP_DAL`、`AP_Math` |
| 传感器 | `AP_InertialSensor`、`AP_Compass`、`AP_Baro`、`AP_GPS`、`AP_BattMonitor`、`AP_LeakDetector` |
| 参数/日志/调度 | `AP_Param`、`AP_Logger`、`AP_Scheduler`、`AP_Stats`、`StorageManager` |
| 通信/输入/输出 | `GCS_MAVLink`、`RC_Channel`、`AP_RCProtocol`、`SRV_Channel`、`AP_SerialManager`、`AP_IOMCU` |
| HAL/板级 | `AP_HAL`、`AP_HAL_Empty`、`AP_HAL_ChibiOS`、`AP_BoardConfig`、`AP_Filesystem`、`AP_ROMFS` |
| 基础设施 | `AP_Common`、`AP_Arming`、`AP_Notify`、`AP_InternalError` 等 |

删除库的判据不是“看起来没用”，而是：feature guard 关闭、源码引用闭包清除、全新 configure、全量目标板构建和链接都通过。重新添加能力时必须反向完成同样的依赖审计。

## 9. 当前不具备的能力

- 没有 AltHold/PosHold/Guided/Auto/Surface/Motor Detect。
- 没有 mission、waypoint、position controller、trajectory shaping。
- 没有 SITL/autotest；本分支不能提供仿真回归证据。
- 没有其他板卡和 bootloader。
- 没有通用传感器扩展集合。
- 部分原 failsafe 动作因目标模式不存在而退化为 disarm。

因此，这个分支适合学习和拆桨台架，不适合直接下水。

## 10. 验证清单

### 每次代码变更

```bash
git status --short
./waf configure --board Pixhawk1 --out build_minimal_pixhawk1 --no-submodule-update
./waf sub -j4
./waf list_boards
```

记录真实任务数、固件大小、修改文件和未测风险。

### 进入硬件前

1. 拆桨或物理隔离所有推进器，准备独立断电。
2. 验证 boot、USB console、时钟、参数存储和 SD/FATFS。
3. 核对两个 IMU、两个 compass backend 和 MS5611 的枚举、方向、健康、温度与校准。
4. 核对 RC/MAVLink 输入超时、arming、safety、interlock 和 GCS failsafe。
5. 逐通道核对 motor function、方向、中位、范围和 disarm 输出。
6. 最后才允许受控系留水池验证。

## 11. 怎样开始二次开发

不要直接在这个极简分支继续堆功能。先用它定位改动层：

- 新模式：从 `mode.h`、`mode.cpp` 和 GCS/RC 模式身份开始。
- 新传感器：在完整基线上设计 frontend/backend、总线、健康、日志与 estimator 接口。
- 新参数：保留已有索引，定义默认兼容、单位、范围和迁移。
- 新控制算法：先锁定目标/反馈/输出接口，保留 motors 与安全链。
- 新估计算法：先做旁路 shadow/log-only，再讨论主估计器准入。
- 新板卡：在 HAL/hwdef 完成硬件适配，先 boot/总线/存储，推进器输出最后验证。

具体练习见 [10_case_catalog.md](10_case_catalog.md)。
