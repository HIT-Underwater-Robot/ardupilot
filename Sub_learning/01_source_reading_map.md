# 源码阅读地图：从入口到推进器输出

## 1. 生命周期入口

ArduSub 的全局车辆对象和 HAL 生命周期入口位于 [ArduSub/Sub.cpp](../ArduSub/Sub.cpp)：

```cpp
Sub sub;
AP_Vehicle& vehicle = sub;
AP_HAL_MAIN_CALLBACKS(&sub);
```

`AP_HAL_MAIN_CALLBACKS(&sub)` 把 `Sub` 接入 AP_HAL/AP_Vehicle 的初始化和循环框架。分析启动、调度或板级行为时，应继续追踪 AP_Vehicle 与对应 HAL，而不是在 `ArduSub/` 中寻找一个传统的 `main()`。

## 2. Fast loop 的关键顺序

[Sub::scheduler_tasks](../ArduSub/Sub.cpp) 中与控制链最相关的 fast tasks 顺序是：

```text
AP_InertialSensor::update
    -> Sub::run_rate_controller
    -> Sub::motors_output
    -> Sub::read_AHRS
    -> Sub::read_inertia
    -> Sub::check_ekf_yaw_reset
    -> Sub::update_flight_mode
```

这是一条持续循环的实时流水线。`run_rate_controller()` 在非 Manual、非 Motor Detect 模式下调用 `attitude_control.rate_controller_run()`；`motors_output()` 再进入推进器库。不要只根据一个循环内的文字排列，武断地把某个目标值和某个 PWM 视为同一步计算，调试时应结合时间戳、控制器状态和日志确认。

`fifty_hz_loop()` 还执行驾驶员输入 failsafe 检查、读取遥控输入和模式开关。改变输入处理时，必须同时审计 fast loop 与 50 Hz 路径。

## 3. 飞行模式是运行时分派

[Sub::update_flight_mode()](../ArduSub/mode.cpp) 的核心只有：

```cpp
flightmode->run();
```

这里不是“调用整个 `mode_manual.cpp`”，而是通过 `Mode` 基类虚函数分派到当前对象的 `run()`。[Sub::mode_from_mode_num()](../ArduSub/mode.cpp) 返回 `ModeManual`、`ModeStabilize`、`ModeAlthold`、`ModePoshold`、`ModeGuided` 等静态模式对象；这些对象在 [ArduSub/Sub.h](../ArduSub/Sub.h) 中由 `Sub` 持有。

模式切换由 `Sub::set_mode()` 负责，当前源码会依次处理：

1. 是否已经处于目标模式。
2. 模式号能否映射为有效对象。
3. 目标模式需要的位置估计是否可用。
4. 从不要求高度的模式切换到要求高度的模式时，高度估计是否有效。
5. 新模式 `init(false)` 是否成功。
6. 旧模式退出清理、当前模式指针更新、日志和 heartbeat 通知。

因此，增加或修改模式不能只改 `run()`；还要检查模式选择、初始化条件、退出清理、failsafe 回退和 GCS/MAVLink 入口。

## 4. 实际的控制与输出对象

[ArduSub/Sub.h](../ArduSub/Sub.h) 中的实际推进器对象是：

```cpp
AP_Motors6DOF motors;
```

这决定了阅读 `AP_Motors` 时只需沿 `AP_Motors6DOF -> AP_MotorsMulticopter -> AP_Motors` 的实际继承路径追踪，不需要横向读完 Heli 等无关实现。

主要控制对象包括：

- `attitude_control`：姿态外环目标与角速度内环。
- `pos_control`：North-East 和垂直方向的位置/速度/加速度控制与目标整形。
- `wp_nav`：航点、样条和轨迹约束。
- `inertial_nav`、`ahrs`：状态估计接口。
- `motors`：推进器许可状态、6DOF 力分配和最终输出转换。

## 5. 分层职责

| 层 | 典型位置 | 主要职责 |
|---|---|---|
| 输入和状态 | `ArduSub/radio.cpp`、GCS/MAVLink、AHRS、InertialNav | 取得驾驶员指令、外部目标和估计状态 |
| 模式策略 | `ArduSub/mode*.cpp` | 选择目标类型、限制、控制器和回退行为 |
| 姿态控制 | `libraries/AC_AttitudeControl/` | 姿态误差生成角速度目标，陀螺反馈生成 R/P/Y 控制量 |
| 位置与轨迹 | `AC_PosControl`、`AC_WPNav` | 位置/速度/加速度目标、限速、限加速度、jerk/S 曲线整形 |
| 推进器状态与混控 | `libraries/AP_Motors/` | armed/interlock 约束、spool 状态、6DOF 分配、饱和限制 |
| 硬件输出 | `SRV_Channels`、`AP_HAL`、`AP_HAL_ChibiOS` | 通道功能、PWM/数字输出和定时器硬件 |

## 6. 库名前缀只表示职责线索

- `AP_*`：跨车辆基础设施、传感器、状态估计、任务、HAL 上层和通用库。
- `AC_*`：主要源于 Copter 的姿态、位置、航点和避障控制，也被 ArduSub 复用。
- `AR_*`：主要面向 Rover，不应为了命名统一而引入 ArduSub。

前缀不是严格依赖边界。[ArduSub/wscript](../ArduSub/wscript) 明确列出 `AC_AttitudeControl`、`AC_WPNav`、`AP_InertialNav`、`AP_Motors` 等直接库，并叠加 `ap_common_vehicle_libraries()` 的公共依赖。判断依赖必须看 `wscript`、include、feature guard 和链接结果。

## 7. 每次阅读使用同一张记录表

研究一个行为时，至少记录：

1. 输入来源和有效范围。
2. 当前模式与进入条件。
3. 坐标系、正方向和单位。
4. 目标类型及其限制器。
5. 使用的状态估计量和有效性检查。
6. 控制器输出怎样转换到 roll/pitch/yaw/throttle/forward/lateral。
7. armed、interlock、failsafe、safety switch 和 spool 状态。
8. 最终 SRV 功能与物理推进器通道。
9. 能证明结论的源码函数、SITL 测试或日志字段。
