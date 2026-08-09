# 从零掌握 ArduSub：先建立系统心智模型

## 1. 学习目标不是“读完仓库”

ArduSub 是实时、安全关键的分层系统。初学者最容易犯的错误是打开 `libraries/` 后逐文件阅读，或者在一个 `mode_*.cpp` 中寻找完整算法。更有效的目标是：

> 选定一个可观察行为，从输入和状态估计开始，沿模式、控制器、混控和硬件输出一路追到底，并能说明每层的契约与失效行为。

先掌握主线，再按需求补数学、传感器、任务或硬件知识。

## 2. 六层系统架构

| 层 | 当前源码锚点 | 输入 | 输出 | 本层不负责 |
|---|---|---|---|---|
| 生命周期与调度 | `ArduSub/Sub.cpp`、`AP_Vehicle`、`AP_Scheduler` | HAL 时钟和任务表 | 按频率运行车辆任务 | 具体模式策略 |
| 输入与状态估计 | `radio.cpp`、`GCS_MAVLink`、`AP_InertialSensor`、`AP_AHRS`、`AP_NavEKF3` | RC/MAVLink 和传感器观测 | 姿态、角速度、位置、速度、深度及健康状态 | 决定当前模式目标 |
| 飞行模式策略 | `ArduSub/mode.cpp`、`mode_*.cpp` | 驾驶员意图、导航状态、failsafe 状态 | 姿态/速度/位置目标或直接力请求 | 直接产生硬件 PWM |
| 闭环与目标整形 | `AC_AttitudeControl`、`AC_PosControl`、`AC_WPNav` | 目标与估计反馈 | R/P/Y 控制量及平移力请求 | 判断硬件 safety |
| 推进器状态与混控 | `AP_MotorsMulticopter`、`AP_Motors6DOF` | 六自由度请求、armed/interlock 状态 | 各推进器受限输出 | 读取 DVL 或决定导航目标 |
| 通道与硬件 | `SRV_Channels`、`AP_HAL`、`AP_HAL_ChibiOS` | 推进器/舵机功能输出 | PWM/数字协议和定时器信号 | 修改控制策略 |

遇到新需求时，先判断它属于哪一层。板级 GPIO 特例不应写进模式；DVL 协议解析也不应塞进位置控制器。

## 3. 最重要的运行模型：两条跨周期主线

[`Sub::scheduler_tasks`](../ArduSub/Sub.cpp) 中关键 fast tasks 的当前顺序是：

```text
AP_InertialSensor::update
    -> run_rate_controller
    -> motors_output
    -> read_AHRS
    -> read_inertia
    -> check_ekf_yaw_reset
    -> update_flight_mode
```

它不能被理解为“模式先算完目标，然后同一次循环立刻输出 PWM”的普通同步调用栈。长期运行时有两条互相交接的流水线：

```text
低层高频主线：
最新陀螺数据
    -> rate_controller_run()
    -> 使用已经存在的角速度目标
    -> motors.output()
    -> 混控与硬件输出

高层目标主线：
传感器观测
    -> AHRS / EKF 更新
    -> flightmode->run()
    -> 生成或更新下一阶段使用的姿态/速度/位置目标
```

也就是说，系统依靠连续周期工作：低层先快速消费已有目标，高层随后用更新后的估计生成后续目标。调试“目标已经变了但本周期输出为何不同”时，必须看时间戳和连续循环，不能只凭源文件行号推断。

`fifty_hz_loop()` 还运行驾驶员输入和多项 failsafe 检查。输入问题不一定发生在 fast loop 内。

## 4. 一条数据怎样改变推进器

以 PosHold 中的水平驾驶输入为例：

```text
forward / lateral stick
    -> ModePoshold::control_horizontal()
    -> body frame 速度请求
    -> ahrs.body_to_earth2D()
    -> North/East 速度目标
    -> AC_PosControl
    -> translate_pos_control_rp()
    -> body lateral / forward 力请求
    -> AP_Motors6DOF
    -> 每个推进器的混控结果
    -> SRV_Channels / HAL
```

沿链路阅读时，每经过一个函数都记录四件事：

1. 数据类型：position、velocity、acceleration、angle、angular rate 或归一化力。
2. 坐标系：body、North-East、NED/NEU、Up 或 Down。
3. 单位：`cm`、`cm/s`、`cm/s²`、`rad`、`centi-degree` 或 `[-1,1]`。
4. 健康与限幅：数据何时有效，在哪里超时、饱和或降级。

只要其中一项说不清，就还没有真正理解这条链。

## 5. 用模式递进理解控制能力

不要一开始阅读 AUTO。按以下顺序，每次只学习新增加的一层：

| 模式 | 在前一个模式上新增什么 | 最值得追踪的问题 |
|---|---|---|
| Manual | 无闭环，六个驾驶输入直接进入 motors | 通道、归一化、方向和混控是否正确 |
| Stabilize | 姿态外环和角速度内环 | roll/pitch 角度、yaw rate/hold 怎样变成 R/P/Y |
| AltHold | 垂直位置/速度/加速度闭环 | throttle 怎样变成升沉速度和深度目标 |
| PosHold | 水平 NE 位置/速度闭环 | body 输入怎样转换为地固目标并再转回 body 力 |
| Guided | GCS 给定不同目标类型 | 位置、速度、位置+速度和姿态目标怎样分派与超时 |

完成这五个模式后，再学习 `AC_WPNav`、AUTO/mission 和轨迹细节会顺畅很多。

## 6. 初学者的五次源码练习

### 练习一：找到程序入口和任务表

在 [`ArduSub/Sub.cpp`](../ArduSub/Sub.cpp) 找到：

- 全局 `Sub sub`。
- `AP_HAL_MAIN_CALLBACKS(&sub)`。
- `scheduler_tasks`。
- `run_rate_controller()` 和 `update_flight_mode()`。

成果不是抄代码，而是画出“谁按什么频率调用谁”。

### 练习二：追踪 Manual 的一个摇杆

从 [`ModeManual::run()`](../ArduSub/mode_manual.cpp) 任选 forward 输入，追到 `motors.set_forward()`，再到 [`AP_Motors6DOF::output_armed_stabilizing()`](../libraries/AP_Motors/AP_Motors6DOF.cpp) 中的 `_forward_factor[]`。

成果是列出该输入的范围、正方向、各推进器系数和最终 PWM 中值。

### 练习三：比较 Stabilize 与 Manual

确认 `Sub::run_rate_controller()` 为什么跳过 Manual，却为 Stabilize 调用角速度控制器。继续追踪姿态外环目标和陀螺反馈。

成果是能解释“驾驶员输入相同，为何两种模式的输出含义不同”。

### 练习四：追踪 AltHold 垂直轴

沿 [`ModeAlthold::run()`](../ArduSub/mode_althold.cpp) 的 `run_pre() -> control_depth() -> run_post()`，记录输入升沉速度、目标深度、Up/Down 符号、surface/bottom 约束和 throttle 输出。

成果是一张带单位和正方向的垂直数据表。

### 练习五：追踪 PosHold 水平轴

沿 [`ModePoshold::control_horizontal()`](../ArduSub/mode_poshold.cpp) 标出 body 与 North-East 的两次转换，并假设 yaw 为 0°、90° 分别手算方向。

成果是能解释“保持 North 方向目标时，艇体转向后哪些推进器分量会变化”。

## 7. 什么时候算掌握了架构

在开始新功能前，至少能独立回答：

- `flightmode->run()` 是怎样由模式号变成具体派生类调用的？
- 姿态目标为什么不是同一函数栈直接变成 PWM？
- `requires_GPS()` 在当前模式切换中实际检查的是哪种位置条件？
- PosHold 为什么既使用地固 NE 坐标，又最终输出 body forward/lateral？
- `set_desired_spool_state()` 为什么不是简单的“允许输出”布尔开关？
- `FRAME_CONFIG` 怎样选择 `AP_Motors6DOF` 的系数表？
- 新传感器、新模式、新控制器、新混控和新板卡分别应该落在哪一层？

如果这些问题已经清楚，就可以进入[新模式开发](10_new_mode_development.md)或[新推进器构型开发](11_new_thruster_frame_development.md)，把架构知识变成可验证的工程改动。

## 8. 暂时不要做的事

- 不要为了“精简”删除看似没有直接调用的 `libraries/`。
- 不要先改 PID，再回头寻找坐标系和单位。
- 不要把传感器超时、EKF 健康、模式 failsafe 合并成一个模糊布尔值。
- 不要把 `motors.set_*()` 当作最终 PWM。
- 不要把 SITL 通过当作推进器方向和硬件 failsafe 已验证。
- 不要在 `master` 上进行教学实验；新功能使用独立分支并保留回退点。
