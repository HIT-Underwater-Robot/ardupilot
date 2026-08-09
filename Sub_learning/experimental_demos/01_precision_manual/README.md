# Demo 01：只增加一个 Precision Manual 模式

## 学习目标

这个 demo 把需求锁定为：“保留输入、解锁/输出安全链和 6DOF 混控，只新增一个低速手动模式。”它用最少代码展示 ArduSub 的模式身份、对象注册、虚函数分派和每周期输出。

模式号是 `22`，名称是 `Precision Manual`，四字符名称是 `PRMN`。

## 真实调用链

```text
Sub::scheduler_tasks
  → FAST_TASK(update_flight_mode)
  → Sub::update_flight_mode()
  → flightmode->run()
  → ModePrecisionManual::run()
  → AP_Motors6DOF set_roll/pitch/yaw/throttle/forward/lateral
  → 后续 motors.output() / SRV_Channels / HAL
```

`run()` 是 `Mode` 的虚函数。新增一个 `.cpp` 文件不会自动运行；还必须完成下表中的接线。

| 文件 | 本 demo 的改动 |
|---|---|
| `ArduSub/mode.h` | 给 `Mode::Number` 增加 22，声明 `ModePrecisionManual` |
| `ArduSub/Sub.h` | 在 `Sub` 中建立唯一模式对象 |
| `ArduSub/mode.cpp` | 在 `mode_from_mode_num()` 中把数字 22 映射到对象 |
| `ArduSub/mode_precision_manual.cpp` | 实现 `init()` 和 `run()` |
| `ArduSub/Parameters.cpp` | 在 `FLTMODEn` 元数据中列出模式 22 |

## 控制行为

在 armed 状态下：

```text
roll/pitch/yaw = 原手动归一化命令 × 0.35
forward/lateral = 原手动归一化命令 × 0.40
heave = throttle 归一化命令 × 0.40
motors throttle = 0.5 + 0.5 × heave
```

这里不能直接把缩放后的 `[-0.4, 0.4]` 写入 throttle，因为 ArduSub 6DOF throttle 接口以 `0.5` 为中性点。转换后范围为 `[0.3, 0.7]`。

在 disarmed 状态下，代码沿用 Manual 的安全结构：请求 `GROUND_IDLE`、输出中性升沉，并放松姿态控制器。armed 后才请求 `THROTTLE_UNLIMITED`。模式没有绕过 arming、safety、interlock、spool 状态机或最终输出链。

## 为什么这是“只增加模式”

- 没有修改 RC/joystick 输入定义。
- 没有修改 AHRS/EKF。
- 没有修改姿态/位置控制器。
- 没有修改 `AP_Motors6DOF` 混控系数。
- 没有修改 SRV/HAL 或板卡定义。

因此出现问题时，审查范围集中在模式选择、输入缩放、中性点和 spool 请求。

## 实验步骤

1. 先编译 SITL，确认模式身份能链接进固件。
2. 启动 SITL，在 disarmed 状态选择 custom mode `22`，确认 heartbeat 报告 `custom_mode=22`。
3. 用 MAVLink Inspector 或日志观察输入与输出；先测试中位、正向、反向和饱和值。
4. 若进入硬件台架，只允许拆桨/隔离推进器，逐通道验证中性、方向和范围。

## 新人可以继续做什么

- 把固定比例改为新参数，但必须新增参数索引，不能改已有索引。
- 给 forward/lateral 加斜率限制，观察“模式策略”和“控制算法”的边界。
- 增加进入模式时的状态 reset，比较有状态和无状态整形器。

不要把它扩展成位置保持；一旦需要 DVL/EKF 状态，需求已经跨入估计与闭环控制，应另立实验。
