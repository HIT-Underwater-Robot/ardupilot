# 推进器、6DOF 混控与硬件输出

## 1. 模式层提交的是状态请求

飞行模式调用 `motors.set_desired_spool_state()` 时，只是在请求以下三种目标状态之一：

- `SHUT_DOWN`：推进器应回到停止状态。
- `GROUND_IDLE`：推进器处于地面/低能量待命状态。
- `THROTTLE_UNLIMITED`：启动流程不再限制正常推力范围。

请求不会绕过安全条件，也不是一次布尔检查。实际动态类型是 [AP_Motors6DOF](../libraries/AP_Motors/AP_Motors6DOF.cpp)，其状态逻辑沿 [AP_MotorsMulticopter](../libraries/AP_Motors/AP_MotorsMulticopter.cpp) 路径运行。

## 2. armed/interlock 是硬约束

当前 `AP_MotorsMulticopter::set_desired_spool_state()` 在未 armed 或 interlock 未满足时强制把目标设为 `SHUT_DOWN`。`output_logic()` 每个周期推进实际状态机，并再次执行安全约束。

实际 `SpoolState` 包含：

```text
SHUT_DOWN
    <-> GROUND_IDLE
    <-> SPOOLING_UP / SPOOLING_DOWN
    <-> THROTTLE_UNLIMITED
```

状态机会处理启动/停止斜坡、safe-time、idle-time、spool-up block、当前限制和输出许可。修改模式请求并不等于修改状态转换；排查“已解锁但无推力”时，要同时记录 desired state、actual state、armed、interlock、safety switch 和 block 条件。

## 3. ArduSub 当前基线的输出入口

[Sub::motors_output()](../ArduSub/motors.cpp) 在普通路径中执行：

```text
clear ArduSub spool-up block
    -> skip direct output when MOTOR_DETECT owns the thrusters
    -> motor-test path or normal path
    -> motors.set_interlock(true)
    -> SRV cork
    -> SRV_Channels::calc_pwm()
    -> SRV_Channels::output_ch_all()
    -> motors.output()
    -> SRV push
```

当前基线会在 `motors.output()` 前每周期清除 spool-up block，因为 ArduSub 没有设置该 block 的起飞前 RPM 门。这是本分支已经存在的状态机配套逻辑；改动前必须联合审计 `output_logic()`，并重新执行 SITL、目标板构建和拆桨台架验证。

## 4. motors.output() 不是单一 PWM 写操作

`AP_MotorsMulticopter::output()` 当前依次完成：

1. throttle filter 和电池/推力能力更新。
2. `output_logic()` 推进 spool 状态。
3. `output_armed_stabilizing()` 计算允许的控制推力。
4. 机架补偿和 6DOF 力分配。
5. `output_to_motors()` 把推力转换为通道输出。
6. 输出附加 throttle、原始控制量和外部限制状态。

因此，PWM 异常可能来自模式目标、控制器饱和、spool 状态、混控系数、方向参数、SRV 功能分配或 HAL 定时器，不能只在 `ArduSub/motors.cpp` 中查找。

## 5. 6DOF 混控

[AP_Motors6DOF::add_motor_raw_6dof()](../libraries/AP_Motors/AP_Motors6DOF.cpp) 为每个推进器记录六个方向的系数：

```text
roll, pitch, yaw, throttle(vertical), forward, lateral
```

`output_armed_stabilizing_vectored()` 或 `output_armed_stabilizing_vectored_6dof()` 根据机架类型合成各推进器需求，并处理尺度和饱和。研究某个推进器方向时，应同时核对：

- 选择的 frame class/type。
- 推进器编号和 testing order。
- 六方向 mixing factors。
- `MOT_n_DIRECTION` 参数。
- SRV 通道功能与物理接线。
- ESC 中性点、最小/最大范围和协议。

## 6. 参数索引不可重排

`AP_Motors6DOF::var_info[]` 中现有 `AP_GROUPINFO` 索引已经写入用户参数存储约定。新增参数只能使用未占用索引；不得为了“看起来连续”而修改已有索引。删除过的索引也不应随意复用。

方向参数改变属于实机风险操作。必须逐通道验证方向，而不是一次性把整套机架参数写入带桨设备。

## 7. 输出问题的诊断顺序

```text
输入是否正确
 -> 模式是否接受该输入/目标
 -> 状态估计是否有效
 -> 控制器目标与输出是否合理
 -> desired/actual spool state 是否允许
 -> 6DOF mixer 是否饱和或方向错误
 -> SRV function/PWM 是否正确
 -> HAL timer/DMA/pin 是否正确
 -> ESC、接线和推进器是否正确
```

每一步都保留日志或示波器证据。不要通过临时绕过 arming、interlock、failsafe 或 safety 来“验证最后一级”。

## 8. 实机测试红线

- 首次测试拆桨或拆除推进器负载，必要时只接示波器/逻辑分析仪。
- 保证人员、线缆和工具不在旋转/喷水范围内。
- 预先验证硬件 safety、disarm、通信中断和独立断电。
- 一次只验证一个通道，记录编号、功能、方向、中性、最小和最大值。
- 完成空载验证后才进入受控水池测试；每一级都保留可回退固件和参数备份。
