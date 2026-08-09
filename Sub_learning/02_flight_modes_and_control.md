# 飞行模式与控制器

## 1. 所有模式都先过安全门

常见模式的运行路径都会检查 `motors.armed()`。未解锁分支请求 `GROUND_IDLE`、把 throttle 置于中性并放松相关控制器；解锁后才请求 `THROTTLE_UNLIMITED`。这个请求仍会被 AP_Motors 的 armed/interlock 状态机约束，不代表推进器立即获得全输出。

修改模式时，应保留这类未解锁分支，并核对该分支的返回范围，以及是否需要重新初始化位置目标、放松姿态控制器或更新保持航向。

## 2. Manual：最短的 6DOF 输入路径

[ModeManual::run()](../ArduSub/mode_manual.cpp) 是理解驾驶员输入到 6DOF 推力映射的最短路径。解锁后，当前源码基本直接写入归一化量：

```text
roll stick      -> motors.set_roll()
pitch stick     -> motors.set_pitch()
yaw stick       -> motors.set_yaw()
throttle stick  -> [-1, 1] 映射为 [0, 1] -> motors.set_throttle()
forward stick   -> motors.set_forward()
lateral stick   -> motors.set_lateral()
```

Manual 模式不运行常规姿态 rate controller；`Sub::run_rate_controller()` 对 `MANUAL` 和 `MOTOR_DETECT` 明确跳过。因此 Manual 适合定位输入映射、通道方向和混控问题，但不能代表稳定模式的闭环行为。

## 3. Stabilize：姿态闭环，平移仍由驾驶员直接控制

[ModeStabilize::run()](../ArduSub/mode_stabilize.cpp) 将驾驶员 roll/pitch 转换为目标倾角：

- roll/pitch 输入经过 `get_pilot_desired_lean_angles()` 和最大倾角限制。
- yaw 有输入时生成目标角速度。
- yaw 松杆后先给约 250 ms 减速窗口，再保持记录的绝对航向。
- throttle 经过姿态控制器的 throttle 输出接口。
- forward/lateral 仍直接使用驾驶员归一化输入。

姿态控制器的外环把姿态目标变为角速度目标；fast loop 的 rate controller 再用陀螺反馈生成 roll/pitch/yaw 控制量。调整 Stabilize 手感时，要区分输入映射、姿态外环、rate PID、滤波器和推进器饱和，不能只看模式文件。

## 4. AltHold：在 Stabilize 基础上增加垂直闭环

[ModeAlthold::run()](../ArduSub/mode_althold.cpp) 分三段执行：

```text
run_pre() -> control_depth() -> run_post()
```

`run_pre()` 处理姿态和航向，`run_post()` 写入 forward/lateral；`control_depth()` 完成垂直控制：

- 驾驶员 throttle 被解释为目标升沉速度，并受上下速度限制。
- 松杆进入死区后，位置控制器维持垂直位置目标。
- `at_surface` 和 `at_bottom` 状态会约束目标，避免持续向水面外或水底内积分。
- 接近水面时 `set_max_throttle()` 按距离限制最大垂直推力。

当前接口同时出现 `D_*` 控制器命名和 `get_position_z_up_cm()` 等 Up 轴量。任何修改都必须逐个确认 API 的轴定义，不能仅根据字母 D 猜正负号。

## 5. PosHold：机体系驾驶输入，地固 NE 闭环

[ModePoshold::run()](../ArduSub/mode_poshold.cpp) 同时运行姿态、深度和水平位置/速度控制。水平路径位于 `control_horizontal()`：

```text
pilot forward/lateral
    -> body frame velocity request
    -> ahrs.body_to_earth2D()
    -> North-East velocity target
    -> AC_PosControl input_vel_accel_NE_cm()
    -> position controller output
    -> translate_pos_control_rp()
    -> body lateral/forward effort
    -> AP_Motors6DOF
```

“目标向 North、零速度、保持位置”必须在地固坐标系中理解。NE 误差怎样变成艇体前后/左右推力由当前 yaw 决定；艇体转向后，同一个 North 误差对应的 forward/lateral 分量会变化。

位置估计失效时，当前代码在允许驾驶速度的情况下退回按归一化速度请求手动平移。改变该路径时，必须明确 position health、failsafe 和驾驶员接管策略。

## 6. Guided：按目标类型选择控制器

[ModeGuided::run()](../ArduSub/mode_guided.cpp) 根据 `sub.guided_mode` 分派：

| Guided 子模式 | 主要目标 | 运行函数 |
|---|---|---|
| `Guided_WP` | 位置/航点 | `guided_pos_control_run()` |
| `Guided_Velocity` | 速度 | `guided_vel_control_run()` |
| `Guided_PosVel` | 位置 + 速度 | `guided_posvel_control_run()` |
| `Guided_Angle` | 姿态 + 升沉速度 | `guided_angle_control_run()` |

速度、位置速度和姿态请求都有超时处理。当前源码在相关 Guided 更新超过 3 秒后会将速度或姿态/升沉请求收敛到安全目标。修改 MAVLink Guided 行为时，要从消息解析继续追到 `guided_set_*()`、超时字段、目标整形、yaw mode 和最终模式运行函数。

## 7. 推荐的开发优先级

本仓库默认按以下顺序理解和维护控制模式：

```text
Manual -> Stabilize -> AltHold -> PosHold -> Guided
```

AUTO/mission 仍被保留，但不是默认的首要产品路径。除非需求明确涉及任务航行，不应优先在 `mode_auto.cpp` 做宽泛重构。

## 8. 修改模式的最小审计面

每次模式变更至少检查：

- 输入来源、死区、归一化范围和单位。
- 模式进入条件、`init()`、`run()`、退出清理和回退路径。
- armed、interlock、pilot input failsafe、EKF/位置/高度有效性。
- body/NE/垂直坐标转换及正方向。
- 目标位置、速度、加速度、姿态或角速度的限制器。
- 姿态/位置控制器输出到 `motors` 的转换。
- 饱和、限幅、积分器状态和松杆行为。
- SITL 场景、日志字段、Pixhawk4 构建和必要的拆桨台架测试。
