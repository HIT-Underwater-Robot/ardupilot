# 轨迹、目标整形与 Guided

## 1. 不要在模式文件里寻找完整 S 曲线公式

`mode_auto.cpp` 和 `mode_guided.cpp` 主要选择目标、yaw 策略和运行哪个控制器。航点轨迹、速度/加速度限制和 jerk 整形主要位于：

- [libraries/AC_WPNav/](../libraries/AC_WPNav/)
- [AC_PosControl](../libraries/AC_AttitudeControl/AC_PosControl.cpp)
- 公共轨迹/输入整形函数

`AC_WPNav` 使用航点速度、加速度、jerk 和根据姿态控制能力计算的 snap/jerk 约束更新目标位置、速度和加速度。`AC_PosControl` 的 `input_*` 接口继续对位置、速度和加速度目标做限幅与 jerk-limited shaping。

因此，“启动是否平滑”是目标生成、整形、位置控制、姿态控制和混控共同作用的结果，不是给位置 PID 增加一个单独的 S 型开关。

## 2. 先识别目标类型

修改轨迹前先把需求写成以下之一：

| 目标类型 | 常见入口 | 需要确认的约束 |
|---|---|---|
| 位置 | Guided WP、mission waypoint | 最大速度、加速度、jerk、停止点、到达判定 |
| 速度 | Guided Velocity、PosHold 驾驶输入 | 加速度和 jerk、命令超时、位置稳定是否启用 |
| 位置 + 速度 | Guided PosVel | 目标积分、位置/速度一致性、超时后的速度归零 |
| 加速度 | `AC_PosControl::input_*accel*` | jerk、姿态能力、饱和 |
| 姿态 + 升沉 | Guided Angle | 倾角上限、升沉速度、消息超时、深度控制 |

同一个“向前走 1 米”的需求，可以被实现成 body-forward 速度、NE 速度、NE 位置或 Guided 航点，它们的坐标含义和松杆/超时行为完全不同。

## 3. PosHold 的速度整形

PosHold 将驾驶员 forward/lateral 解释为机体系水平速度，再通过 `body_to_earth2D()` 转为 NE 目标，随后调用：

```cpp
position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});
```

该接口不是直接把速度阶跃送到推进器。当前 `AC_PosControl` 会依据设置的最大加速度和 jerk 生成受约束的内部目标，再由 `NE_update_controller()` 计算控制输出。

## 4. Guided 的四条路径

### Guided WP

`guided_set_destination()` 设置航点，运行阶段调用 `wp_nav.update_wpnav()`。WPNav 负责目标轨迹，随后通过 `translate_wpnav_rp()` 变为艇体 lateral/forward 输出。

### Guided Velocity

`guided_set_velocity()` 设置速度目标。运行阶段停止 NE 位置稳定、更新 NE 速度控制，并把垂直速度交给 D 轴位置控制器。超过当前的 3 秒更新时间限制后，目标速度被置零。

### Guided PosVel

位置目标按速度和控制器 `dt` 推进，再通过 `input_pos_vel_accel_NE_cm()` 与垂直对应接口交给位置控制器。命令超时后速度目标归零，但位置目标和控制器状态仍需结合当前代码判断。

### Guided Angle

姿态请求受最大倾角限制，升沉速度受 WPNav 上下速度限制。消息超时后 roll、pitch 和升沉速度归零，再由姿态与深度控制器闭环执行。

## 5. 研究轨迹必须记录的量

- 外部命令时间戳和更新频率。
- 原始目标 position/velocity/acceleration。
- 整形后的 desired position/velocity/acceleration。
- 实际估计 position/velocity/attitude。
- NE 与 body 转换时使用的 yaw。
- 速度、加速度、jerk、snap 和倾角限制。
- 控制器限幅、积分器、motor limit flags 和各轴输出。
- 命令超时、position health 变化和 failsafe 时的目标重置。

## 6. 推荐验证场景

1. 静止开始的小幅速度阶跃，确认速度和加速度没有越限。
2. 正反向切换，确认制动和过零行为。
3. 不同 yaw 下相同 North 目标，确认 body 输出旋转正确。
4. Guided 更新正常、接近超时、超过超时三种情况。
5. 位置估计短时失效与恢复。
6. 推进器输出接近饱和时，确认轨迹误差和积分器恢复。

SITL 能证明软件路径和约束逻辑，但不能替代真实水动力、推进器死区和机体耦合验证。
