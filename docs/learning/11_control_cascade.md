# 第 11 章：从目标到位置和姿态控制

控制器不负责决定任务目的地，也不负责产生 PWM。它位于模式和混控之间：

~~~text
Mode 产生目标
    ↓
位置/速度控制
    ↓
姿态角目标
    ↓
角速度目标
    ↓
归一化力矩/垂向推力
    ↓
AP_Motors6DOF
~~~

ArduSub 水平平移还有专门路径，把位置控制器的 roll/pitch 形式输出转换为 lateral/forward 推力。

## 1. 文件地图

1. **ArduSub/mode_guided.cpp**：目标怎样进入控制器；
2. **ArduSub/mode_althold.cpp**：深度控制；
3. **ArduSub/mode_poshold.cpp**：水平位置保持；
4. **ArduSub/Attitude.cpp**：车辆姿态辅助；
5. **ArduSub/motors.cpp**：水平控制量转换；
6. **libraries/AC_AttitudeControl/**；
7. **libraries/AC_PID/**；
8. **libraries/AC_WPNav/**。

## 2. 为什么使用串级控制

位置误差不能直接稳定地转换为某个电机 PWM，因为中间还有速度、惯性、姿态和车辆布局。

典型串级结构：

~~~text
位置误差
    ↓ 位置 P
期望速度
    ↓ 速度 PID/前馈
期望加速度或水平控制量
    ↓
姿态/平移目标
    ↓ 姿态控制
期望角速度
    ↓ 角速度 PID
力矩指令
~~~

外环慢、内环快。内环必须及时抑制扰动，外环才可以把它近似为可控执行对象。

## 3. 目标和估计怎样相减

以局部位置为例：

~~~text
desired_position
    -
estimated_position from AHRS/EKF
    =
position_error
~~~

估计值来自第 9 章的数据融合，不是最新一帧原始传感器。

目标还会经过：

- 速度和加速度限制；
- jerk 限制或轨迹整形；
- 输入滤波；
- 地形/水面/水底约束；
- Guided 超时；
- 模式手动覆盖。

所以控制器跟踪的是内部整形目标，不一定等于外部消息瞬间给出的阶跃。

## 4. Guided 位置控制主线

~~~text
SET_POSITION_TARGET_LOCAL_NED
    ↓
guided_set_destination
    ↓
WPNav destination
    ↓ 每轮
ModeGuided::guided_pos_control_run
    ↓
wp_nav.update_wpnav
    ├─ 水平导航输出
    └─ 垂向位置目标
    ↓
translate_wpnav_rp
    ↓
motors.set_lateral / set_forward
    ↓
position_control.D_update_controller
    ↓
attitude_control 输入 yaw/roll/pitch
~~~

ArduSub 的水平推进器可以直接产生平移力，因此 WPNav 传统的 roll/pitch 形式输出被归一化后转为 lateral/forward。

## 5. Guided 速度控制

ModeGuided::guided_vel_control_run：

1. 检查 armed；
2. 设置 spool 为 THROTTLE_UNLIMITED；
3. 检查 setpoint 超时；
4. 停止水平位置稳定，运行 NE velocity controller；
5. 根据垂向速度推进 D 轴目标；
6. 运行垂向控制；
7. 把水平控制结果转为 lateral/forward；
8. 把 yaw 目标交给姿态控制器。

速度目标不是直接写 motors.forward；它先与 EKF 速度反馈形成闭环。

## 6. 位置速度联合目标

Guided_PosVel 每轮按目标速度推进目标位置：

~~~text
pos_target += vel_target × dt
~~~

再把位置、速度和零加速度目标交给位置控制器。

这样外部规划器既给参考位置，又给轨迹前馈速度。若只低频跳点而不处理时间和速度连续性，内部控制会看到不平滑目标。

## 7. 深度轴的 Up 与 Down

MAVLink 常用 NED Down，位置控制接口存在 U/Up 与 D/Down 命名。

阅读每个函数要记录：

- 输入是 NEU 还是 NED；
- z 正方向；
- 单位是米还是厘米；
- get_pos_estimate_U_m 与 set_pos_desired_U_cm 的单位不同。

仅凭变量叫 altitude 或 depth 不足以判断符号。

## 8. 姿态控制两层

模式可能给：

- roll/pitch/yaw 角；
- roll/pitch 角 + yaw rate；
- body angular rate。

姿态控制器把姿态误差转换为角速度目标；rate controller 再把角速度目标与陀螺仪反馈比较，输出 roll/pitch/yaw 力矩请求。

Acro 主要从角速度目标进入内环，Stabilize 从姿态角进入外层。

## 9. 任务表形成一拍流水线

当前 FAST_TASK 顺序中：

~~~text
run_rate_controller
→ motors_output
→ read_AHRS
→ read_inertia
→ update_flight_mode
~~~

因此本轮 mode.run 更新的姿态/位置目标，会由后续主循环的 rate controller 和 motor output 持续消费。系统是高频流水线，不是一个函数从目标一直同步调用到 PWM。

不要为了让源码看起来“从上往下直观”而随意重排快速任务；时序变化本身就是控制逻辑变化。

## 10. dt 为什么每轮更新

Sub::run_rate_controller 读取 scheduler 的 last_loop_time_s，并设置给 motors、attitude_control 和 pos_control。

积分和微分都依赖 dt：

~~~text
积分增量约等于 error × dt
微分约等于 Δerror ÷ dt
~~~

假定永远精确 0.0025 s 会在调度抖动或过载时产生误差。

## 11. PID 四个关注点

### P

当前误差越大，纠正越强。过大易振荡，过小响应慢。

### I

累积长期误差，可抵消浮力不平衡、重心偏差和稳态水流。需要 anti-windup 和限幅。

### D

对变化趋势提供阻尼，但对噪声敏感，常配滤波。

### Feed Forward

根据目标变化直接给预期控制量，不等待误差出现。

调参必须基于日志和逐层闭环，不应同时修改所有轴和所有环。

## 12. 饱和怎样影响上游

推进器达到极限时，实际控制量无法继续增加。Motors limit flags 可告诉控制器某方向受限，积分器应避免继续累积。

若忽略饱和：

~~~text
长期大误差
→ I 项持续累积
→ 离开饱和后仍输出过大
→ 严重过冲
~~~

## 13. 滤波的代价

滤波降低噪声，但增加相位延迟。DVL、外部里程计、IMU 和深度数据若在香橙派和 Pixhawk 两端重复重滤波，可能让反馈明显滞后。

每个滤波器都要知道：

- 输入采样率；
- 截止频率；
- 延迟；
- 位于估计前还是控制器内；
- 上游是否已经滤波。

## 14. 控制器为何不直接写 PWM

控制器只产生物理轴意义的归一化请求。这样：

- 同一控制器可用于不同推进器布局；
- 混控器处理几何系数；
- SRV 处理通道映射和端点；
- HAL 处理 PWM/DShot 硬件；
- Safety 和 spool 状态有统一落点。

若新模式直接写 hal.rcout，它会绕过这些约束，是危险设计。

## 15. 大小脑接口建议

香橙派输出：

- ODOMETRY：状态观测；
- Guided position/velocity setpoint：运动目标。

Pixhawk 保留：

- EKF 与状态有效性；
- 目标超时；
- 位置/速度/姿态/rate 闭环；
- 推进器混控；
- 解锁和 failsafe。

不要让香橙派发送逐电机 PWM 作为常规控制方式，否则小脑失去闭环与安全保护价值。

## 16. 验收问题

1. 为什么位置误差不直接变 PWM？
2. 串级控制内外环谁更快？
3. Guided position 与 velocity 走哪些不同分支？
4. PosVel 的速度目标有什么前馈意义？
5. ArduSub 为什么把某些 roll/pitch 输出转成 lateral/forward？
6. 本轮 Mode 目标何时被 rate controller 消费？
7. dt 错误会影响哪些项？
8. 饱和为何引起积分 windup？
9. 滤波为什么不能只看降噪？
10. 伴随计算机为何不应常规发送逐电机 PWM？
