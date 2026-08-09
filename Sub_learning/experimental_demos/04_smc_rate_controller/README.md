# Demo 04：用边界层 SMC 示例替换 PID 角速度内环

## 学习目标

这个 demo 展示“修改底层姿态闭环算法”应切在哪一层：保留 Stabilize 的驾驶员输入、姿态外环、目标角速度、AP_Motors6DOF 混控和全部安全输出链，只在一个独立模式中把 PID rate controller 换成简单的边界层滑模示例。

模式号是 `23`，名称是 `SMC Stabilize`，四字符名称是 `SMCD`。原 `STABILIZE` 模式和原 PID 实现保持不变，便于 A/B 对照和立即回退。

## 控制链对比

```text
共同部分：
pilot roll/pitch/yaw
  → ModeStabilize::run()
  → attitude error / yaw-rate command
  → desired body angular rate

原模式：desired rate + gyro → PID rate_controller_run() → motors roll/pitch/yaw

模式 23：desired rate + gyro → rate_controller_run_smc_demo() → motors roll/pitch/yaw
```

`ModeSMCStabilize` 继承 `ModeStabilize`，没有复制外环代码。`Sub::run_rate_controller()` 只在当前模式为 23 时选择 SMC；其他受控模式仍调用原 PID。

## 教学控制律

每个轴的角速度误差为：

```text
e = desired_rate - measured_gyro_rate
s = e / boundary
sat(s) = constrain(s, -1, 1)
u = K × sat(s)
```

roll/pitch 共用 `ATC_SMC_RP_K`，yaw 使用 `ATC_SMC_Y_K`；输出仍是 AP_Motors 接受的归一化 roll/pitch/yaw 命令。

| 参数 | 默认值 | 范围 | 含义 |
|---|---:|---:|---|
| `ATC_SMC_RP_K` | 0.25 | 0.01–1.0 | roll/pitch 切换增益 |
| `ATC_SMC_Y_K` | 0.20 | 0.01–1.0 | yaw 切换增益 |
| `ATC_SMC_BOUND` | 0.15 rad/s | 0.01–2.0 | 误差边界层；层内输出线性变化，层外饱和 |

新参数使用 `AC_AttitudeControl_Sub` 中未占用的 group index 7–9；原有索引 0–6 未改变。进入或离开模式 23 时重置 PID I 项，避免回到 PID 模式后带入旧积分状态。

## 为什么它还不是产品级 SMC

这段代码只是清晰展示算法替换点，不能据此声称已经获得滑模控制的工程鲁棒性。它缺少至少以下内容：

- 根据潜航器惯量、附加质量、阻尼和执行器能力推导的滑模面；
- 等效控制项或模型补偿；
- 扰动上界和稳定性/到达条件证明；
- 采样、噪声、滤波、延迟和抖振分析；
- 推进器死区、饱和、速率限制及 anti-windup 等等效处理；
- 失效降级、参数准入和实机整定证据。

所以准确称呼是“rate-error boundary-layer SMC teaching demo”，不是可直接下水的 PID 替代品。

## 源码入口

| 文件 | 职责 |
|---|---|
| `ArduSub/mode.h` | 模式 23 继承 Stabilize 外环 |
| `ArduSub/mode.cpp` | 注册模式并在进出时 reset PID I 项 |
| `ArduSub/Sub.cpp` | 根据当前模式选择 PID 或 SMC rate controller |
| `libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h/.cpp` | 新参数和教学控制律 |

算法不应写进 `AP_Motors6DOF`：混控器负责把归一化力/力矩请求分配到推进器，不负责产生姿态闭环控制量。也不应直接写在模式 `run()` 中，否则会把外环策略、内环反馈和调度时序揉在一起。

## 建议验证路线

1. 编译 SITL 和 Pixhawk4，证明接口、feature guard、参数和链接完整。
2. disarmed SITL 选择模式 23，确认 heartbeat 报告 `custom_mode=23`；再切回 Stabilize，确认模式身份和回退路径。
3. 增加专用日志，记录 desired rate、gyro rate、error、surface、SMC output 和 motor limit；当前 demo 还没有这组完整观测量。
4. 在仿真中做小阶跃、正反向、边界层内外、饱和和传感器噪声测试，与原 PID 对照。
5. 只有完成模型审查和仿真后，才进入拆桨台架；任何有推进器的台架都必须物理隔离、有限幅和独立断电。
6. 系留水池与真实航行属于后续单独准入，不在本教学 demo 的验证范围内。

## 推荐的下一次教学改进

不要立即堆叠复杂公式。先补一组只读日志和 SITL 阶跃脚本，让新人能画出目标、反馈、误差和输出；随后再把控制律抽象为清晰接口，比较 PID、当前 boundary-layer 示例和一个有模型依据的候选算法。每次只改变一个层级，原 Stabilize 始终作为回退基线。
