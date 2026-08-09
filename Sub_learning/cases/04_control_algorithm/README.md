# 案例 04：为 Precision Manual 增加输入斜率限制

> 设计练习：当前极简分支没有 Precision Manual。控制实验必须在独立分支保留 arming、spool、混控和回退路径。

> 状态：教学设计，依赖[案例 01](../01_new_flight_mode/README.md)中的新模式骨架，当前仓库尚未实现。本案例只改变新模式的目标输入整形，不修改现有 Manual、姿态/位置 PID、6DOF mixer 或 PWM 输出。

## 1. 需求定义

Precision Manual 的静态比例只能限制“最大请求”，不能限制驾驶员从 `-1` 突然打到 `+1` 时的变化速度。本案例为 forward、lateral 和升沉请求增加对称 slew-rate limiter，让目标按配置的每秒最大变化量接近输入。

第一版作用范围：

| 轴 | 内部整形范围 | motors 接口 | 是否限斜率 |
|---|---|---|---|
| Forward | `[-1,1]` | `set_forward()` | 是 |
| Lateral | `[-1,1]` | `set_lateral()` | 是 |
| Heave | `[-1,1]`，0 为中性 | 转换后 `set_throttle([0,1])` | 是 |
| Roll/Pitch/Yaw | 保持案例 01 的静态限制 | 原接口 | 第一版否 |

需求不声称能降低真实推进器电流或机械冲击到某个绝对值。归一化请求变化率不是 RPM、推力或安培；真实效果仍需 ESC、推进器和供电系统测试。

## 2. 算法接口

对每个归一化轴，定义：

- 输入 `u[k]`：当前模式的静态限幅目标，范围 `[-1,1]`。
- 状态 `y[k-1]`：上一周期 shaped target。
- 配置 `r`：最大变化率，单位 `normalized units/s`。
- 周期 `dt`：当前控制周期，单位秒。
- 输出 `y[k]`：本周期送入 motors 的目标。

离散公式：

```text
max_step = max(r, 0) * clamp(dt, 0, dt_max)
delta    = clamp(u[k] - y[k-1], -max_step, +max_step)
y[k]     = clamp(y[k-1] + delta, -1, +1)
```

这是一阶变化率限制，不是低通滤波器：当误差大时输出以固定斜率追赶，接近目标后直接到达，不引入指数尾巴。

## 3. 当前源码边界

| 源码 | 与本算法的关系 |
|---|---|
| [`ArduSub/mode_manual.cpp`](../../../ArduSub/mode_manual.cpp) | 展示六轴归一化输入到 motors 请求的最短路径 |
| [`ArduSub/mode.h`](../../../ArduSub/mode.h) | `Mode` 已持有 `G_Dt` 引用，可取得控制 loop 的秒周期 |
| [`ArduSub/Sub.cpp`](../../../ArduSub/Sub.cpp) | fast loop 中 mode run 后续进入 motors 输出，`run_rate_controller()` 更新 dt |
| [`libraries/Filter/SlewLimiter.h`](../../../libraries/Filter/SlewLimiter.h) | 名称相似，但当前类返回 PID P+D 的增益 modifier，用来抑制控制器输出振荡 |
| [`libraries/AC_PID/AC_PID.cpp`](../../../libraries/AC_PID/AC_PID.cpp) | 证明现有 `SlewLimiter` 的消费者是 PID slew protection |
| [`libraries/AP_Motors/AP_Motors6DOF.cpp`](../../../libraries/AP_Motors/AP_Motors6DOF.cpp) | 继续负责六轴混控、饱和和最终输出请求 |

不要直接复用 `Filter/SlewLimiter` 来完成本需求。当前这个类不是“把目标限制为每秒最多变化多少”的通用 target limiter；误用会改变 PID 增益语义，并不能保证 shaped input 的固定斜率。

## 4. 实现位置

第一版推荐把一个极小、无参数依赖的 helper 放在 `mode_precision_manual.cpp` 的本地实现范围内，或作为 `ModePrecisionManual` 的私有方法：

```cpp
float ModePrecisionManual::update_axis_limiter(float target,
                                                float &state,
                                                float rate_per_s,
                                                float dt)
{
    const float safe_target = constrain_float(target, -1.0f, 1.0f);
    const float safe_dt = constrain_float(dt, 0.0f, 0.05f);
    const float max_step = MAX(rate_per_s, 0.0f) * safe_dt;
    state += constrain_float(safe_target - state, -max_step, max_step);
    state = constrain_float(state, -1.0f, 1.0f);
    return state;
}
```

这是伪代码，`dt_max=0.05 s` 和具体 rate 是候选安全界限，必须结合实际 loop、暂停恢复行为和测试确定。

只有第二个模式或控制器需要完全相同语义时，才把 helper 提升到共享 library。不要一开始就在 `AP_Math` 增加“通用”API。

## 5. 状态生命周期

新增状态建议：

```cpp
float forward_shaped;
float lateral_shaped;
float heave_shaped;
```

| 事件 | 状态处理 | 原因 |
|---|---|---|
| `init()` | 三轴设为 0 | 新模式从中性请求开始，避免继承旧模式状态 |
| 未解锁的每个 `run()` | 三轴重置为 0 | 解锁前杆量不能在 limiter 中积累 |
| 正常 armed loop | 用受限 `G_Dt` 更新 | 保持与真实调度周期一致 |
| `G_Dt <= 0` 或非有限 | 本周期不移动状态 | 不允许非法 dt 产生跳变 |
| 调度长间隔 | 将 dt 限制到审查后的 `dt_max` | 恢复时不能一步跨越大距离 |
| 离开模式 | 状态随对象保留但下次 `init()` 必须 reset | 不依赖析构或动态对象 |

如果产品要求“已解锁切入时从当前 motor request 平滑接管”，那是不同的 bumpless-transfer 需求，需要可靠取得上一模式的同语义目标，不能简单用当前 PWM 反算；第一版以中性开始更容易证明。

## 6. 升沉轴的正确处理

驾驶员 throttle channel 的 `norm_input()` 是以零为中心的 `[-1,1]` 请求，而 `motors.set_throttle()` 使用 `[0,1]`，中性为 `0.5`。

正确顺序：

```text
raw throttle norm [-1,1]
 -> Precision static scale [-1,1]
 -> heave slew limiter [-1,1]
 -> 0.5 + 0.5 * shaped_heave
 -> motors.set_throttle([0,1])
```

这样上下方向使用对称斜率，松杆最终回到 `0.5`。在 `[0,1]` 上直接把值向零 slew 会把“零”误解为最大下沉请求。

## 7. 参数方案

为了让控制案例本身聚焦算法，第一阶段可使用编译期测试常量验证公式；产品化时再追加参数，例如：

| 候选参数 | 单位 | 语义 | 禁用语义 |
|---|---|---|---|
| `PREC_XY_SLEW` | normalized/s | forward/lateral 最大变化率 | `<=0` 是不限制还是冻结，必须二选一并写清 |
| `PREC_Z_SLEW` | normalized/s | heave 最大变化率 | 同上 |

更安全的产品语义通常是“0 表示不启用 limiter，直接使用静态限幅目标”，避免配置错误导致完全无法操纵；但这会在运行中产生一次跳变。另一种语义“0 表示冻结”适合测试，不适合一般用户。正式接口必须在[参数案例](../03_new_parameter/README.md)的方法下单独评审默认值、范围和 identity。

默认配置是否启用新算法取决于新模式的产品定义。无论如何，现有 Manual 的默认行为必须不变。

## 8. 调用伪代码

```cpp
void ModePrecisionManual::run()
{
    if (!motors.armed()) {
        // 保留案例 01 的 GROUND_IDLE、neutral 和 controller relax。
        reset_limiter_to_neutral();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const float forward_target = precision_scale(channel_forward->norm_input());
    const float lateral_target = precision_scale(channel_lateral->norm_input());
    const float heave_target = precision_scale(channel_throttle->norm_input());

    const float forward = update_axis_limiter(forward_target, forward_shaped, xy_rate, G_Dt);
    const float lateral = update_axis_limiter(lateral_target, lateral_shaped, xy_rate, G_Dt);
    const float heave = update_axis_limiter(heave_target, heave_shaped, z_rate, G_Dt);

    motors.set_forward(forward);
    motors.set_lateral(lateral);
    motors.set_throttle(0.5f + 0.5f * heave);
}
```

Roll、Pitch、Yaw 的现有 Precision 映射和整个 disarmed 分支在真实实现中不能省略。伪代码只突出算法位置。

## 9. 日志设计

调试日志至少能同时观察：

| 字段 | 单位/范围 | 用途 |
|---|---|---|
| raw forward/lateral/heave | normalized `[-1,1]` | 复现驾驶员输入 |
| limited static target | normalized `[-1,1]` | 区分比例限制与 slew 限制 |
| shaped target | normalized `[-1,1]` | 验证斜率公式 |
| `dt` | s | 找调度异常和误差 |
| configured rate | normalized/s | 证明运行配置 |
| motor requests/output | 现有日志定义 | 检查 mixer 饱和和安全状态 |

不要每个 fast loop 发送 `STATUSTEXT`。使用结构化 logger，并在 feature guard/日志开关下控制开销。

## 10. 数学与单元测试

以 `rate=0.5 unit/s`、`dt=0.01 s` 为例：

| 场景 | 初值 | 目标 | 期望 |
|---|---:|---:|---|
| 第一步上升 | 0 | 1 | `0.005` |
| 连续 100 步 | 0 | 1 | 约 `0.5` |
| 达到目标附近 | 0.998 | 1 | 直接到 `1.0`，不越过 |
| 正反切换 | 0.5 | -1 | 第一步为 `0.495` |
| target 越界 | 0 | 2 | 先限制 target 为 1 |
| `dt=0` | 0.2 | 1 | 保持 `0.2` |
| 超长 dt | 0 | 1 | 只使用 `dt_max` 对应步长 |

对 forward、lateral、heave 使用同一 helper 时，要证明状态互不串扰。

## 11. SITL 与实机验证

### SITL

- 单轴阶跃 `0 → +1`、`+1 → -1`、`-1 → 0`。
- 三轴同时阶跃，确认每轴斜率独立。
- 改变 loop jitter，检查每秒总变化率而不是每周期固定步长。
- 未解锁长时间保持满杆后解锁，输出必须从中性开始。
- 模式快速切入/切出，确认 reset。
- 触发 pilot/GCS failsafe，确认算法没有延迟安全动作。
- 饱和组合输入，区分 shaped target 正确与 mixer 最终 saturation。

### Pixhawk 类目标板和实物

1. 记录 SITL 与目标板 build 的真实结果和固件体积。
2. 推进器拆桨或物理隔离，准备独立断电。
3. 小斜率、低幅度开始，逐轴观察 motor output 与时间。
4. 核对升沉中位、上下方向和 forward/lateral 方向。
5. 执行 disarm、safety、输入丢失和模式切换。
6. 只有台架通过后，才进行低功率系留水池测试；记录电流、响应延迟和操纵手可接管性。

## 12. 验收与回退

验收条件：

- 公式、单位、dt、状态和 reset 条件均有测试。
- 斜率限制只影响 Precision Manual 的三个选定轴。
- heave 中位仍为 `0.5`，正负方向正确。
- 现有 Manual、控制器、mixer 和 failsafe 行为不变。
- 日志能区分 raw、static-limited、shaped 和 final output。
- SITL、目标板、拆桨及必要水池结果均真实记录。

运行时回退可关闭 limiter 或把 Precision 模式切回 Manual；固件回退到加入算法前提交。不能通过把 rate 设成极大且不限制 dt 来假装关闭，因为调度异常仍可能导致数值问题。正式参数应提供明确、经过测试的禁用路径。
