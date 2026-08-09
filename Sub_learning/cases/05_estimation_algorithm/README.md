# 案例 05：压力深度垂速 Shadow Estimator

> 状态：教学设计，当前仓库尚未实现。本案例只生成旁路估计与日志，严禁把输出接入 EKF、姿态/位置控制器、模式进入条件、arming 或 failsafe。

## 1. 为什么先做 shadow estimator

修改估计算法的第一步不是直接改 `AP_NavEKF3`。先用同一传感器数据建立一个“只观察、不控制”的旁路估计器，可以验证：

- 传感器实际单位和正方向；
- 采样时间、丢包和重置语义；
- 微分放大的噪声与滤波延迟；
- 与当前惯导垂直速度的差异；
- 静态、升沉、温漂和传感器故障数据是否足够支持后续立项。

本案例的需求是：用外部水压深度传感器对应的 barometer altitude，计算一个压力导数垂直速度，低通后和当前惯导 `velocity_z_up_cms` 一起写日志。

## 2. 当前 4.7.0 的真实数据契约

| 源码 | 当前事实 |
|---|---|
| [`ArduSub/system.cpp`](../../../ArduSub/system.cpp) | 启动时寻找 `BARO_TYPE_WATER`，保存 `depth_sensor_idx`，并把首个水压传感器设为 primary |
| [`ArduSub/sensors.cpp`](../../../ArduSub/sensors.cpp) | `read_barometer()` 更新 AP_Baro，并用 `barometer.healthy(depth_sensor_idx)` 更新深度健康 |
| [`libraries/AP_Baro/AP_Baro.h`](../../../libraries/AP_Baro/AP_Baro.h) | `get_altitude()` 的接口契约是“自最后校准起的相对高度，单位米” |
| [`ArduSub/Sub.cpp`](../../../ArduSub/Sub.cpp) | `update_altitude()` 以 10 Hz 调用 `read_barometer()`，随后写 CTUN 日志 |
| [`ArduSub/inertia.cpp`](../../../ArduSub/inertia.cpp) | `climb_rate = inertial_nav.get_velocity_z_up_cms()` |
| [`libraries/AP_InertialNav/AP_InertialNav.h`](../../../libraries/AP_InertialNav/AP_InertialNav.h) | `get_velocity_z_up_cms()` 明确为 z-up、cm/s |
| [`ArduSub/Log.cpp`](../../../ArduSub/Log.cpp) | CTUN 同时记录 baro altitude、inertial altitude 和 climb rate |

这里有一个很适合初学者的审查点：`sensors.cpp` 中函数上方仍写着“centimeters”，`surface_bottom_detector.cpp` 也有相似旧注释，但 `AP_Baro.h` 的公开接口以及 CTUN 中与米制目标的使用证明 `get_altitude()` 返回米。新代码必须服从 API 契约和实际消费者，不能复制过时注释。

统一坐标定义：

```text
baro_alt_up_m       : 米，上为正；下潜后通常为负
shadow_vel_up_cms   : 厘米/秒，上为正；上浮为正、下潜为负
inav_vel_up_cms     : 厘米/秒，上为正
display_depth_m     : 如需显示“深度向下为正”，应为 -baro_alt_up_m，并明确改名
```

本案例全部内部速度采用 up-positive，避免比较前来回改符号。

## 3. 算法定义

对两次有效压力高度样本：

```text
raw_vel_up_cms = (alt_up_m[k] - alt_up_m[k-1]) * 100 / dt_s
alpha          = dt_s / (tau_s + dt_s)
filt_vel       = filt_vel + alpha * (raw_vel_up_cms - filt_vel)
```

其中：

- `dt_s` 必须由样本时间戳差得到，不能假定永远精确 0.1 秒。
- `tau_s` 是一阶低通时间常数；候选值必须通过日志数据选择。
- 第一帧只建立基线，不产生有效速度。
- 输入高度和结果必须是 finite。
- 时间间隔过小、过大或传感器不健康时，不提交新有效估计。

滤波器只降低微分噪声，不能修复压力传感器的温漂、气泡、管路延迟、运动动态压强或错误校准。

## 4. 最小软件结构

建议把实验状态封装在 ArduSub 车辆层，而不是放进 `AP_NavEKF3` 或 `AP_Baro`：

```text
ArduSub/depth_shadow_estimator.h
ArduSub/depth_shadow_estimator.cpp
```

候选类契约：

```cpp
class DepthShadowEstimator {
public:
    void reset();
    void update(bool sensor_healthy, float altitude_up_m, uint32_t sample_ms);
    bool valid() const;
    float raw_velocity_up_cms() const;
    float filtered_velocity_up_cms() const;

private:
    bool initialised;
    bool output_valid;
    uint32_t last_sample_ms;
    float last_altitude_up_m;
    float filtered_velocity_up_cms_state;
};
```

类不持有 motors、AHRS、EKF 或 GCS 引用。输入由调用者显式提供，使算法可以用录制数据做确定性测试。

## 5. 调度位置

当前 `update_altitude()` 已在 10 Hz 顺序执行：

```text
read_barometer()
 -> Log_Write_Control_Tuning()
```

最小接入位置是读取成功后、写日志前：

```cpp
void Sub::update_altitude()
{
    read_barometer();

#if DEPTH_SHADOW_ESTIMATOR_ENABLED
    const bool healthy = ap.depth_sensor_present && sensor_health.depth;
    const float altitude_up_m = healthy ? barometer.get_altitude(depth_sensor_idx) : 0.0f;
    depth_shadow_estimator.update(healthy, altitude_up_m, AP_HAL::millis());
#endif

    // existing logging remains
}
```

上面是伪代码。真实实现还要防止同一 barometer 样本被重复消费：如果 AP_Baro 能提供该实例的更新时间，应优先使用真实 sample timestamp/sequence；否则要记录当前 10 Hz 调度假设和限制。

不要新增 fast-loop 任务来反复微分同一个 10 Hz 压力样本，那只会产生零值和间歇尖峰。

## 6. 健康、reset 和异常语义

| 场景 | 处理 |
|---|---|
| 无 `BARO_TYPE_WATER` | invalid，保持不输出估计 |
| depth sensor unhealthy | `reset()` 或进入等待新基线状态，不沿用旧高度微分 |
| 第一帧/恢复后第一帧 | 只保存高度和时间，invalid |
| `dt <= dt_min` | 拒绝该差分，避免除以接近零 |
| `dt > dt_max` | 重新建立基线，避免用长时间缺口算平均速度 |
| altitude 非 finite | 拒绝并 reset/invalid |
| raw velocity 超出物理候选范围 | 记录 rejection；不能静默作为正常样本 |
| disarm/arm | shadow 算法本身可继续记录，但日志必须区分状态；不能改变 arming |
| surface calibration 更新 | 需要检测/记录 reset，否则高度基准跳变会被误判为速度 |

`dt_min`、`dt_max` 和速度 gate 的数值不能由教程拍脑袋给定。应从 10 Hz 调度、AP_Baro sample cadence、载具最大升沉速度和真实日志确定，并写入设计记录。

## 7. 日志接口

建议新增一个独立、短名称的 logger message，而不是改变既有 CTUN 字段含义。候选字段：

| 字段 | 单位 | 含义 |
|---|---|---|
| `TimeUS` | s/内部 µs | 日志时间 |
| `AltUp` | m | 压力高度，上为正 |
| `RawVUp` | cm/s | 未滤波压力差分速度 |
| `FiltVUp` | cm/s | shadow 低通输出 |
| `InavVUp` | cm/s | 同时刻惯导垂速 |
| `Dt` | s | 实际样本间隔 |
| `Valid` | boolean/bit | shadow 是否有效 |
| `Reason` | enum/bitmask | 无传感器、unhealthy、first sample、bad dt、gate 等 |

日志消息名、字段类型、单位和 multiplier 必须按当前 AP_Logger 规范审查。不要复用已有字段装入不同单位，也不要只记滤波结果而丢掉 raw 和 validity。

第一版可放在 `HAL_LOGGING_ENABLED` 与专用实验 feature guard 下，默认关闭或明确只在开发构建开启。guard 关闭时不得留下成员、调用或链接依赖。

## 8. 预计修改文件

| 文件 | 修改 |
|---|---|
| `ArduSub/depth_shadow_estimator.h/.cpp` | 独立算法状态、update/reset/accessors |
| `ArduSub/Sub.h` | feature guard 下持有 estimator，声明日志函数（若采用车辆 logger） |
| `ArduSub/Sub.cpp` | 在 10 Hz 新压力样本后调用 update |
| `ArduSub/Log.cpp` 及日志结构声明处 | 写独立 shadow message |
| 构建配置/feature guard | 默认和裁剪策略；保持关闭时可编译 |
| 测试目录 | 确定性数据序列、故障和日志验证 |

明确不修改：

- `libraries/AP_NavEKF3/`
- `AP_AHRS` 的状态输出
- `AC_PosControl`、`AC_AttitudeControl`
- `mode_althold.cpp`、`mode_poshold.cpp`
- arming、failsafe 或 motors

一旦需求要求改这些消费者，就已经超出 shadow 案例，必须用数据结果重新立项。

## 9. 离线与单元测试

### 9.1 确定性序列

| 输入 | 期望 |
|---|---|
| 恒定高度、固定 dt | raw 约 0，filtered 收敛到 0 |
| 线性上升 `+0.1 m/s` | raw 约 `+10 cm/s`，filtered 同方向且有可量化延迟 |
| 线性下潜 `-0.2 m/s` | raw 约 `-20 cm/s` |
| 单个高度尖峰 | raw 出现异常，gate/reason 按设计；恢复不产生无限值 |
| 重复 timestamp | 拒绝除法，状态有限 |
| 长时间缺口 | 下一帧只重建基线 |
| unhealthy → healthy | 恢复第一帧 invalid，第二个有效样本才估速 |
| 高度基准跳变 | 被 reset/标记，不能当真实升沉 |

### 9.2 录包重放

用真实压力传感器数据离线重放多个候选 `tau`：

- 静止水池至少数分钟，统计均值、标准差和峰峰值。
- 多次已知方向的上浮/下潜，检查符号。
- 与 `InavVUp` 比较偏差、RMSE、相关性和相位延迟，但不把其中任何一个默认当“真值”。
- 标出 sensor health、arming、motor output、surface calibration 和模式切换时刻。
- 保留原始数据和脚本，避免只展示最漂亮的一段图。

## 10. SITL 与硬件验证

### SITL

- 静止、恒速上升、恒速下潜、速度反转。
- 压力噪声和离群值注入。
- 传感器断线/不健康和恢复。
- 调度抖动与长间隔。
- Manual、Stabilize、AltHold、PosHold 运行时对比 motor outputs，确认开启 shadow 后输出 bit-for-bit 或在允许容差内不变。

### Pixhawk 类目标板

- guard 开/关两种 board build，记录 flash/RAM 差异。
- 地面静置和水池静置长时间日志。
- 受控的缓慢上浮/下潜；推进器试验必须拆桨台架先行，再系留。
- 水压传感器断开、重连、气泡/管路扰动和 surface calibration。
- 检查 logger 开销、scheduler load 和丢包。

## 11. 从 shadow 到正式估计的准入门槛

只有以下证据齐全，才可以讨论将压力垂速用于正式估计：

1. 单位、符号和 timestamp 在所有目标硬件上已证明。
2. 静态噪声、动态延迟、温漂、异常值和断线数据足够。
3. 有明确的观测模型、measurement noise、gate、bias 和 reset 设计。
4. 已解释它与 EKF 当前 barometer/depth 融合的重复信息和相关噪声。
5. 有日志重放、SITL、多传感器冲突和全模式回归计划。
6. 有参数/固件回退和水池安全计划。

不能因为 shadow 曲线“看起来更平滑”就接入控制。平滑往往意味着延迟，且两个估计之间的相似性不等于精度。

## 12. 验收与回退

本案例的验收标准是“得到可信对照数据且不改变载具行为”，不是“让深度控制更好”。

- feature guard 关闭时与基线构建/行为一致。
- guard 开启时仅新增计算和日志，无任何 consumer。
- 单位、符号、dt、reset、health 和 invalid reason 均可验证。
- shadow 与 Inav 对照日志完整，原始数据可重放。
- SITL 和目标板真实构建完成，scheduler/logger 开销可接受。

运行时回退为关闭实验 guard/日志；固件回退到加入 estimator 前的提交。若发现任何 motor output、模式进入、arming 或 failsafe 因 shadow 状态而改变，应立即视为越界实现并撤回，而不是继续调参。
