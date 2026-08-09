# 案例 01：新增 Precision Manual 飞行模式

> 状态：教学设计，当前仓库尚未实现本模式。文中的模式号和参数名是候选接口，正式编码前必须再次审计当前分支、地面站兼容性和参数编号。

## 1. 需求定义

为近距离观察、机械臂作业和狭小空间操控增加 `Precision Manual` 模式。它仍然是六自由度人工直控，不引入姿态、深度或位置闭环，只把驾驶员的最大请求限制在较小范围。

可观察行为定义如下：

| 项目 | Manual 基线 | Precision Manual 目标 |
|---|---|---|
| Roll/Pitch/Yaw | 归一化输入直接送入 motors | 保持方向和中位，只缩小最大请求 |
| Throttle | `[-1,1]` 映射到 `[0,1]`，中位为 `0.5` | 围绕 `0.5` 缩放，不能把中位移走 |
| Forward/Lateral | 归一化输入直接送入 motors | 保持方向，只缩小最大请求 |
| 姿态/深度/位置保持 | 无 | 无 |
| 未解锁行为 | `GROUND_IDLE`、中性油门、放松姿态控制器 | 与 Manual 完全一致 |
| 现有模式 | 不变 | 不修改 Manual 的默认行为 |

第一版建议只提供一组保守的六轴比例，例如角运动和水平平移分别限制；具体数值必须由载具能力、操纵手和拆桨试验确定，不能由教程替产品决定。

## 2. 当前 4.7.0 源码证据

| 源码 | 当前职责 | 新模式为什么要读它 |
|---|---|---|
| [`ArduSub/mode_manual.cpp`](../../../ArduSub/mode_manual.cpp) | Manual 的解锁检查、spool 请求和六轴输入映射 | 新模式最接近的行为基线 |
| [`ArduSub/mode.h`](../../../ArduSub/mode.h) | `Mode::Number`、虚接口和所有模式类 | 增加模式号和派生类声明 |
| [`ArduSub/Sub.h`](../../../ArduSub/Sub.h) | 保存各模式静态对象和当前 `flightmode` 指针 | 新模式对象必须拥有与车辆相同的生命周期 |
| [`ArduSub/mode.cpp`](../../../ArduSub/mode.cpp) | `mode_from_mode_num()`、`set_mode()`、`update_flight_mode()` | 注册、进入检查、初始化和运行时分派 |
| [`ArduSub/Sub.cpp`](../../../ArduSub/Sub.cpp) | `FAST_TASK(update_flight_mode)` 与 motors 输出调度 | 证明 `run()` 位于真实 fast loop 中 |
| [`ArduSub/Parameters.cpp`](../../../ArduSub/Parameters.cpp) | `FLTMODE1` 的模式枚举元数据 | 让参数和地面站显示新模式 |
| [`ArduSub/GCS_Sub.cpp`](../../../ArduSub/GCS_Sub.cpp) | ArduSub 的 heartbeat/base-mode 报告 | 核验模式报告与地面站识别 |
| [`Tools/autotest/ardusub.py`](../../../Tools/autotest/ardusub.py) | ArduSub SITL 行为测试 | 增加进入、输出、切换和拒绝场景 |

当前 `ModeManual::run()` 的关键链路是：

```text
pilot/joystick channels
 -> channel_*->norm_input()
 -> ModeManual::run()
 -> motors.set_roll/pitch/yaw/throttle/forward/lateral()
 -> AP_Motors6DOF mixing
 -> SRV_Channels / HAL output
```

模式层提出 `GROUND_IDLE` 或 `THROTTLE_UNLIMITED` 请求，真正的安全约束、斜坡、混控和 PWM 输出仍在后续 `AP_Motors`、SRV 和 HAL 链路中完成。

## 3. 最小设计边界

### 3.1 模式身份

- 候选类名：`ModePrecisionManual`。
- 候选显示名：`Precision Manual`，四字符名需选择唯一且可辨认的值。
- 候选模式号：从当前未使用值中选择，但不得使用已经保留的 30；正式确定前要全仓搜索、核验 MAVLink/地面站和已有日志解释。
- `requires_GPS()` 与 `requires_altitude()` 均为 `false`。
- `allows_arming()` 是否与 Manual 一致，需要结合产品的 arming 策略确认；教程不额外放宽任何检查。
- 第一版不修改 `ModeManual`，把影响范围限制在新增类中。后续只有在重复逻辑已有充分测试时，才考虑抽取共享 helper。

### 3.2 输入缩放

Roll、Pitch、Forward 和 Lateral 可使用同一类公式：

```cpp
limited = constrain_float(raw * scale, -1.0f, 1.0f);
```

Yaw 仍应保留当前 Manual 使用的 `ACRO_YAW_P` 比例关系，再施加 Precision 限制；不能因为新增模式而悄悄改变原有 yaw 参数语义。

Throttle 必须先在以零为中心的升沉请求中缩放，再映射到 motors 的 `[0,1]`：

```cpp
const float heave = constrain_float(channel_throttle->norm_input() * heave_scale,
                                    -1.0f, 1.0f);
motors.set_throttle(0.5f + 0.5f * heave);
```

错误写法是把已经位于 `[0,1]` 的油门直接乘比例；那会把中性点从 `0.5` 移走，使松杆也产生升沉请求。

### 3.3 与参数和控制算法案例的关系

本案例只负责“模式骨架 + 静态限幅”。参数化比例可接续[案例 03](../03_new_parameter/README.md)，输入斜率限制可接续[案例 04](../04_control_algorithm/README.md)。第一次实现时应拆成可独立审查的提交，避免一次同时引入新模式、新参数和新算法。

## 4. 预计修改文件

| 文件 | 最小修改 | 不应做的事 |
|---|---|---|
| `ArduSub/mode.h` | 追加模式号和 `ModePrecisionManual` 声明 | 重排已有模式号 |
| `ArduSub/mode_precision_manual.cpp` | 实现 `init()`、`run()` 和模式属性 | 把 mixer/PWM 逻辑写入模式 |
| `ArduSub/Sub.h` | 增加静态模式对象 | 动态分配模式对象 |
| `ArduSub/mode.cpp` | 在 `mode_from_mode_num()` 增加映射 | 绕开 `set_mode()` 直接换指针 |
| `ArduSub/Parameters.cpp` | 追加 `FLTMODE1 @Values` 显示项 | 改已有枚举值或默认模式 |
| `Tools/autotest/ardusub.py` 或 `Tools/autotest/ArduSub_Tests/` | 增加模式行为回归 | 只测试固件能启动 |

是否需要改 GCS 文件必须以源码和实际 heartbeat/mode 报告为证据。不要因为文件名含 `GCS` 就机械修改。

## 5. `init()` 与 `run()` 骨架

下面是说明控制意图的伪代码，不可直接当作已审查实现：

```cpp
bool ModePrecisionManual::init(bool ignore_checks)
{
    position_control->set_pos_desired_U_cm(0);
    sub.set_neutral_controls();
    return true;
}

void ModePrecisionManual::run()
{
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 读取归一化输入，按已审查的比例缩放并 constrain。
    // throttle 必须围绕 0.5 映射。
    // 最后仍调用 motors.set_*，不直接写 servo/PWM。
}
```

实现审查的重点不是语法，而是新模式是否完整保留了 Manual 的 disarmed/spool/neutral 语义，以及所有输出是否仍经过 `AP_Motors6DOF`。

## 6. 模式进入、退出和失效策略

| 场景 | 预期行为 |
|---|---|
| 未解锁进入 | 可以选择模式，但只能请求 `GROUND_IDLE` 和中性输出 |
| 已解锁切入 | 从中性/受限请求开始；若以后加入状态算法，必须在 `init()` reset |
| 切回 Manual | Manual 恢复原始比例，不残留 Precision 状态 |
| RC/GCS 输入丢失 | 继续使用现有 pilot-input/GCS failsafe，不建立私有超时旁路 |
| safety/interlock 未许可 | 不绕过 `AP_Motors` 状态机 |
| EKF/深度不可用 | 因本模式不依赖位置和深度，不伪造相关健康条件 |
| 非法比例 | 在消费点 constrain；参数版本还要定义合法范围和默认值 |

## 7. 验证矩阵

### 7.1 静态检查

- 模式号在当前分支唯一，未占用保留值。
- `FLTMODE1` 到 `FLTMODE6` 的元数据能够显示新模式。
- `mode_from_mode_num()` 有且只有一个映射。
- 旧模式源文件除必要元数据外没有行为改变。
- 所有六轴请求仍进入 `motors.set_*()`，没有直接写 `SRV_Channels` 或 HAL。

### 7.2 SITL

| 测试 | 输入 | 通过标准 |
|---|---|---|
| 模式选择 | 参数、RC/GCS 选择新模式 | heartbeat 和日志报告正确模式号 |
| 未解锁 | 六轴满输入 | motor outputs 保持安全状态 |
| 单轴正负 | 每次只给一个轴 ±1 | 方向与 Manual 相同，幅值不超过设计比例 |
| Throttle 中位 | throttle 杆归中 | motors throttle 请求为 `0.5` |
| 组合输入 | 六轴组合到边界 | 每轴先正确限制，最终混控仍由 AP_Motors 饱和处理 |
| 快速切换 | Manual ↔ Precision Manual | 无状态残留、NaN 或未定义突跳 |
| failsafe | 中断 pilot input/GCS | 现有 failsafe 动作未被新模式屏蔽 |

SITL motor output 只能证明软件映射、限幅和安全链，不能替代真实推进器方向和载具稳定性验证。

### 7.3 目标板与实机

1. 在 WSL 分别构建 SITL 和实际 Pixhawk 类目标板，记录命令和真实结果。
2. 不接动力时核对参数、模式显示、arming 拒绝和日志。
3. 接 ESC 前准备独立断电；推进器必须拆桨或物理隔离。
4. 逐轴从小输入开始，对照 Manual 检查方向和最大请求比例。
5. 完成 failsafe、disarm、safety 和模式切换检查后，才允许进入低功率系留水池测试。

## 8. 验收标准

- 现有 Manual 在默认参数下逐轴输出与基线一致。
- 新模式六轴方向与 Manual 一致，最大请求符合设计表。
- throttle 中位严格保持中性。
- 未解锁、disarm、safety、interlock 和 failsafe 行为未被绕过。
- 模式选择、heartbeat、日志和地面站显示一致。
- SITL、目标板构建和拆桨结果均有真实记录；未执行的测试明确标为未执行。

## 9. 风险与回退

主要风险是 throttle 中位换算错误、模式号冲突、复制 Manual 安全分支后产生行为漂移，以及操纵手误以为该模式具有姿态或深度保持。

回退分两层：

1. 运行时把 `FLTMODEn` 恢复为 Manual/Stabilize 等既有模式，不再选择新模式。
2. 固件回退到加入该模式前的已验证提交；参数文件和旧固件一并保存。

不要通过删除安全检查来“修复”无法解锁或没有输出的问题。先沿 mode → motors spool state → SRV/HAL 和 failsafe 链查证真实原因。
