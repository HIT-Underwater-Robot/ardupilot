# 实践二：开发一种新的 ROV 推进器构型

## 0. 先限定“新机型”的含义

本章的“新机型”是仍然运行 ArduSub 的新推进器数量、安装位置、方向和 6DOF 力分配，例如一套项目专用六推进器或八推进器 ROV。

它不表示：

- 增加 ArduPlane、Copter 或其他车辆产品；
- 把 Orange Pi 等伴随计算机变成飞控；
- 为新的 STM32 主控板写控制特例。

如果变化的是飞控硬件，应阅读[系统总览的板卡移植部分](01_source_reading_map.md#12-pixhawk-类-stm32chibios-板卡移植)。板卡适配解决 MCU、总线和输出资源；本章解决“六自由度控制请求怎样分配到物理推进器”。

## 1. 当前 4.7.0 的构型选择链

先沿源码把参数追到底：

```text
Parameters.cpp: FRAME_CONFIG
    -> g.frame_configuration
    -> Sub::init_rc_out() in radio.cpp
    -> motors.init(frame_class, MOTOR_FRAME_TYPE_PLUS)
    -> AP_Motors6DOF::setup_motors()
    -> add_motor_raw_6dof()
    -> roll/pitch/yaw/throttle/forward/lateral factor arrays
    -> output_armed_stabilizing()
    -> _thrust_rpyt_out[]
    -> output_to_motors()
    -> rc_write()
    -> SRV_Channels / HAL
```

当前 [`AP_Motors6DOF.h`](../libraries/AP_Motors/AP_Motors6DOF.h) 的 `sub_frame_t` 与 [`Parameters.cpp`](../ArduSub/Parameters.cpp) 的 `FRAME_CONFIG @Values` 对应：

| 值 | 枚举 | 参数显示 |
|---:|---|---|
| 0 | `SUB_FRAME_BLUEROV1` | BlueROV1 |
| 1 | `SUB_FRAME_VECTORED` | Vectored |
| 2 | `SUB_FRAME_VECTORED_6DOF` | Vectored_6DOF |
| 3 | `SUB_FRAME_VECTORED_6DOF_90DEG` | Vectored_6DOF_90 |
| 4 | `SUB_FRAME_SIMPLEROV_3` | SimpleROV-3 |
| 5 | `SUB_FRAME_SIMPLEROV_4` | SimpleROV-4 |
| 6 | `SUB_FRAME_SIMPLEROV_5` | SimpleROV-5 |
| 7 | `SUB_FRAME_CUSTOM` | Custom |

### 重要的 4.7.0 基线事实

[`AP_Motors6DOF::setup_motors()`](../libraries/AP_Motors/AP_Motors6DOF.cpp) 中的 `SUB_FRAME_CUSTOM` 只有注释，没有 `break`，当前会继续执行 `SUB_FRAME_SIMPLEROV_3` 的三推进器配置。

因此，设置 `FRAME_CONFIG=7` 并不会自动得到一张可由用户配置的任意混控表。“Custom”只是预留入口，正式使用前必须实现、审查和验证；这个事实也必须写入产品参数说明。

## 2. 先写物理规格，不先填系数

为每个推进器建立唯一编号和实物表：

| 推进器 | 机体位置 | 推力正方向 | 产生的力矩方向 | ESC/输出口 | 正向 PWM 结果 | 测试顺序 |
|---|---|---|---|---|---|---|
| T1 | x/y/z 实测值 | 单位向量 | `r × F` | 待定 | 待测 | 1 |
| T2 |  |  |  |  |  | 2 |
| … |  |  |  |  |  |  |

在项目规格中明确：

- 机体系轴定义和观察方向。
- 位置原点、长度单位和推进器作用线。
- 推进器“正转”产生的真实推力方向。
- ESC 正反向、中值、死区和最大/最小值。
- 每个推进器故障后剩余可控自由度。
- 电源、总电流和持续推力限制。

不要通过反复试错猜系数。先由几何与力矩 `r × F` 得到符号和相对比例，再把结果归一化为混控系数，最后用拆桨台架验证。

## 3. 理解六列系数真正做什么

`add_motor_raw_6dof()` 的参数顺序是：

```cpp
add_motor_raw_6dof(
    motor_number,
    roll_factor,
    pitch_factor,
    yaw_factor,
    throttle_factor,
    forward_factor,
    lateral_factor,
    testing_order);
```

对通用混控路径，第 `i` 个推进器的核心关系可按当前源码理解为：

```text
angular_i = roll * R_i + pitch * P_i + yaw * Y_i
linear_i  = throttle * T_i + forward * F_i + lateral * L_i
output_i  = constrain(reverse_i * (angular_i + linear_i), -1, 1)
```

这解释了每一行不是“电机参数”，而是一个六自由度控制分配向量：

| 列 | 正请求时该推进器的贡献 |
|---|---|
| Roll | 滚转力矩 |
| Pitch | 俯仰力矩 |
| Yaw | 偏航力矩 |
| Throttle | 垂直平移 |
| Forward | 前后平移 |
| Lateral | 左右平移 |

`_motor_reverse[]` 是最后施加的方向修正。不要同时随意翻转系数和 `MOT_n_DIRECTION`，否则物理定义会失去唯一真相。

`SUB_FRAME_VECTORED` 和 `SUB_FRAME_VECTORED_6DOF` 还有专用 `output_armed_stabilizing_*()` 路径。开发新构型时必须确认它使用通用混控还是需要有证据的新饱和分配算法，不能只复制名称相近的 frame。

## 4. 先在纸面验证控制分配矩阵

把所有推进器行组成矩阵后，逐轴做六次纯输入检查：

| 测试输入 | 其他轴 | 纸面检查 |
|---|---|---|
| Roll = +1 | 0 | 合力尽量为零，产生期望滚转力矩 |
| Pitch = +1 | 0 | 合力尽量为零，产生期望俯仰力矩 |
| Yaw = +1 | 0 | 合力尽量为零，产生期望偏航力矩 |
| Throttle = +1 | 0 | 产生期望垂直合力，附带力矩可接受 |
| Forward = +1 | 0 | 产生期望前向合力，附带力矩可接受 |
| Lateral = +1 | 0 | 产生期望侧向合力，附带力矩可接受 |

然后再检查：

- 矩阵是否有足够秩来控制声称支持的自由度。
- 对置推进器是否在应抵消的轴上抵消。
- 单轴满指令时是否过早饱和。
- 两轴或三轴叠加时，姿态与平移谁优先。
- 单推进器失效后是否出现不可控方向。
- 重心、浮心偏移和非对称推力是否需要通过机械设计或控制补偿，而不是随意改系数。

只有纸面矩阵能自洽，才进入源码。

## 5. 选择兼容的实现策略

### 策略 A：实现现有 `SUB_FRAME_CUSTOM`

适用于这个聚焦仓库只维护一套项目专用构型，并且已确认没有现场载具依赖当前 `FRAME_CONFIG=7` 的 SimpleROV-3 落入行为。

需要在 `case SUB_FRAME_CUSTOM` 中：

1. 设置明确的 `_frame_class_string`。
2. 为每个推进器调用 `add_motor_raw_6dof()`。
3. 使用唯一、连续且与实物标签一致的 `testing_order`。
4. 明确 `break`，禁止继续落入其他 frame。
5. 记录参数值 7 的行为迁移和回退方法。

### 策略 B：追加新的 frame 值

适用于必须保留既有参数行为，或未来确实要维护多种产品构型。

- 只在 `sub_frame_t` 末尾追加新枚举，不重排 0–7。
- 同步 `FRAME_CONFIG @Values`。
- 增加新的 `setup_motors()` case 和 frame string。
- 审计参数存储、GCS 显示和配置迁移。

不要改变 `FRAME_CONFIG` 参数本身的参数索引；枚举值兼容和 `AP_GROUPINFO/GSCALAR` 索引是两类不同问题，都要保护。

## 6. 最小源码改动面

| 文件 | 职责 | 常见错误 |
|---|---|---|
| [`AP_Motors6DOF.h`](../libraries/AP_Motors/AP_Motors6DOF.h) | 需要时追加 frame 枚举 | 重排已有值 |
| [`AP_Motors6DOF.cpp`](../libraries/AP_Motors/AP_Motors6DOF.cpp) | 定义系数表、测试顺序和 frame string | 漏 `break`、复制错误方向、无证据新增专用 mixer |
| [`ArduSub/Parameters.cpp`](../ArduSub/Parameters.cpp) | 同步 `FRAME_CONFIG @Values` 和说明 | 改参数索引或无迁移说明改默认值 |
| [`ArduSub/radio.cpp`](../ArduSub/radio.cpp) | 核验初始化链，通常不需要修改 | 为一个 frame 写板级输出特例 |
| `SRV_Channels`/板卡 hwdef | 核验输出能力和通道资源 | 把电机几何混控放进 HAL |
| [`Tools/autotest/ardusub.py`](../Tools/autotest/ardusub.py) 或库测试 | 验证参数选择、输出符号和安全状态 | 只证明固件能启动 |

大多数新构型不需要改模式、姿态控制器、位置控制器或 EKF。如果不得不修改这些层，说明需求已经不只是 frame，应拆成独立立项。

## 7. 从系数表到实物的验证阶梯

### 阶段一：静态和构建验证

- 两人独立核对几何表、轴定义、系数符号和测试顺序。
- 检查所有 motor number 唯一且未越界。
- 构建 SITL 与 Pixhawk4。
- 记录固件大小、编译警告和参数元数据。
- 确认旧 `FRAME_CONFIG` 值仍选择原构型。

### 阶段二：SITL/软件输出验证

对六个轴分别施加小幅正负请求，记录各 motor output 的符号并与纸面矩阵逐格比较。再验证：

- 未解锁为中性输出。
- armed、interlock、safety 和 spool 状态仍限制输出。
- 切换模式时没有未定义通道或突跳。
- 混合轴输入发生饱和时，limit 标志和输出符合设计。

仅修改固件混控表不会自动让 SITL 物理模型拥有新几何。因此 SITL 可以证明软件选择、符号、限幅和 failsafe 链；若未同步建立可信的动力学模型，不能用它证明真实 ROV 稳定。

### 阶段三：无推进器和拆桨台架

1. 飞控先不接动力，核对 `FRAME_CONFIG`、frame banner、`SERVOx_FUNCTION` 和输出通道。
2. 接 ESC 前准备独立断电，限制电源电流。
3. 拆桨或物理隔离推进器，逐个执行 motor test，确认实物标签与 `testing_order`。
4. 核对中值、正反转、最小/最大 PWM 和失联行为。
5. 对 roll/pitch/yaw/throttle/forward/lateral 分别给小指令，观察每个推进器方向。
6. 触发 disarm、safety、RC/GCS failsafe，确认回到预期输出。

如果某个推进器方向错误，先回到物理规格判断是线序/ESC 方向、`MOT_n_DIRECTION` 还是混控系数错误；不要用多处符号翻转把问题“调到能动”。

### 阶段四：系留水池

- 从低功率单轴开始，再做组合轴。
- 检查实际正方向、姿态耦合、浮心/重心偏置和电流。
- 先验证 Manual，再验证 Stabilize、AltHold、PosHold。
- 每次只改变一个因素并保存参数、日志和视频。
- 明确人工接管、漏水、电池和通信 failsafe。

自由航行必须建立在拆桨台架与系留测试均通过之后。

## 8. 新构型的测试记录模板

| 项目 | 应记录的证据 |
|---|---|
| 基线 | commit、分支、`FRAME_CONFIG`、板卡和参数文件 |
| 几何 | 推进器位置、方向、编号、照片和版本 |
| 矩阵 | 六列系数、推导方法、复核人 |
| 构建 | WSL 命令、SITL/Pixhawk4 真实结果、固件哈希 |
| 单轴输出 | 六个轴正负请求与每个推进器的期望/实际符号 |
| 安全状态 | disarmed、armed、interlock、safety、failsafe 输出 |
| 台架 | 中值、范围、方向、motor test 顺序和断电方案 |
| 水池 | 系留方式、功率限制、日志、异常和通过标准 |
| 回退 | 恢复旧 `FRAME_CONFIG`/旧固件/旧参数的具体步骤 |

## 9. 完成标准

一个新推进器构型只有同时满足以下条件才算完成：

- 物理布局与混控矩阵可由另一个工程师独立复核。
- 已有参数值和旧 frame 行为没有被无意改变。
- SITL 和 Pixhawk4 均真实构建通过。
- 六轴正负输出与纸面矩阵一致。
- arming、safety、interlock、spool 和 failsafe 没有被绕过。
- 拆桨台架逐通道核验通过。
- 系留水池逐级验证通过。
- 有参数、固件和源码三层回退方案。

“推进器都能转”只完成了最前面的硬件连通性检查，不代表混控、安全或闭环控制正确。
