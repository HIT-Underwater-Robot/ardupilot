# ArduSub 六类二次开发教学案例

> 本文是基于完整 ArduSub 4.7.0 架构的立项练习。当前极简分支只保留 Manual/Stabilize/Pixhawk1，案例所需的可选库可能已删除；实际实现必须从合适的完整基线另建实验分支。

如果目标是第一次亲手修改并构建代码，请先完成 [`labs/`](labs/README.md) 的 UART 与新模式渐进实验。本文回答“需求应该落在哪一层”，`labs/` 回答“一个文件怎样被复制、编译、持有、初始化、调度并验证”。

本项目的二次开发教学只围绕六条实际需求主线：

| 主线 | 要解决的问题 | 主要源码层 | 典型交付物 |
|---|---|---|---|
| 新增飞行模式 | 增加一种新的驾驶或作业行为 | `ArduSub/mode*.cpp` 及模式注册/GCS | 模式实现、参数、日志、SITL 和实机验证 |
| 添加新传感器 | 让飞控可靠读取一个明确型号的设备 | 对应 `AP_*` frontend/backend、HAL bus | driver、参数、健康状态、parser/unit test |
| 增加新参数 | 把固定行为变成可配置且兼容的接口 | `Parameters.*` 或公共库 `var_info[]` | 参数定义、元数据、默认值、迁移和测试 |
| 修改控制算法 | 改变目标整形、闭环或混控行为 | `ArduSub`、`AC_*`、`AP_Motors` | 数学说明、代码、日志、SITL/台架结果 |
| 修改估计算法 | 改变状态、观测、滤波、噪声或融合逻辑 | sensor frontend、`AP_AHRS/AP_NavEKF3` | 数据契约、回放/对照、创新和失效验证 |
| 新开发板 | 适配新的 Pixhawk 类 STM32/ChibiOS 板卡 | `AP_HAL_ChibiOS/hwdef`、必要 HAL | hwdef、bootloader、board build 和硬件记录 |

自动测试、日志、failsafe 和回退不再作为独立案例；它们是每一条开发主线必须包含的工程环节。

本目录已经为六类推荐需求各写一篇独立 README：

| 推荐顺序 | 需求类型 | 教学案例 | 当前状态 |
|---:|---|---|---|
| 1 | 新参数 | [新增航向保持过渡时间参数](cases/03_new_parameter/README.md) | 设计教程，未实现 |
| 2 | 新模式 | [新增 Precision Manual 飞行模式](cases/01_new_flight_mode/README.md) | 设计教程，未实现 |
| 3 | 控制算法 | [为 Precision Manual 增加输入斜率限制](cases/04_control_algorithm/README.md) | 设计教程，未实现 |
| 4 | 新传感器 | [新增 MCP9808 温度传感器 backend](cases/02_new_sensor_driver/README.md) | 设计教程，未实现 |
| 5 | 估计算法 | [压力深度垂速 Shadow Estimator](cases/05_estimation_algorithm/README.md) | 设计教程，未实现且禁止接入控制 |
| 6 | 新开发板 | [迁移 Pixhawk 类 STM32F765/ChibiOS 飞控板](cases/06_new_board/README.md) | 设计教程，等待真实硬件资料 |

所有模式号、参数名、参数索引、feature guard 和板卡名在正式编码前都必须重新核对当前 4.7.0 分支、地面站和硬件资料，不能把设计教程误认为已经实现的固件功能。

## 1. 新增飞行模式

| 编号 | 候选需求 | 最接近的现有模式 | 最小设计边界 | 难度 | 风险 |
|---|---|---|---|---:|---|
| [M01（推荐）](cases/01_new_flight_mode/README.md) | Precision Manual：限制六轴最大请求，用于近距离低速操控 | Manual | 保留直接操控语义，只增加比例和限幅，不新增 PID | 3 | 高 |
| M02 | Precision Stabilize：限制倾角、yaw rate、升沉和水平请求 | Stabilize | 复用姿态控制器，只修改目标约束 | 3 | 高 |
| M03 | 作业保深模式：保持姿态和深度，明确限制水平控制范围 | AltHold | 复用 AltHold，定义进入、退出、水平输入和接管策略 | 3 | 高 |
| M04 | 受限控制器验证模式，仅允许在 SITL/拆桨条件运行 | 无直接产品模式 | feature guard、严格 arming 条件、自动退出和一次性告警 | 4 | 极高 |

### 模式案例必须讲清

- 模式号、`Mode` 派生类、静态对象和 `mode_from_mode_num()`。
- `set_mode()` 的位置/高度检查、`init()`、`run()` 和退出清理。
- RC/joystick/GCS 怎样选择模式。
- 未解锁、输入超时、状态估计失效和模式回退。
- 新模式是否复用现有控制器，为什么不复制算法。

M01 最适合做第一个模式教程，但必须先确认“精细操控”需要限制哪些轴，以及限制值是否由参数提供。

## 2. 添加新传感器

传感器开发必须提供准确型号、官方 datasheet、总线电气信息和真实数据；不接受虚构的“通用串口协议”。

| 编号 | 候选需求 | 建议复用的抽象 | 主要职责 | 难度 | 风险 |
|---|---|---|---|---:|---|
| [S01（推荐）](cases/02_new_sensor_driver/README.md) | 增加 MCP9808 I2C 水温/舱内温度传感器 | `AP_TemperatureSensor` | probe、寄存器读取、换算、超时和健康 | 3 | 低 |
| S02 | 增加一款有明确协议的 UART 环境传感器 | 对应现有 frontend 或新 backend | 非阻塞串口、parser、校验、频率和重连 | 3 | 中 |
| S03 | 增加一款 downward rangefinder | `AP_RangeFinder` | 距离、质量、方向、超时和 SurfTrak 消费 | 4 | 高 |
| S04 | 增加 GPIO/I/O expander leak sensor backend | `AP_LeakDetector` | wet/dry、反相、debounce、断线和健康 | 3 | 高 |
| S05 | 增加新的 IMU backend | `AP_InertialSensor` | 高速采样、DRDY、FIFO、温度、旋转和校准 | 5 | 极高 |

### 传感器案例推荐结构

```text
datasheet/真实录包
 -> 独立 parser
 -> frontend/backend
 -> update/read 调度
 -> 单位/坐标/时间戳
 -> health/timeout
 -> 参数与日志
 -> consumer
 -> SITL/目标板/断线测试
```

S01 风险较低且 `AP_TemperatureSensor` 已是 ArduSub 直接依赖，适合作为第一个真实硬件 driver 教程。

## 3. 增加新参数

| 编号 | 候选需求 | 参数位置 | 教学重点 | 难度 | 风险 |
|---|---|---|---|---:|---|
| [P01（推荐）](cases/03_new_parameter/README.md) | 将 Stabilize/AltHold/PosHold yaw 杆回中后当前固定约 250 ms 的过渡时间参数化，默认保持原行为 | `ArduSub/Parameters.*` | 新索引、单位、范围、默认兼容和模式消费 | 1 | 中 |
| P02 | 为 Precision 模式增加各轴最大请求比例 | `ArduSub/Parameters.*` | 参数分组、多个轴约束和默认值 | 2 | 中 |
| P03 | 为新传感器增加采样率、质量或超时参数 | 对应 frontend `var_info[]` | 公共库参数、backend 配置和 guard | 2 | 中 |
| P04 | 为估计观测增加噪声或 gate 参数 | 对应 estimator/frontend | 数学单位、范围、旧日志重放和安全默认值 | 3 | 高 |

### 参数案例必须检查

- 现有 `AP_GROUPINFO/GSCALAR` 索引绝不重排或复用。
- 参数全名、单位、范围、增量、用户级别和 reboot 要求。
- 默认值是否严格保持旧固件行为。
- 参数存储、升级、恢复默认和地面站显示。
- 极小、极大、NaN/非法配置怎样被限制。

P01 改动最小，适合用来建立参数开发的完整模板。

## 4. 修改控制算法

第一批控制算法案例优先修改目标整形或限幅，不直接重写姿态/位置 PID。

| 编号 | 候选需求 | 建议作用范围 | 关键观测量 | 难度 | 风险 |
|---|---|---|---|---:|---|
| [C01（推荐）](cases/04_control_algorithm/README.md) | 为 Precision Manual 增加 forward/lateral/升沉 slew-rate limiter | 只影响新模式 | raw input、shaped input、`dt`、motor request | 2 | 中 |
| C02 | 将 forward/lateral 方形输入限制为单位圆 | 先只影响新模式 | 输入向量长度、方向和 limiter flag | 2 | 中 |
| C03 | 改进 yaw 杆回中到 heading hold 的目标过渡 | Stabilize 或新 Precision 模式 | yaw input、target rate、target heading、actual yaw | 3 | 高 |
| C04 | 调整深度位置/速度/加速度目标整形 | AltHold/`AC_PosControl` | U 轴目标、速度、加速度、jerk、throttle limit | 4 | 极高 |
| C05 | 修改 6DOF 混控饱和时的轴优先级 | `AP_Motors6DOF` | 六轴请求、motor saturation、limit flags | 5 | 极高 |

### 控制算法案例必须提供

1. 输入、输出、坐标系、单位和调用频率。
2. 公式、状态变量、初始化和 reset 条件。
3. 限幅、饱和、integrator 和异常输入行为。
4. 默认配置与旧算法的差异。
5. 日志中的目标、估计和最终输出。
6. SITL 阶跃、正反切换、超时和饱和场景。
7. Pixhawk4 build、拆桨台架和必要的系留水池结果。

C01 可接在 M01 后实施，因为它不会改变现有模式，影响范围最容易证明。

## 5. 修改估计算法

估计算法比普通 sensor driver 风险更高。观测“能读到”不等于估计器“应该融合”；进入 EKF 前必须定义时间、坐标、噪声、质量、reset 和失效语义。

| 编号 | 候选需求 | 设计级别 | 推荐验证方式 | 难度 | 风险 |
|---|---|---|---|---:|---|
| [E01（推荐入门）](cases/05_estimation_algorithm/README.md) | 增加一个只记录、不参与控制的压力深度/垂直速度 shadow estimator | 车辆/实验旁路 | 同一日志对比现有估计、静态噪声和升沉响应 | 3 | 低 |
| E02 | 为新传感器观测增加离群值 gate 和健康状态机 | sensor frontend/输入预处理 | 录包重放、异常值、超时和恢复 | 3 | 中 |
| E03 | 修改压力深度零偏或 surface 条件下的 bias 补偿 | frontend/AHRS 输入路径 | 长时间静止、上浮/下潜、温漂和 reset | 4 | 高 |
| E04 | 根据观测质量动态调整 measurement noise/gate | EKF 观测接口或 core | innovation、variance、accept/reject、故障注入 | 5 | 极高 |
| E05 | 向 EKF3 增加新的观测融合或状态 | `AP_NavEKF3` frontend/core | 仿真、日志重放、多源冲突、reset 和全模式回归 | 5 | 极高 |

### 估计修改的分级路线

```text
先做 log-only shadow estimator
    -> 证明数据、单位、噪声和时序
    -> 做录包/故障重放
    -> 再考虑输入 gate 或 bias
    -> 最后才修改 EKF fusion/core
```

E01 的输出不得接入控制器，只用于建立对照证据。E04/E05 不能作为简单教学提交，必须单独做数学审查、日志回放、SITL、目标板和水池验证。

## 6. 新开发板

新板卡限定为 Pixhawk 类 STM32/ChibiOS 实时飞控，不把 Linux SBC 扩展为飞控产品。

| 编号 | 候选需求 | 主要改动 | 前置资料 | 难度 | 风险 |
|---|---|---|---|---:|---|
| B01（入门） | 在现有兼容板 hwdef 中增加一个已验证 UART/I2C 外设实例 | `hwdef.dat` 和 defaults | 原理图、pin、总线、电平和设备方向 | 3 | 中 |
| [B02（完整案例）](cases/06_new_board/README.md) | 为同 MCU、参考设计兼容的新板建立 board definition | `hwdef.dat`、`hwdef-bl.dat`、README | 原理图、MCU/晶振、board ID、flash、传感器和输出表 | 5 | 极高 |
| B03 | 修改 PWM/DSHOT 输出 timer/DMA 分配 | hwdef 与 ChibiOS 输出资源 | timer/channel/DMA/AF、电气接口和示波器 | 5 | 极高 |
| B04 | 现有 HAL 无法表达硬件时增加可复用 ChibiOS HAL 能力 | `AP_HAL/AP_HAL_ChibiOS` | 多板复用证据和接口设计 | 5 | 极高 |

### 板卡案例固定顺序

```text
原理图与参考板差异
 -> hwdef-bl
 -> bootloader build/SWD
 -> hwdef
 -> ArduSub board build
 -> console/clock/storage/USB
 -> bus/sensor
 -> safety/arming/failsafe
 -> 示波器输出
 -> 拆桨
 -> 系留水池
```

控制层原则上不因新板卡修改；如果需求要求在 `mode_*.cpp` 中判断板卡名，通常说明职责放错了。

## 7. 六类案例共同的验证要求

测试不单独立项，但每个案例都必须从下表选择相应层级：

| 开发类型 | 必需软件验证 | 必需硬件验证 | 必需回退 |
|---|---|---|---|
| 参数 | 默认值/边界、参数存储、相关 SITL 行为、Pixhawk4 build | 涉及控制时拆桨检查 | 参数备份和旧默认值 |
| 模式 | 进入/拒绝/退出/失效、SITL、模式报告、Pixhawk4 build | 拆桨，必要时系留水池 | 旧固件、旧模式配置 |
| 传感器 | parser/unit、超时/断线、frontend health、consumer integration、board build | 实物总线、方向、异常和断线 | 禁用 backend/旧固件 |
| 控制算法 | 数学/边界、日志、SITL 阶跃/饱和、目标板 build | 拆桨和受控水池 | 可关闭 guard/参数、旧固件 |
| 估计算法 | 录包重放、shadow 对照、innovation/reset、SITL 全模式回归 | 传感器实物、长时间静态和水池 | 恢复旧 estimator 参数/固件 |
| 开发板 | bootloader、board build、flash/RAM、参数和 feature | SWD、总线、传感器、safety、示波器和拆桨 | SWD 恢复、旧 bootloader/固件 |

## 8. 推荐的实际学习顺序

| 顺序 | 主线 | 推荐案例 | 原因 |
|---:|---|---|---|
| 1 | 参数 | P01 | 改动最小，先掌握参数兼容和验证记录 |
| 2 | 模式 | M01 | 串起车辆层模式生命周期和输出链 |
| 3 | 控制算法 | C01 | 只在新模式内加入小算法，影响范围明确 |
| 4 | 传感器 | S01 | 学习真实 backend，但不直接改变姿态/位置估计 |
| 5 | 估计算法 | E01 | 先做 shadow 对照，不接入控制 |
| 6 | 开发板 | B02 | 在软件方法成熟后进入硬件与恢复链 |

## 9. 选定案例前需要提供的信息

| 类型 | 必需输入 |
|---|---|
| 模式 | 一句话行为、最接近模式、输入、进入/退出/失效和人工接管 |
| 传感器 | 精确型号、datasheet、协议、电气连接、真实录包和故障语义 |
| 参数 | 当前固定行为、范围、单位、默认值和兼容要求 |
| 控制算法 | 公式、输入输出、坐标、单位、频率、约束和通过标准 |
| 估计算法 | 状态/观测模型、噪声假设、时间/坐标、数据集和对照基线 |
| 开发板 | 原理图、BOM、MCU/晶振、pin/bus/timer/DMA 表、参考板和恢复手段 |

六个推荐需求已经各有独立教程。下一步若选择其中某项进入实现，必须先补齐该 README 列出的真实需求、硬件资料和通过标准，再建立独立开发分支；没有可验证输入的候选不进入固件实现。
