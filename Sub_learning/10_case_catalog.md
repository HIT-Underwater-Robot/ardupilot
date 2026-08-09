# ArduSub 二次开发教学案例候选表

本篇只列候选，不代表这些功能已经实现或确定要进入 `master`。选中案例后，再为它单独建立需求、源码证据、分支、实现文档和测试记录。

案例遵循同一教学模板：

```text
用户需求
 -> 当前行为与源码证据
 -> 最小设计
 -> 修改文件
 -> 参数/guard/兼容性
 -> SITL 或 unit test
 -> Pixhawk4 build
 -> 必要的拆桨/硬件验证
 -> 回退
```

难度使用 1–5 级；安全风险表示错误实现对真实载具的潜在影响，不等同于开发难度。

## 1. 新增参数类案例

| 编号 | 功能需求 | 主要源码入口 | 能学到什么 | 难度 | 安全风险 |
|---|---|---|---|---:|---|
| P01（推荐） | 把 Stabilize 松开 yaw 杆后当前固定约 250 ms 的减速/转保持时间改为可配置参数，默认保持现行为 | `Parameters.h/.cpp`、`mode_stabilize.cpp` | 参数索引、元数据、单位、默认兼容、模式读取和时序测试 | 1 | 中 |
| P02 | 为一个新“精细操控模式”增加最大水平/升沉/转向比例参数，不改变现有模式默认行为 | `Parameters.*`、新模式文件 | 参数分组、范围、默认值和多参数联动 | 2 | 中 |
| P03 | 把某个仅用于诊断的状态记录周期改为参数，并做上下限保护 | `Parameters.*`、`Log.cpp` 或调度消费点 | 非控制参数、周期单位、CPU/日志容量约束 | 1 | 低 |
| P04 | 给新传感器 backend 增加采样率或质量门槛参数 | 对应 `AP_*` frontend `var_info[]` | 公共库参数、backend 专用配置、feature disabled 构建 | 2 | 中 |

P01 最适合作为第一个参数案例，因为当前固定时序可以在源码中定位，改动小，也能完整讲清“参数不是加一个成员变量”。

## 2. 新飞行模式类案例

| 编号 | 功能需求 | 设计方向 | 主要改动面 | 难度 | 安全风险 |
|---|---|---|---|---:|---|
| M01（推荐） | 增加 Precision Manual：保留 Manual 的直接操控语义，但限制六轴最大输出，便于水池近距离低速操作 | 从 Manual 行为出发，只增加输入比例/限幅；不新增 PID | `mode.h`、新 `mode_*.cpp`、`Sub.h`、`mode.cpp`、参数、GCS mode reporting、autotest | 3 | 高 |
| M02 | 增加 Precision Stabilize：保留姿态稳定，限制倾角、yaw rate、水平和升沉请求 | 复用 Stabilize/AttitudeControl，只修改模式目标约束 | 模式注册、参数、模式实现、GCS、SITL | 3 | 高 |
| M03 | 增加只允许姿态和深度保持、禁止水平自动目标的作业模式 | 基于 AltHold 明确屏蔽/限制水平请求，并定义驾驶员接管 | 模式生命周期、AltHold 复用、failsafe、状态报告 | 3 | 高 |
| M04 | 增加短时控制器验证模式，只在拆桨/SITL 中产生受限测试目标 | 必须有编译 guard、严格 arming 条件和自动退出 | 模式、arming、guard、测试、状态文本 | 4 | 极高 |

M01 适合教学模式号、静态对象、`set_mode()`、`init()/run()`、GCS 和测试，但真实载具风险仍高，不能因为算法简单就跳过拆桨验证。

## 3. 新传感器驱动类案例

传感器案例必须先选定具体器件并提供官方 datasheet、总线电气信息和真实数据，不能编造“通用协议”。

| 编号 | 功能需求 | 建议实现边界 | 主要测试 | 难度 | 安全风险 |
|---|---|---|---|---:|---|
| S01（推荐） | 为一个明确型号的 I2C 水温/舱内温度传感器增加 backend | 复用 `AP_TemperatureSensor` frontend；backend 只负责探测、读取、换算和健康 | 寄存器/字节序 unit test、断线/异常值、SITL stub、Pixhawk4 build | 3 | 低 |
| S02 | 为一个明确协议的 UART ASCII 环境传感器增加 parser/backend | parser 与串口调度分离，不阻塞主循环 | 有效帧、截断、乱码、超时、频率和重连 | 3 | 中 |
| S03 | 为一个明确型号的 downward rangefinder 增加 backend | 复用 `AP_RangeFinder`，保持距离/质量/方向语义 | parser、质量、超时、方向、SurfTrak consumer integration | 4 | 高 |
| S04 | 为外接 GPIO/I/O expander leak sensor 增加 backend | 复用 `AP_LeakDetector`，不在 `failsafe.cpp` 直接读硬件 | wet/dry、反相、debounce、断线、leak failsafe | 3 | 高 |
| S05 | 先写一个 SITL 模拟 backend，能注入正常值、超时和异常值 | 不接实物，先证明 frontend/backend 与 consumer 契约 | unit/SITL 故障注入 | 2 | 低 |

最顺畅的传感器教学组合是 `S05 -> S01`：先学习抽象和健康状态，再接入一颗协议明确、不会直接控制推进器的温度传感器。

## 4. 新控制算法类案例

第一批算法案例只处理目标整形和限幅，不重新发明姿态/位置 PID。

| 编号 | 功能需求 | 设计边界 | 观测量 | 难度 | 安全风险 |
|---|---|---|---|---:|---|
| A01（推荐） | 在 Precision Manual 内给 forward/lateral/升沉指令增加可配置 slew-rate limiter | 只影响新模式；使用调度 `dt`；默认值和 reset 行为明确 | raw input、shaped input、motor request、切换模式时状态 | 2 | 中 |
| A02 | 将 forward/lateral 方形输入限制为单位圆，避免对角输入幅值大于单轴 | 只做二维向量归一化，不改变坐标系 | 输入向量长度、方向、限幅标志 | 2 | 中 |
| A03 | 改进 yaw 杆回中到 heading hold 的过渡整形 | 不改 rate PID，只改变目标角速度到航向保持的切换 | yaw input、target rate、target heading、actual yaw | 3 | 高 |
| A04 | 为新传感器 backend 增加中值滤波与超时状态机 | 滤波属于设备观测，不把无效值伪装成健康值 | raw/filtered、age、quality、health | 2 | 中 |
| A05 | 研究 6DOF 混控饱和时的轴优先级 | 需要独立数学审查和大量回归，不作为第一批实现 | 各轴请求、motor saturation、limit flags | 5 | 极高 |

A01 可以接在 M01 后面：先完成模式生命周期，再在不影响已有模式的范围内加入一个可观察的小算法。

## 5. 日志、状态与可观测性案例

| 编号 | 功能需求 | 主要源码入口 | 教学价值 | 难度 | 安全风险 |
|---|---|---|---|---:|---|
| L01（推荐） | 给 Precision 模式增加 raw input、scaled input 和 limiter 状态日志 | `Log.cpp`、模式文件 | 日志结构、单位、频率、验证证据 | 2 | 低 |
| L02 | 模式进入失败时发送一次明确原因，避免每循环刷屏 | `mode.cpp`/具体 `init()`、GCS text | 一次性事件、错误路径和操作员可见性 | 1 | 低 |
| L03 | 为新传感器记录 raw、converted、age 和 health | 传感器 frontend/backend、`AP_Logger` | parser 与 consumer 问题分层诊断 | 2 | 低 |
| L04 | 在 GCS sensor status 中准确报告新控制器或传感器状态 | `GCS_Sub.cpp`、`GCS_MAVLink_Sub.cpp` | present/enabled/health 的区别 | 2 | 中 |

日志案例适合与参数、模式或传感器案例一起完成，不建议单独制造没有消费者的长期日志。

## 6. Failsafe 与状态机案例

| 编号 | 功能需求 | 设计重点 | 难度 | 安全风险 | 建议阶段 |
|---|---|---|---:|---|---|
| F01 | 新传感器超时后由 healthy 变为 unhealthy，并一次性告警 | 数据 age、状态边沿、恢复条件、日志 | 2 | 中 | 传感器案例第二步 |
| F02 | Precision 模式切出时清理 limiter/hold 状态，重新进入不继承旧目标 | `init()`、退出行为、reset | 2 | 高 | M01/A01 后 |
| F03 | 为一个非核心传感器增加 warn-only/degraded 行为 | 与现有 battery/leak/EKF/pilot failsafe 的优先级 | 4 | 高 | 后期 |
| F04 | 修改 leak、EKF、pilot input 或 spool 安全动作 | 全链路、实机和回退要求极高 | 5 | 极高 | 不作为入门案例 |

入门教学只建议 F01/F02。对现有核心 failsafe 动作的修改应作为独立安全项目，而不是“小案例”。

## 7. 自动测试与回归案例

| 编号 | 测试需求 | 主要位置 | 能证明什么 | 难度 |
|---|---|---|---|---:|
| T01（推荐） | 增加 Manual/Stabilize/AltHold/PosHold 的模式进入条件和切换回归 | `Tools/autotest/ardusub.py` | 模式生命周期和基础状态要求 | 2 |
| T02 | 为 P01 验证不同参数值下 yaw 过渡时间 | ArduSub autotest + 日志 | 参数确实改变目标时序且默认兼容 | 2 |
| T03 | 为 sensor parser 添加有效/截断/校验/超时用例 | 对应 `libraries/<lib>/tests/` | 驱动边界和错误处理 | 2 |
| T04 | 对新 frame 的六轴 factor 与 testing order 做单元检查 | `AP_Motors` tests 或针对性测试 | 软件混控表与设计矩阵一致 | 3 |
| T05 | 对 Precision mode 注入阶跃，检查 slew 和最大输出 | SITL/autotest | 新算法的限幅、reset 和模式隔离 | 3 |

测试不是最后补的章节。每个功能案例的设计文档都应先写出相应 T 类用例。

## 8. 推进器构型与板卡案例

| 编号 | 功能需求 | 当前资料 | 难度 | 安全风险 | 备注 |
|---|---|---|---:|---|---|
| H01 | 实现一个项目专用 `SUB_FRAME_CUSTOM` 六/八推进器布局 | [新推进器构型案例](11_new_thruster_frame_development.md) | 4 | 极高 | 已有完整教学文档，必须拆桨/系留验证 |
| H02 | 为 Pixhawk4 兼容自绘板增加 hwdef 和 bootloader | 总览第 12 节 | 5 | 极高 | 需要原理图、SWD、示波器和板卡实物 |
| H03 | 只新增一个经过验证的 UART/I2C 外设实例到现有 hwdef | 目标板 `hwdef.dat` | 3 | 中 | 不修改控制层 |

## 9. 推荐的第一组教学案例

| 顺序 | 案例 | 类型 | 为什么适合作为课程 |
|---:|---|---|---|
| 1 | T01：已有模式切换 autotest | 测试 | 先学会观察和证明现有行为，不改控制逻辑 |
| 2 | P01：Stabilize yaw hold delay 参数 | 参数 | 改动小，能完整学习参数 ABI、默认兼容和时序验证 |
| 3 | M01：Precision Manual | 新模式 | 串起模式号、对象、init/run、GCS、参数和输出链 |
| 4 | A01 + L01 + T05：新模式输入整形 | 控制算法/日志/测试 | 算法只作用于新模式，影响边界清楚且可观测 |
| 5 | S05 + S01 + L03 + T03 | 传感器 backend | 从模拟、parser、health 到真实 I2C 设备逐级学习 |
| 6 | H01 | 混控与硬件 | 在前面方法成熟后进入高风险推进器开发 |

## 10. 选择案例时需要提供的信息

| 案例类型 | 开始前需要的输入 |
|---|---|
| 参数 | 要改变的当前常量、期望范围/单位/默认值和旧行为兼容要求 |
| 模式 | 一句话用户行为、最接近现有模式、进入/退出/失效策略 |
| 控制算法 | 输入、输出、坐标、单位、频率、约束和可判定的通过标准 |
| 传感器 | 精确型号、datasheet、协议、电气连接、真实录包和故障语义 |
| Failsafe | 触发条件、动作优先级、恢复条件、操作员接管和物理回退 |
| Frame/板卡 | 机械/电气图、推进器或引脚表、测试设备和恢复手段 |

建议先从推荐顺序中选 1–3 个案例。选定后再写正式教程和代码，未选案例只保留在候选表中，不进入固件能力边界。
