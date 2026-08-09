# ArduSub 4.7.0 二次开发学习资料

本目录采用“一个系统总览 + 多个需求案例”的结构。

- “总”部分只讲 ArduSub 4.7.0 的稳定架构、源码职责、控制链、构建测试和安全边界。
- “案例”部分从一个小功能需求出发，完整走过源码证据、设计、实现、测试和回退。
- 不再为单一传感器或尚未确定的产品路线建立大而全的专题。

## 文档结构

| 类型 | 文档 | 内容 |
|---|---|---|
| 系统总览 | [ArduSub 系统架构、源码与工程总览](01_source_reading_map.md) | 合并原 00–08：生命周期、调度、逐文件职责、模式、控制器、混控、轨迹、共享库、Waf、测试、安全和板卡移植 |
| 案例规划 | [六类二次开发教学案例](10_case_catalog.md) | 六类需求的候选范围、难度、风险和推荐顺序 |
| 案例 01 | [新增 Precision Manual 飞行模式](cases/01_new_flight_mode/README.md) | 模式身份、注册、六轴限幅、安全状态和模式验证 |
| 案例 02 | [新增 MCP9808 温度传感器 backend](cases/02_new_sensor_driver/README.md) | datasheet、frontend/backend、I²C、健康度、日志和断线测试 |
| 案例 03 | [新增航向保持过渡时间参数](cases/03_new_parameter/README.md) | 参数 identity、元数据、默认兼容和三模式一致性 |
| 案例 04 | [为 Precision Manual 增加输入斜率限制](cases/04_control_algorithm/README.md) | 目标整形公式、dt、状态 reset、日志和阶跃测试 |
| 案例 05 | [压力深度垂速 Shadow Estimator](cases/05_estimation_algorithm/README.md) | log-only 旁路估计、单位/坐标、滤波、重放和准入边界 |
| 案例 06 | [迁移 Pixhawk 类 STM32F765/ChibiOS 飞控板](cases/06_new_board/README.md) | 原理图差异审计、hwdef、bootloader、bring-up 和拆桨验证 |
| 已成型案例 | [开发一种新推进器构型](11_new_thruster_frame_development.md) | 从物理布局、6DOF 系数和 `FRAME_CONFIG` 到拆桨台架与系留水池验证 |

## 推荐使用方法

1. 先读总览第 1–5 节，画出输入、估计、模式、控制器、混控和输出数据流。
2. 再读总览第 6–9 节，掌握 Manual → Stabilize → AltHold → PosHold → Guided 的递进关系。
3. 阅读第 10–13 节，理解 WSL 构建、验证矩阵、变更安全规则和 Pixhawk 类板卡边界。
4. 按案例 03 → 01 → 04 → 02 → 05 → 06 阅读六篇需求教程；这是从低风险软件接口逐步走向硬件 bring-up 的顺序。
5. 每篇中的代码均为设计伪代码，不代表功能已进入固件；真正实施时仍需单独分支、源码复核和逐级验证。
6. 为每个实现拆分可独立审查的提交，不把多个功能混进同一个教学提交。

## 每个案例统一回答的问题

| 阶段 | 必须回答 |
|---|---|
| 需求 | 用户能观察到什么变化？旧行为和默认值是否保持？ |
| 定位 | 输入从哪里来，经过哪些文件和共享库？ |
| 设计 | 应修改模式、frontend/backend、控制器、参数还是 hwdef？ |
| 安全 | 坐标、单位、健康、arming、failsafe 和回退是什么？ |
| 验证 | unit、SITL、autotest、Pixhawk4、拆桨台架分别证明什么？ |
| 交付 | 修改文件、真实结果、未测风险和恢复步骤是什么？ |

## 仓库边界

- 固定在官方 `Sub-4.7.0` 派生基线，不自动升级上游 `master`。
- 只维护 ArduSub 与 Pixhawk 类 ChibiOS 飞控。
- `libraries/` 是共享构建依赖，不代表扩展其他车辆产品。
- 不直接修改 `modules/`。
- 不把伴随计算机应用、ROS/MAVROS 工作区放入固件仓库。
- 不改变已有参数索引，不绕过 feature guard、arming、interlock、failsafe、safety 或 spool 状态机。
- 推进器相关实机测试必须拆桨或物理隔离，并准备独立断电。

`build/` 等目录是 Waf 产物，不是精简源码。任何删库、依赖裁剪或稳定版升级都必须单独立项、独立分支并逐级验证。
