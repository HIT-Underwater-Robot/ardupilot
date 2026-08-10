# Sub_learning：从最短控制链掌握 ArduSub

本目录位于极简实验分支，目标是让初学者先读懂一条能在 Pixhawk1 上编译的最短真实链路，再回到完整 `master` 做独立功能实验。

## 先理解本分支是什么

```text
RC / MAVLink Manual Control
        ↓
MANUAL 或 STABILIZE
        ↓
姿态/角速度控制（仅 Stabilize）
        ↓
AP_Motors spool 状态机与 AP_Motors6DOF
        ↓
SRV_Channels → ChibiOS HAL → IOMCU/PWM
```

本分支删除了位置保持、任务、航点、轨迹、测距、地形、视觉、CAN、Lua 和绝大多数非 Pixhawk1 驱动。`cases/` 是需求设计练习；`labs/` 另行提供可手工接入的教学参考源码。两者都不属于默认固件，更不是可直接发布的产品实现。

## 推荐学习顺序

| 阶段 | 阅读内容 | 完成标准 |
|---|---|---|
| 1 | [极简系统架构与源码地图](01_source_reading_map.md) | 能从调度器追到 Manual/Stabilize、姿态控制、6DOF 混控和 PWM |
| 2 | `ArduSub/mode_manual.cpp` | 能解释六个驾驶通道怎样进入 motors，以及 armed/spool 约束 |
| 3 | `ArduSub/mode_stabilize.cpp`、`Attitude.cpp` | 能区分姿态目标、角速度目标、陀螺反馈和控制输出 |
| 4 | `libraries/AC_AttitudeControl`、`AC_PID` | 能指出外环和 rate PID 的职责，不把 PID 输出直接等同于 PWM |
| 5 | `libraries/AP_Motors`、`SRV_Channel`、`AP_HAL_ChibiOS` | 能解释 spool、6DOF 系数、servo function、IOMCU/PWM 的边界 |
| 6 | [动手实验总入口](labs/README.md) | 能手工接入 UART、Waf 库、新模式和安全超时，并完成 Pixhawk1 构建 |
| 7 | [六类二次开发案例目录](10_case_catalog.md) | 能为新需求选择模式层、驱动层、控制层、估计层或 hwdef 层 |

## 文档目录

| 文档 | 用途 |
|---|---|
| [01_source_reading_map.md](01_source_reading_map.md) | 与当前极简源码一致的架构、文件职责、库白名单和验证边界 |
| [labs/README.md](labs/README.md) | 默认不参与编译的四级实操课：UART Ping、串口帧库、Precision Manual、串口模式 |
| [10_case_catalog.md](10_case_catalog.md) | 新模式、传感器、参数、控制算法、估计算法、新板卡的立项表 |
| [cases/01_new_flight_mode](cases/01_new_flight_mode/README.md) | 新飞行模式设计练习 |
| [cases/02_new_sensor_driver](cases/02_new_sensor_driver/README.md) | 新传感器 frontend/backend 设计练习 |
| [cases/03_new_parameter](cases/03_new_parameter/README.md) | 参数 identity、索引、默认兼容和迁移练习 |
| [cases/04_control_algorithm](cases/04_control_algorithm/README.md) | 控制目标整形/控制器修改练习 |
| [cases/05_estimation_algorithm](cases/05_estimation_algorithm/README.md) | 旁路估计、观测、健康与准入练习 |
| [cases/06_new_board](cases/06_new_board/README.md) | STM32/ChibiOS hwdef 与拆桨 bring-up 练习 |
| [11_new_thruster_frame_development.md](11_new_thruster_frame_development.md) | 新推进器构型与 6DOF 系数的完整审计方法 |

旧的虚拟 DVL、串口转发和 SMC“直接可用 demo”已经删除。新的 `labs/` 只保留与当前极简 Pixhawk1 源码逐项核对过的教学参考，并要求学习者自己复制、注册和验证；默认构建仍然只有 Manual/Stabilize。

## 实验纪律

1. 在 `master` 上只做完整基线维护，不直接试验。
2. 每个新功能单独建实验分支，只改变一个架构边界。
3. 先写输入、坐标、单位、健康、arming/failsafe、输出和回退，再写代码。
4. 构建成功只证明编译链接；控制改动还需要日志、拆桨台架和受控实机验证。
5. 不修改 `modules/`，不重排已有参数索引，不绕过 feature guard、safety、interlock 或 spool 状态机。
6. `Sub_learning/labs/**/files` 中出现 `.cpp/.h` 不代表它已进入固件；以目标路径、Waf 白名单、对象成员、初始化和 scheduler/mode 注册为证据。

## 当前验证基线

当前唯一支持的构建命令是：

```bash
./waf configure --board Pixhawk1 --out build_minimal_pixhawk1 --no-submodule-update
./waf sub -j4
```

该结果不能替代 Pixhawk1 烧录和拆桨硬件验证。
