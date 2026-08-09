# ArduSub 4.7.0 初学者二次开发课程

本目录不是一组需要从头背到尾的 API 说明，而是一条“总—分—实践”的学习路线。目标是让初学者先看懂整个 ArduSub 怎样运行，再能沿真实数据流定位问题，最后独立完成一个小模式或一种推进器构型的设计、实现和验证。

本课程只面向本仓库的产品边界：官方 `Sub-4.7.0` 基线、ArduSub 固件、Pixhawk 类 ChibiOS 飞控。其他 ArduPilot 车辆、伴随计算机应用和 ROS/MAVROS 工程不在这里扩展。

## 先看总图，再进入细节

```text
驾驶员 / MAVLink                         IMU / 罗盘 / 压力 / ExternalNav
        |                                           |
        +-------------- 输入与状态估计 -------------+
                            |
                            v
                     当前飞行模式策略
                            |
            +---------------+----------------+
            |                                |
            v                                v
      姿态/角速度目标                 位置/速度/加速度目标
            |                                |
            +------ AC_AttitudeControl ------+
                            |
                  roll / pitch / yaw
                            |
                  throttle / forward / lateral
                            |
             AP_Motors 状态机与 6DOF 混控
                            |
                  SRV_Channels / HAL
                            |
                         推进器
```

模式不是完整控制系统。模式只决定“现在要控制什么、允许什么、失效后去哪里”；状态估计、目标整形、闭环控制、推进器许可、混控和硬件输出分别属于不同层。

## 必修主线

### 第一部分：总——建立完整心智模型

| 顺序 | 文档 | 学完应能回答 |
|---|---|---|
| 1 | [从零掌握 ArduSub](00_beginner_learning_path.md) | 系统有哪些层，fast loop 为什么是跨周期流水线，应该先学什么 |
| 2 | [源码阅读地图](01_source_reading_map.md) | 程序从哪里启动，一次控制请求怎样走到推进器 |

这一部分不要陷入 PID 公式。先能在源码中指出生命周期入口、调度顺序、模式分派和最终输出对象。

### 第二部分：分——逐层拆开一条控制链

| 顺序 | 文档 | 学习主线 |
|---|---|---|
| 3 | [飞行模式与控制器](02_flight_modes_and_control.md) | Manual → Stabilize → AltHold → PosHold → Guided，每一步只增加一层能力 |
| 4 | [推进器、混控与输出](03_motors_mixing_and_output.md) | desired spool state → armed/interlock → 6DOF 分配 → PWM |
| 5 | [轨迹与目标整形](04_trajectory_and_guided.md) | position/velocity/acceleration 目标怎样被限速、限加速度和 jerk 整形 |

阅读每个模式时都用同一组问题：

1. 输入是什么，坐标系和单位是什么？
2. 模式生成的是姿态、角速度、位置、速度，还是归一化推力？
3. 依赖什么估计状态，健康条件在哪里检查？
4. 哪个控制器消费目标？
5. 输出怎样进入 `AP_Motors6DOF`？
6. 未解锁、数据失效和 failsafe 时会怎样？

### 第三部分：实践——完成两个端到端开发练习

| 顺序 | 文档 | 实践成果 |
|---|---|---|
| 6 | [开发一个新飞行模式](10_new_mode_development.md) | 以 DVL 辅助定点为例，走完需求、复用、注册、安全降级和测试 |
| 7 | [开发一种新推进器构型](11_new_thruster_frame_development.md) | 从物理布局和 6DOF 系数表走到 `FRAME_CONFIG`、输出核验和水池回归 |
| 8 | [DVL 功能案例附录](09_dvl_feature_case_study.md) | 深入理解 ExternalNav、EKF、数据健康和专用模式边界 |

这里的“新机型”特指 ArduSub 内的新 ROV 推进器布局。若需求是自绘 Pixhawk 类主控板，应走[板卡移植](07_pixhawk_board_porting.md)；若需求是增加另一种 ArduPilot 车辆，则超出本仓库范围。

### 第四部分：工程化——让改动可以维护和发布

| 顺序 | 文档 | 解决的问题 |
|---|---|---|
| 9 | [公共库管理](08_shared_library_management.md) | 何时复用、扩展或新增 `libraries/` 组件 |
| 10 | [安全变更工作流](06_safe_change_workflow.md) | 怎样把需求拆成小变更并保留审查、测试和回退证据 |
| 11 | [WSL 构建、测试与发布](05_build_test_and_release.md) | 怎样构建 SITL/Pixhawk4 并如实记录结果 |
| 12 | [Pixhawk 类板卡移植](07_pixhawk_board_porting.md) | 怎样通过 `hwdef.dat` 与 ChibiOS HAL 适配自绘 STM32 飞控 |

## 建议的学习节奏

不要用“看完多少文件”衡量进度，用以下四个里程碑：

- 里程碑 A：能不看文档画出输入、估计、模式、控制器、混控、输出六层图。
- 里程碑 B：能从 `scheduler_tasks` 解释为什么 rate controller 与 mode 在同一循环中的先后顺序不是一条同步函数调用栈。
- 里程碑 C：能独立追踪 Manual、Stabilize、AltHold、PosHold 各自新增的闭环，并标出坐标系、单位和失效条件。
- 里程碑 D：能为一个新模式或推进器构型写出文件清单、测试矩阵、实机安全步骤和回退方案，然后才开始编码。

## 学习与开发纪律

1. 当前源码是行为事实；文档与源码不一致时，先核验源码。
2. 不按目录逐文件漫游，只沿一个可观察行为的真实数据流阅读。
3. `libraries/` 是共享依赖，不能凭名称或产品聚焦范围删除。
4. `modules/` 是 Git submodule，不在本仓库中直接修改。
5. 不改变已有 `AP_GROUPINFO` 索引和模式号；新增编号也必须先做全局兼容审计。
6. 不绕过 feature guard、arming、interlock、failsafe、safety switch 或 spool 状态机。
7. 推进器实机测试必须先拆桨或物理隔离，并准备独立断电手段。

## 当前基线不是“删库版”

本仓库保留 ArduSub 构建所需的共享库和子模块。`build/` 等忽略目录只是 Waf 产物，不是另一份精简源码。任何进一步裁剪必须单独立项、建立独立分支，取得依赖闭包、固件体积、SITL、目标板构建和回退证据后再逐步实施。
