# ArduSub 4.7.0 二次开发学习索引

本目录服务于本仓库的唯一产品目标：在官方 `Sub-4.7.0` 基础上维护面向 Pixhawk 类 ChibiOS 飞控板的 ArduSub 固件。它记录的是当前仓库已经核验过的源码入口、开发方法和验证边界，不是对最新上游 `master` 的通用说明。

## 使用原则

1. 当前分支源码是行为事实的最终依据；文档与源码不一致时，先核验源码，再修正文档。
2. 阅读控制行为时沿真实数据流追踪，不按目录逐文件漫游。
3. 本仓库的产品范围虽然聚焦 ArduSub，但 `libraries/` 中的共享代码仍可能是 Waf 的直接或传递依赖，不能凭名称删除。
4. `modules/` 是 Git submodule，不在本仓库中直接修改。
5. 任何涉及解锁、failsafe、推进器状态或硬件输出的改动，都按安全关键软件处理。
6. 实机推进器测试必须先拆桨或物理隔离推进器，并准备独立断电手段。

## 推荐阅读顺序

| 顺序 | 文档 | 解决的问题 |
|---|---|---|
| 1 | [源码阅读地图](01_source_reading_map.md) | 程序从哪里启动，控制数据怎样走到输出 |
| 2 | [飞行模式与控制器](02_flight_modes_and_control.md) | Manual、Stabilize、AltHold、PosHold、Guided 分别做什么 |
| 3 | [推进器、混控与输出](03_motors_mixing_and_output.md) | spool 请求、6DOF 混控、SRV/HAL 输出怎样衔接 |
| 4 | [轨迹与目标整形](04_trajectory_and_guided.md) | 位置、速度、加速度和 S 曲线/jerk 限制在哪里生效 |
| 5 | [WSL 构建、测试与发布](05_build_test_and_release.md) | 怎样构建 SITL/Pixhawk4，怎样记录真实验证结果 |
| 6 | [安全变更工作流](06_safe_change_workflow.md) | 二次开发从问题定义到审查、验证、回退的完整流程 |
| 7 | [Pixhawk 类板卡移植](07_pixhawk_board_porting.md) | 自绘 STM32/ChibiOS 飞控板怎样通过 hwdef 接入 |

## 一条主阅读链

```text
RC / MAVLink 输入与传感器状态
    -> ArduSub 飞行模式策略
    -> AC_AttitudeControl / AC_PosControl / AC_WPNav
    -> AP_MotorsMulticopter 状态机
    -> AP_Motors6DOF 混控
    -> SRV_Channels
    -> AP_HAL / ChibiOS 硬件输出
```

不要把某一个 `mode_*.cpp` 文件理解成完整控制系统。模式层主要选择目标和控制策略；状态估计、目标整形、闭环控制、推进器状态许可、混控和硬件输出分布在不同层。

## 坐标系与单位提醒

ArduSub 同时使用机体系和地固坐标系。看到变量或 API 时至少确认以下四项：

- 坐标系：body、North-East、NED、NEU 或 Up/Down 轴接口。
- 目标类型：position、velocity、acceleration、angle、angular rate 或归一化推力。
- 单位：当前源码中常见 `cm`、`cm/s`、`cm/s²`、`m`、`rad`、`centi-degree`。
- 正方向：特别核对垂直轴的 Up/Down 语义和推进器安装方向。

变量名中的 `_cm`、`_cms`、`_cd`、`_rad` 是重要证据，不能在修改时省略或凭经验换算。

## 当前基线不是“删库版”

本仓库已经移除了与产品目标无关的车辆入口和外部集成工具，但仍保留 ArduSub 构建需要的共享库与子模块。忽略目录中的 `build/` 或其他 Waf 输出只是产物，不是另一份“精简源码”。若未来要进一步裁剪依赖，必须单独立项、建立独立分支，先得到 Waf dependency closure、固件体积和测试证据，再逐步实施。
