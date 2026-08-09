# 四个 ArduSub 架构教学 demo

这组代码只存在于实验分支 `learning/four-architecture-demos`，不进入 `master`。它的目的不是增加产品能力，而是让初学者沿四条最常见的二次开发路径看到一份小而完整、可以编译和观察的实现。

## 先看边界

- 基线是官方 ArduSub 4.7.0 派生仓库，目标仍然只是 ArduSub 与 Pixhawk 类 ChibiOS 飞控。
- 所有实验代码由 `AP_SUB_LEARNING_DEMOS_ENABLED` 保护；实验分支默认打开。
- 虚拟 DVL 只完成“串口接收 → 校验 → 状态/健康度 → 日志 → 转发”，不写入 AHRS、EKF、位置控制器或 failsafe。
- SMC 只演示替换最低层角速度控制器的接口。它没有水动力学模型、等效控制项、鲁棒性证明或实机整定结论。
- 没有任何推进器实机测试结果。涉及 armed 或输出的实验必须拆桨或物理隔离推进器，并能独立断电。

## 四条学习路线

| Demo | 需求边界 | 入口 | 主要观察结果 |
|---|---|---|---|
| [01：Precision Manual 新模式](01_precision_manual/README.md) | 不改控制器、估计器和混控，只增加模式 | `Mode::Number`、`mode_from_mode_num()`、`Mode::run()` | 模式 22；六自由度手动输入按固定比例缩小 |
| [02：虚拟 DVL backend](02_virtual_dvl/README.md) | 不融合导航，只建立最小传感器数据通路 | `AP_SerialManager`、`DemoDVL::update()` | 正确帧、坏帧、速度、质量、超时健康度、`DDVL` 日志 |
| [03：DVL 参数、板载串口与转发](03_dvl_parameters_and_forwarding/README.md) | PC 通过 USB-TTL 给 Pixhawk 注入数据，Pixhawk 回包并通过 MAVLink 转发 | `AP_Param`、UART、GCS scheduler | `DVL_*` 参数、`DVLA` 回包、`DVL_VX` 等 Named Value Float |
| [04：SMC 角速度内环](04_smc_rate_controller/README.md) | 保留 Stabilize 外环、混控和安全链，只替换 rate controller | `Sub::run_rate_controller()`、`AC_AttitudeControl_Sub` | 模式 23；目标角速度与陀螺反馈进入边界层滑模示例 |

## 总体数据流

```text
Demo 01: RC/Joystick → Precision Manual → AP_Motors6DOF → SRV/HAL

Demo 02/03:
PC generator → USB-TTL → Pixhawk UART → DemoDVL parser/state
                                      ├→ DDVL onboard log
                                      ├→ DVLA serial status reply
                                      └→ GCS Named Value Float

Demo 04:
RC/Joystick → Stabilize attitude outer loop → desired body rate
           → SMC demo rate loop + gyro → AP_Motors6DOF → SRV/HAL
```

## 文件地图

| 文件 | 职责 |
|---|---|
| `ArduSub/mode_precision_manual.cpp` | Demo 01 的运行逻辑 |
| `ArduSub/mode.h`、`mode.cpp`、`Sub.h` | 模式身份、对象和运行时分派 |
| `ArduSub/demo_dvl.h/.cpp` | Demo 02/03 的参数、协议解析、健康度、日志和串口回包 |
| `ArduSub/sensors.cpp`、`system.cpp`、`Sub.cpp` | DVL 初始化、调度和 MAVLink 转发 |
| `libraries/AP_SerialManager/AP_SerialManager.*` | 注册实验串口协议 51 |
| `libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.*` | Demo 04 的参数和角速度控制律 |
| `Tools/scripts/build_options.py` | 实验 feature guard 的构建选项 |
| `tools/send_virtual_dvl.py` | PC/SITL 虚拟 DVL 发生器 |
| `sitl/virtual_dvl.parm` | SITL 串口与 DVL 参数样例 |

## 最小验证顺序

在 WSL 的实验 worktree 中执行：

```bash
./waf configure --board sitl --no-submodule-update
./waf sub
python3 Sub_learning/experimental_demos/tools/send_virtual_dvl.py --self-test
```

随后再做 SITL 串口集成、Pixhawk4 编译；最后才允许拆桨台架。构建成功只证明编译和链接，不证明控制算法安全，也不证明真实 DVL 协议兼容。

## 如何恢复纯净基线

不要把本分支合并到 `master`。需要结束实验时，直接切回主仓库的 `master` worktree；不要用 `git reset --hard`、`git clean` 或删除用户内容。若未来某个 demo 要产品化，应从 `master` 新建独立功能分支，重新定义协议、健康/failsafe、测试矩阵和回退方案，而不是直接搬运本实验实现。
