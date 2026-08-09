# ArduSub 4.7.0 for Pixhawk-class flight controllers

这是从官方 [ArduPilot](https://github.com/ArduPilot/ardupilot) 稳定标签 `Sub-4.7.0` 迁移出的 ArduSub 固件仓库。

本仓库保留完整的 `ArduSub/`、`libraries/` 和 `modules/`，不修改官方 4.7.0 飞控逻辑，不关闭 ArduSub 的编译功能。其他车辆源码、通用 CI、Docker、Vagrant、文档站点及与 ArduSub 固件构建无关的外部工具已移除。

## 维护边界

- 唯一车辆产品是 ArduSub，不引入 ArduPlane、ArduCopter、Rover 等其他车辆主线。
- 目标硬件限定为 Pixhawk4、Pixhawk1 及同类 ChibiOS 实时飞控板；新板卡支持通过 HAL 和 `hwdef.dat` 完成。
- SITL 仅用于 ArduSub 固件回归验证，不扩展为通用机器人仿真平台。
- 固件必需的 MAVLink 协议、地面站通信和生成链继续保留。
- 伴随计算机应用、中间件工作区、通用机器人集成和非 ArduSub 控制工具不属于本仓库维护范围。
- 不自动跟随上游 `master`；任何升级都必须在独立分支审计 ArduSub 稳定版差异，并完成 SITL、目标板构建和回退验证。

## 获取源码

必须同时初始化 Git 子模块：

```bash
git clone --recurse-submodules https://github.com/HIT-Underwater-Robot/ardupilot.git
cd ardupilot
```

如果已经克隆但缺少子模块：

```bash
git submodule update --init --recursive
```

## Ubuntu / WSL 编译环境

首次使用时安装 ArduPilot 官方依赖：

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

不要使用 `sudo ./waf`。

## 编译 Pixhawk4

当前实际使用的飞控是 Pixhawk4；它在 Waf 中的板卡名称是 `Pixhawk4`：

```bash
./waf configure --board Pixhawk4
./waf sub
```

固件输出位置：

```text
build/Pixhawk4/bin/ardusub.apj
```

如果以后需要兼容 Pixhawk 2.4.8，它对应的 Waf 板卡名称是 `Pixhawk1`。查询其他受支持板卡：

```bash
./waf list_boards
```

## 编译 SITL

```bash
./waf configure --board sitl
./waf sub
```

SITL 可执行文件输出位置：

```text
build/sitl/bin/ardusub
```

## 保留原则

- `ArduSub/`：完整保留官方车辆层源码。
- `libraries/`：完整保留官方公共库，避免破坏隐式依赖。
- `modules/`：完整保留官方 Git 子模块，不在本仓库内修改子模块源码。
- `Tools/`：只保留 ArduSub 所需的 Waf、固件生成、Bootloader、SITL、环境安装和调试工具。

这是面向 Pixhawk 类飞控板的定向固件 Fork。同步官方 ArduSub 稳定版更新时，必须重新执行 Pixhawk4 和 SITL 的完整编译验证。

## 二次开发资料

仓库内的 [`Sub_learning/`](Sub_learning/README.md) 采用“一份系统总览 + 多个需求案例”的教学结构。总览合并了系统架构、逐文件职责、Manual 到 Guided 的控制主线、6DOF 输出、共享库、构建测试和板卡移植；案例部分只围绕新模式、新传感器、新参数、控制算法、估计算法和新开发板六类真实需求展开。
