# ArduSub-focused ArduPilot

这是基于官方 [ArduPilot](https://github.com/ArduPilot/ardupilot) `master` 分支整理的 ArduSub 专用仓库。

本仓库保留完整的 `ArduSub/`、`libraries/` 和 `modules/`，不修改飞控逻辑，不关闭 ArduSub 的编译功能。其他车辆源码、通用 CI、Docker、Vagrant、文档站点及与 ArduSub 构建无关的外部工具已移除，以便集中学习和开发水下机器人控制系统。

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

## 编译 Pixhawk 2.4.8

Pixhawk 2.4.8 在 Waf 中使用板卡名称 `Pixhawk1`：

```bash
./waf configure --board Pixhawk1
./waf sub
```

固件输出位置：

```text
build/Pixhawk1/bin/ardusub.apj
```

查询其他受支持板卡：

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
- `Tools/`：保留 Waf、固件生成、Bootloader、SITL、环境安装、调试、ROS 2 和 Simulink 相关工具。

这是科研与二次开发用的定向 Fork。同步官方更新时，应重新执行 Pixhawk1 和 SITL 的完整编译验证。
