# ArduSub-focused ArduPilot

这是基于官方 [ArduPilot](https://github.com/ArduPilot/ardupilot) `master` 分支整理的 ArduSub 专用仓库。

本仓库的目标不是重新实现一套飞控，也不是立刻删除所有看起来暂时无用的驱动，而是保留一条完整、可编译、便于追踪的 ArduSub 主线，让学习者能够从构建入口逐步走到车辆逻辑、公共库和硬件抽象层。

> ArduSub 属于安全关键软件。修改控制器、解锁、失控保护、传感器融合或电机输出后，必须先进行代码审查、SITL 和台架测试，再考虑下水测试。

## 仓库边界

- `ArduSub/`：完整保留官方车辆层源码。
- `libraries/`：完整保留 ArduSub 所依赖的官方公共库，避免破坏隐式依赖和编译开关。
- `modules/`：完整保留 Git 子模块，包括 Waf、MAVLink、ChibiOS 等第三方工程。
- `Tools/`：保留构建、环境安装、Bootloader、SITL、调试、ROS 2 和 Simulink 所需工具。
- 其他车辆源码、Docker、Vagrant、通用 CI 和大型文档站点不在本仓库中。

[`BUILD.md`](BUILD.md) 是继承自上游的通用构建说明，其中可能出现本仓库没有保留的车辆或工具。对于本仓库，应优先以本 README 为准。

## 从这里开始系统学习

如果你目前只有 C++ 入门语法基础，不要直接从 `libraries/` 或 `Sub.cpp` 第一行开始硬读。先进入[中文学习中心](docs/learning/README.md)，按照课程顺序建立心智模型：

如果准备让新的智能体或其他 AI 工具继续维护本仓库，请先让它阅读根目录的 [`AGENTS.md`](AGENTS.md) 和 [AI 接手开发指南](docs/learning/AI_DEVELOPMENT_GUIDE.md)。后者记录了本项目的硬件范围、系统边界、讲解约定、开发流程和验收标准。

1. [阅读 ArduSub 必须掌握的 C++](docs/learning/01_cpp_foundations.md)：用真实源码理解继承、虚函数、指针引用、构造顺序、单例、宏和回调；
2. [从源码到 Pixhawk 固件的编译链](docs/learning/02_build_chain.md)：逐步理解 Waf、GCC、hwdef、MAVLink、ChibiOS、ELF、BIN 和 APJ；
3. [从 STM32 复位入口到 ArduSub](docs/learning/03_startup_to_ardusub.md)：连接 ChibiOS 启动、宏生成的 `main()`、HAL 回调、`AP_Vehicle::setup()` 和 `Sub::init_ardupilot()`；
4. [实时调度器和主循环](docs/learning/04_scheduler_and_main_loop.md)：理解 400 Hz 主循环、FAST_TASK、普通任务和时间预算；
5. 第 5～15 章依次学习参数、HAL、MAVLink、传感器、EKF、模式、控制器、推进器、任务、安全和日志；
6. 第 16～20 章按照[完整学习地图](docs/learning/README.md)练习新增传感器、模式、MAVLink 消息、新板卡和安全裁剪。

学习材料会将每一章限定在少量真实文件中，并提供调用链、练习和验收问题。源码内的中文注释用于定位当前函数在系统中的位置，完整原理以对应章节为准。

## 五分钟完成第一次编译

### 1. 获取源码和子模块

正常网络环境下，在 Ubuntu 或 WSL 中执行：

```bash
git clone --recurse-submodules https://github.com/HIT-Underwater-Robot/ardupilot.git ArduSub
cd ArduSub
```

如果已经克隆主仓库，但子模块不完整：

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

使用下面的命令检查子模块。每行开头为空格表示版本正确；`-` 表示尚未初始化，`+` 表示当前版本与主仓库记录不一致：

```bash
git submodule status --recursive
```

### 2. 安装 Ubuntu / WSL 编译环境

安装脚本会使用内部的 `sudo` 安装系统软件包、Python 环境和 STM32 ARM 交叉编译器。不要在脚本前加 `sudo`：

```bash
SKIP_AP_COMPLETION_ENV=1 Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

这里设置 `SKIP_AP_COMPLETION_ENV=1`，是因为本仓库没有保留可选的 `Tools/completion/` Bash 补全目录。

安装脚本主要准备四类依赖：

1. `git`、`make`、`g++`、`ccache` 等主机构建工具；
2. Python 3、虚拟环境以及 `pymavlink`、`empy`、`pyserial` 等生成和测试工具；
3. Pixhawk/STM32 使用的 `arm-none-eabi-gcc/g++` 交叉编译工具链；
4. SITL、MAVProxy 和图形界面可能使用的库。

当前 Waf 入口要求 Python 3.9 或更高版本。安装后可以先检查：

```bash
python3 --version
arm-none-eabi-g++ --version
```

如果安装脚本创建了虚拟环境，但新终端没有自动进入它，可以手动执行：

```bash
source ~/venv-ardupilot/bin/activate
```

### 3. 编译 Pixhawk 2.4.8

Pixhawk 2.4.8 在本构建系统中使用板卡名 `Pixhawk1`：

```bash
./waf configure --board Pixhawk1
./waf sub
```

`configure` 选择板卡、编译器和功能配置；切换板卡或修改配置选项时需要重新执行。`sub` 选择 ArduSub 程序组并执行增量编译。Waf 默认会自动并行，也可以限制线程数：

```bash
./waf sub -j4
```

主要输出文件位于 `build/Pixhawk1/bin/`：

| 文件 | 用途 |
| --- | --- |
| `ardusub.apj` | Mission Planner/QGroundControl 等工具通常使用的固件包 |
| `ardusub.bin` | 原始二进制固件 |
| `ardusub` | 带 ELF 信息的链接产物，用于符号、反汇编和调试 |
| `ardusub_with_bl.hex` | 包含 Bootloader 的 HEX 镜像 |

部分 Pixhawk 兼容板只有 1 MiB Flash。若固件在启动时明确报告 1 MiB/2 MiB 型号不匹配，应核对硬件后使用 `Pixhawk1-1M`，不要仅为绕过错误而更换板卡配置。

### 4. 编译 SITL

SITL 使用本机 C++ 编译器，不生成 STM32 固件：

```bash
./waf configure --board sitl
./waf sub
```

输出文件为：

```text
build/sitl/bin/ardusub
```

需要启动完整仿真环境时，可以使用：

```bash
Tools/autotest/sim_vehicle.py -v ArduSub --console --map
```

## 从 `./waf sub` 到固件的完整编译链

初学时先记住下面这条主线：

```text
./waf configure --board Pixhawk1
        │
        ├─ waf
        │    Python 启动器，检查 Python 版本
        │
        ├─ modules/waf/waf-light
        │    Waf 构建引擎
        │
        ├─ 根目录 wscript: configure()
        │    选择 Pixhawk1、ARM 工具链和编译功能
        │
        ├─ Tools/ardupilotwaf/ + AP_HAL_ChibiOS/hwdef/
        │    解析板卡的 MCU、Flash、UART、SPI、I2C、传感器和 ROMFS 配置
        │
        └─ build/Pixhawk1/
             生成 ap_config.h、hwdef.h、链接脚本和缓存配置

./waf sub
        │
        ├─ 根目录 wscript: build()
        │    注册动态源码、公共库和名为 sub 的构建组
        │
        ├─ MAVLink/DroneCAN 代码生成（按板卡功能启用）
        │
        ├─ ArduSub/wscript
        │    声明 ArduSub 直接依赖库和 ardusub 可执行目标
        │
        ├─ ArduSub/*.cpp + libraries/*
        │    由 arm-none-eabi-g++ 编译为目标文件和静态库
        │
        ├─ modules/ChibiOS + AP_HAL_ChibiOS
        │    提供实时操作系统、启动代码和 STM32 硬件实现
        │
        └─ 链接与打包
             生成 ardusub、ardusub.bin、ardusub.apj 等文件
```

`./waf sub` 中的 `sub` 不是某个源码文件名。根 `wscript` 创建了 `sub` 构建命令，`ArduSub/wscript` 又把最终程序加入 `sub` 组，因此 Waf 只构建这一组关联的目标。

ArduSub 也没有一个传统形式、需要我们手写循环的 `main.cpp`。车辆实例和调度表位于 `ArduSub/Sub.cpp`，文件末尾的 `AP_HAL_MAIN_CALLBACKS(&sub)` 将车辆对象交给 HAL；具体平台的程序入口由 HAL 提供。

## 编译依赖文件定位表

| 文件或目录 | 在编译中的职责 | 初学阶段是否需要阅读 |
| --- | --- | --- |
| `waf` | 最外层 Python 入口，调用仓库内的 Waf | 是，文件很短 |
| `modules/waf/` | Waf 构建引擎本体 | 暂时不用深入 |
| `wscript` | 全局配置、板卡选择、动态代码生成、目标递归和构建组定义 | 先读 `configure()`、`build()` 附近 |
| `Tools/ardupilotwaf/` | ArduPilot 对 Waf 的扩展，包括板卡、库、链接和固件打包规则 | 遇到构建问题时按调用定位 |
| `Tools/scripts/build_options.py` | 可选功能的编译开关定义 | 做固件裁剪时再读 |
| `ArduSub/wscript` | 声明 ArduSub 的直接库依赖和最终程序名 | 必读 |
| `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/hwdef.dat` | Pixhawk1 专属传感器和板卡差异 | 必读 |
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3/fmuv3.inc` | Pixhawk1 继承的 FMUv3 引脚、总线和 MCU 基础定义 | 部署或改板时必读 |
| `Tools/environment_install/install-prereqs-ubuntu.sh` | 安装主机软件、Python 包和 STM32 工具链 | 环境出错时阅读 |
| `.gitmodules` | 记录所有第三方子模块及其来源 | 子模块出错时阅读 |
| `build/<board>/ap_config.h` | Waf 根据板卡和编译选项生成的宏配置 | 排查功能是否启用时阅读，不要手改 |
| `build/<board>/hwdef.h` | hwdef 处理器生成的硬件宏 | 排查板卡定义时阅读，不要手改 |

### 关键子模块

| 子模块 | 作用 |
| --- | --- |
| `modules/waf` | 执行整个构建系统 |
| `modules/mavlink` | 保存 MAVLink XML，并由 `pymavlink` 生成消息头文件 |
| `modules/ChibiOS` | Pixhawk1 上使用的实时操作系统和 STM32 底层代码 |
| `modules/DroneCAN` | 板卡启用 CAN 功能时生成并编译 DroneCAN 支持 |
| `modules/littlefs` | 启用 LittleFS 的板卡所用文件系统 |
| `modules/CrashDebug` | 嵌入式崩溃捕获和调试支持 |
| `modules/gtest`、`modules/gbenchmark` | 单元测试和性能测试，不是理解主控制循环的入口 |
| `modules/Micro-CDR`、`modules/Micro-XRCE-DDS-Client` | DDS/micro-ROS 通信相关依赖 |
| `modules/lwip` | 使用网络功能的目标所需 TCP/IP 协议栈 |

仓库中存在某个库或子模块，不等于它的全部代码都会进入 Pixhawk1 固件。`ArduSub/wscript` 从 `ap_common_vehicle_libraries()` 和少量 ArduSub 专属库开始建立依赖；板卡能力、功能宏和各库的 `wscript` 再决定实际参与编译的后端。保留完整一级库目录，是为了保持官方依赖关系可验证，而不是要求学习者一次读完所有源码。

当前 `ArduSub/wscript` 显式增加了下面 7 个库：

- `AC_AttitudeControl`：姿态和角速度控制；
- `AC_WPNav`：航点、位置和导航控制；
- `AP_Camera`：相机控制接口；
- `AP_JSButton`：手柄按键功能映射；
- `AP_LeakDetector`：漏水检测；
- `AP_Motors`：推进器混控和归一化推力输出；
- `AP_TemperatureSensor`：温度传感器管理。

它们并不是 ArduSub 的全部依赖。`ap_common_vehicle_libraries()` 会加入调度器、参数、AHRS、通信、HAL 等公共车辆库，各库还会通过头文件、构建规则和功能宏继续形成传递依赖。初学时应把 `ArduSub/wscript` 看作依赖入口，而不是完整依赖清单。

### MAVLink 在编译时做了什么

根 `wscript` 的动态源码阶段读取：

```text
modules/mavlink/message_definitions/v1.0/all.xml
```

随后通过 `pymavlink`/`mavgen` 生成 C/C++ 消息定义，供 `libraries/GCS_MAVLink/` 和 `ArduSub/GCS_MAVLink_Sub.*` 使用。因此：

- `modules/mavlink/` 解决“协议消息长什么样、如何编码”；
- `libraries/GCS_MAVLink/` 解决“车辆公共 MAVLink 消息如何收发和分发”；
- `ArduSub/GCS_MAVLink_Sub.*` 解决“ArduSub 如何处理车辆相关消息”。

## 最短源码阅读路线

不建议从 `libraries/` 第一项开始逐个阅读。先沿下面的调用路线建立骨架：

| 想理解的问题 | 建议入口 |
| --- | --- |
| Waf 为什么能找到 ArduSub | `waf` → 根 `wscript` → `ArduSub/wscript` |
| ArduSub 对象包含哪些子系统 | `ArduSub/Sub.h` |
| 主循环按什么频率执行什么任务 | `ArduSub/Sub.cpp` 中的 `scheduler_tasks[]` |
| 系统如何初始化 | `ArduSub/system.cpp` |
| MAVLink 消息如何进入车辆层 | `ArduSub/GCS_MAVLink_Sub.cpp`、`GCS_Sub.cpp` |
| 模式如何选择和执行 | `ArduSub/mode.h`、`mode.cpp`、`mode_*.cpp` |
| 姿态和位置控制如何产生推力 | `ArduSub/Attitude.cpp`、`inertia.cpp`、`motors.cpp` |
| 推进器输出如何继续下沉 | `ArduSub/actuators.cpp` → `libraries/AP_Motors/` → `libraries/SRV_Channel/` → HAL |
| 传感器在哪里更新 | `ArduSub/sensors.cpp`，再进入对应的 `libraries/AP_*` 库 |
| 失控保护在哪里判断 | `ArduSub/failsafe.cpp` |
| 参数在哪里注册 | `ArduSub/Parameters.cpp`、`Parameters.h` |

第一轮阅读时，可以暂时跳过：

- 与 Pixhawk1 无关的其他 `hwdef` 板卡目录；
- `libraries/` 中当前主线没有走到的具体传感器后端；
- `modules/` 内部实现；
- `Tools/` 中与当前编译、SITL 或调试无关的辅助脚本。

## 常用 Waf 命令

```bash
./waf list_boards       # 列出板卡名
./waf board             # 查看当前 configure 选择的板卡
./waf list              # 列出当前配置下的目标
./waf -h                # 查看构建选项
./waf clean             # 清理当前板卡的目标文件，保留 configure 结果
./waf distclean         # 清理全部板卡和 configure 结果
```

通常不需要频繁执行 `clean`；Waf 会根据源码和配置变化进行增量编译。切换板卡时直接重新执行 `./waf configure --board <board>`。

## 常见问题

### `./waf: Permission denied`

确认工程位于 WSL 的 Linux 文件系统中，并恢复执行权限：

```bash
chmod +x waf Tools/environment_install/install-prereqs-ubuntu.sh
```

### 找不到 `arm-none-eabi-g++`

重新加载环境并检查 PATH：

```bash
. ~/.profile
which arm-none-eabi-g++
arm-none-eabi-g++ --version
```

### MAVLink、ChibiOS 或 Waf 文件缺失

通常是没有递归获取子模块：

```bash
git submodule update --init --recursive
git submodule status --recursive
```

### 修改了生成文件，但重新编译后丢失

`build/` 中的 `ap_config.h`、`hwdef.h`、MAVLink 头文件和链接脚本都是生成结果。应修改它们对应的 `wscript`、`hwdef.dat`、XML 或源配置，不能直接修改生成文件。

## WSL 无法直接访问 GitHub 时

Windows 只负责联网获取 Git 数据，源码仍可直接落在 WSL 文件系统中。下面命令在 Windows PowerShell 中执行，并把发行版名、用户名替换为实际值：

```powershell
git -c core.autocrlf=false clone --recurse-submodules --jobs 4 `
  https://github.com/HIT-Underwater-Robot/ardupilot.git `
  "\\wsl.localhost\Ubuntu\home\<WSL用户名>\ArduSub"
```

Windows Git 可能不会正确恢复 Linux 可执行位。仅在“刚完成克隆且尚未修改源码”时，在 WSL 中执行：

```bash
cd ~/ArduSub
git config core.filemode true
git checkout-index --all --force
git submodule foreach --recursive \
  'git config core.filemode true && git checkout-index --all --force'
```

子模块、系统依赖和 ARM 工具链准备完成后，普通 `configure` 和 `sub` 编译不需要访问 GitHub。

## 协作与维护原则

- 日常开发在功能分支进行，不直接在 `master` 上堆叠实验代码。
- 每次只沿一条明确主线修改，例如“新增传感器”“新增模式”或“新增 MAVLink 消息”。
- 不直接修改 `modules/` 子模块；协议或第三方组件需要二次开发时，应维护相应上游 Fork 和子模块提交。
- 不随意改变现有参数索引、失控保护和解锁条件。
- 修改后至少重新编译 `Pixhawk1` 和 `sitl`；涉及运行逻辑时，再增加对应 SITL 或台架测试。
- 同步官方 ArduPilot 更新后，重新初始化子模块并完成上述两类编译验证。

推荐的学习顺序是：先读懂本 README 的编译链，再阅读 `Sub.h` 和 `Sub.cpp` 的系统骨架，然后每次只跟踪一条消息、一个模式或一条推进器输出链。这样既保留官方功能，也不必同时面对整个 ArduPilot 工程。
