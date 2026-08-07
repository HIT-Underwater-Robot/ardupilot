# 第 2 章：从 C++ 源码到 Pixhawk ArduSub 固件

## 本章目标

执行下面两条命令时：

```bash
./waf configure --board Pixhawk1
./waf sub
```

你应该能够解释：

- 为什么先 configure，再 sub；
- `Pixhawk1` 在哪里定义；
- `sub` 为什么能找到 `ArduSub/wscript`；
- 哪些文件由 Python 生成；
- C++ 文件如何变成 `.o` 和 `.a`；
- ChibiOS 在哪里参与；
- 为什么最后同时出现 ELF、BIN、APJ 和 HEX。

## 1. 先理解最普通的 C++ 编译

假设只有两个文件：

```text
main.cpp
motor.cpp
```

最简构建过程是：

```text
main.cpp  ──编译──> main.o
motor.cpp ──编译──> motor.o

main.o + motor.o ──链接──> program
```

### 编译

编译器逐个处理 `.cpp`，生成机器代码目标文件 `.o`。这一阶段会发现：

- 语法错误；
- 类型不匹配；
- 找不到头文件；
- `override` 没有匹配基类函数。

### 链接

链接器把目标文件合成一个完整程序，并解析“一个文件调用了另一个文件中函数”的关系。这一阶段会发现：

- 函数只有声明、没有定义；
- 同一个符号被重复定义；
- 程序超过 Flash；
- 某个静态库没有加入链接。

ArduSub 原理相同，只是有上千个目标文件、许多自动生成文件和一个嵌入式操作系统。

## 2. 静态库 `.a` 是什么

如果直接把上千个 `.o` 平铺给链接器，依赖难以组织。ArduPilot 会先把一组目标文件归档成静态库：

```text
ArduSub/*.o                 → libArduSub_libs.a
ChibiOS 和板级底层目标文件 → libch.a
其他公共库目标文件         → 对应静态库/公共对象
```

静态库不是在 Pixhawk 上动态加载的插件。链接阶段会从 `.a` 中选取最终固件需要的符号，合并进同一个程序。

## 3. 原生编译与交叉编译

### SITL 原生编译

```bash
./waf configure --board sitl
./waf sub
```

编译器运行在 x86-64 Linux 上，生成也能在 x86-64 Linux 上运行的 `ardusub`。

### Pixhawk1 交叉编译

```bash
./waf configure --board Pixhawk1
./waf sub
```

编译器仍运行在 x86-64 Linux 上，但生成 ARM Cortex-M4 指令：

```text
构建主机：WSL / Ubuntu，x86-64
目标设备：Pixhawk1，STM32F427，ARM Cortex-M4
编译器：arm-none-eabi-gcc/g++
```

`none-eabi` 大致表示目标不是普通 Linux 用户程序，而是使用嵌入式 ABI 的裸机/RTOS 环境。

## 4. Waf 解决什么问题

你可以手工为每个 `.cpp` 执行 `arm-none-eabi-g++`，但必须自己处理：

- 上千个源文件；
- include 路径；
- 编译宏；
- 板卡差异；
- 自动生成的 MAVLink 代码；
- ChibiOS Make 规则；
- 链接顺序和链接脚本；
- 哪些文件发生变化，需要重新编译。

Waf 是构建系统：它读取 Python 编写的规则，建立任务依赖图，再调用真正的编译器、链接器和生成工具。

必须区分：

```text
Waf          决定“应该执行哪些任务以及先后顺序”
GCC/G++      把 C/C++ 编译成 ARM 目标文件
链接器       合并目标文件和静态库
objcopy      从 ELF 提取原始固件镜像
APJ 任务     把固件与板卡元数据封装给地面站
```

## 5. `./waf` 到底是什么

根目录的 `waf` 是一个很短的 Python 启动器。它做两件事：

1. 检查 Python 是否达到最低版本；
2. 执行 `modules/waf/waf-light`。

调用关系：

```text
Linux Shell
    ↓ 执行 ./waf
根目录 waf
    ↓ runpy.run_path()
modules/waf/waf-light
    ↓ Waf 寻找项目规则
根目录 wscript
```

为什么 Waf 放在 `modules/waf`？因为它是一个独立上游项目，本仓库通过 Git 子模块固定具体提交。

## 6. 根 `wscript` 是构建总入口

`wscript` 是 Python 文件，但不使用 `.py` 后缀。Waf 按约定寻找它。

最重要的两个函数：

```python
def configure(cfg):
    ...

def build(bld):
    ...
```

可以把它们理解为：

```text
configure：决定编译环境和功能
build：根据已保存的环境建立具体构建任务
```

## 7. `configure --board Pixhawk1` 的逐步过程

### 7.1 解析命令行

根 `wscript` 的 `options()` 注册 `--board`、`--debug`、功能开关等参数。

命令：

```bash
./waf configure --board Pixhawk1
```

使 `cfg.options.board` 变为 `Pixhawk1`。

### 7.2 动态发现板卡

`Tools/ardupilotwaf/boards.py` 扫描：

```text
libraries/AP_HAL_ChibiOS/hwdef/*/hwdef.dat
```

发现目录：

```text
libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/
```

后，动态创建名为 `Pixhawk1` 的 ChibiOS 板卡类。

因此 `Pixhawk1` 主要不是写死在一段巨大 `if/else` 中，而是来自 hwdef 目录。

### 7.3 选择 ChibiOS 和 ARM 工具链

板卡类加载 `Tools/ardupilotwaf/chibios.py`。它查找：

```text
arm-none-eabi-gcc
arm-none-eabi-g++
arm-none-eabi-objcopy
make
```

找不到工具时，configure 阶段就失败，不会等到编译一半才失败。

### 7.4 解析 Pixhawk1 硬件定义

入口文件：

```text
libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/hwdef.dat
```

它首先包含：

```text
libraries/AP_HAL_ChibiOS/hwdef/fmuv3/fmuv3.inc
```

可以理解为：

```text
fmuv3.inc
    提供 STM32F427、Flash、引脚、总线、UART、SPI、I2C 等基础定义

Pixhawk1/hwdef.dat
    在基础定义上补充 Pixhawk1 的 IMU、气压计、罗盘和 IO 固件差异
```

### 7.5 生成板卡配置

`chibios_hwdef.py` 把 hwdef 转换为构建文件，主要位于：

```text
build/Pixhawk1/hwdef.h
build/Pixhawk1/hw.dat
build/Pixhawk1/ldscript.ld
build/Pixhawk1/common.ld
build/Pixhawk1/Makefile
```

同时根构建系统生成：

```text
build/Pixhawk1/ap_config.h
```

作用区别：

- `hwdef.h`：板卡硬件能力和设备定义；
- `ap_config.h`：车辆、板卡和功能开关组合后的编译宏；
- `ldscript.ld`：告诉链接器 Flash/RAM 如何布局；
- `hw.dat`：保存用于固件识别/运行的硬件定义数据。

这些都是生成文件。应该修改来源，不应手改 `build/` 中结果。

### 7.6 保存 configure 结果

Waf 把板卡、编译器、宏和路径写入构建缓存。之后执行 `./waf sub` 时不需要重新输入板卡名。

检查当前板卡：

```bash
./waf board
```

## 8. `./waf sub` 中的 `sub` 是什么

根 `wscript` 有车辆命令列表，其中包含：

```python
vehicles = [..., 'sub']
```

随后为每个名字创建 Waf build command。

`ArduSub/wscript` 声明最终程序：

```python
bld.ap_program(
    program_name='ardusub',
    program_groups=['bin', 'sub'],
    ...
)
```

因此：

- `sub` 是构建组/命令；
- `ardusub` 是最终程序名；
- `bin` 决定它属于主要固件输出。

`./waf sub` 不是“编译名为 sub.cpp 的文件”，也不是“运行 main”。

## 9. `ArduSub/wscript` 怎样选择库

它先建立 `ArduSub_libs` 静态库，依赖由两部分组成：

```text
ap_common_vehicle_libraries()
    公共车辆基础库

ArduSub 显式增加的库
    AC_AttitudeControl
    AC_WPNav
    AP_Camera
    AP_JSButton
    AP_LeakDetector
    AP_Motors
    AP_TemperatureSensor
```

这仍不是全部最终依赖，因为：

- 一个库可以继续依赖其他库；
- 头文件会产生编译依赖；
- 板卡功能会加入 HAL、CAN、文件系统等目标；
- 功能宏会决定某些后端是否存在。

因此不要根据“文件夹在不在”判断它是否进入固件，应查看真实构建目标或 `Linker.map`。

## 10. 动态源码为什么必须先生成

有些 C++ 文件不是保存在 Git 主仓库中的固定文件，而是根据协议或配置生成。

### MAVLink

根 `wscript` 读取：

```text
modules/mavlink/message_definitions/v1.0/all.xml
```

通过 mavgen 生成 C/C++ 消息定义，然后 `GCS_MAVLink` 和 ArduSub 才能编译。

顺序必须是：

```text
MAVLink XML
    ↓ mavgen
生成消息头文件
    ↓
编译 GCS_MAVLink 和 ArduSub
```

### DroneCAN

Pixhawk1 hwdef 声明 CAN 接口后，构建系统才根据 DSDL 生成 DroneCAN 代码。

### 为什么叫动态源码

因为它们不是开发者直接编辑的最终 `.h/.cpp`，而是由 XML、DSDL、hwdef 或版本信息产生。

## 11. ChibiOS 怎样参与编译

ArduSub 是车辆程序，ChibiOS 是 Pixhawk1 上的实时操作系统和底层驱动环境。

`Tools/ardupilotwaf/chibios.py` 会调用 ChibiOS 的 Make 规则，生成：

```text
build/Pixhawk1/modules/ChibiOS/libch.a
```

它包含或关联：

- RTOS 内核；
- STM32 启动代码；
- 中断和线程支持；
- ChibiOS HAL；
- 与 hwdef 对应的板级配置。

最终链接关系可以简化为：

```text
ArduSub 车辆对象
    + ArduPilot 公共库
    + AP_HAL_ChibiOS
    + ChibiOS libch.a
    + 链接脚本 ldscript.ld
        ↓
build/Pixhawk1/bin/ardusub（ELF）
```

## 12. ELF、BIN、APJ 和 HEX 的区别

### `ardusub`：ELF

ELF 包含：

- 可执行机器码；
- 数据段；
- 符号信息；
- 各段地址；
- 调试器和分析工具需要的信息。

它最适合开发分析，但不是地面站常用上传格式。

### `ardusub.bin`：原始固件

`objcopy` 从 ELF 提取需要写入 Flash 的原始字节。它没有完整符号和丰富元数据。

### `ardusub.apj`：地面站固件包

APJ 是 JSON 结构的封装，包含：

- 压缩后的固件镜像；
- board ID；
- 板卡类型；
- Flash 大小；
- Git 标识；
- USB ID 等元数据。

地面站根据这些信息判断固件是否与板卡匹配。

### `ardusub_with_bl.hex`

把应用固件与 Bootloader 合并为 Intel HEX，适合特定烧录/恢复场景。普通地面站升级通常使用 APJ。

## 13. 为什么修改注释也可能触发大量重编译

Waf 计算文件内容和构建配置的变化。修改根 `wscript` 或广泛包含的头文件时：

- 构建规则哈希变化可能触发重新 configure；
- `Sub.h` 被许多编译单元包含，会使这些文件重新编译；
- 仅修改普通 `.cpp` 注释通常只影响单个编译单元。

重新编译不代表程序逻辑一定变化，只代表依赖图判断需要重新生成目标。

## 14. 如何观察真实构建，而不是猜测

### 查看当前板卡

```bash
./waf board
```

### 查看目标

```bash
./waf list
```

### 查看编译命令

```text
build/Pixhawk1/compile_commands.json
```

它记录每个 `.cpp` 使用的编译器、宏、include 路径和参数。遇到“为什么这个宏开启”“头文件从哪里找到”时，从这里查。

### 查看链接结果

```text
build/Pixhawk1/Linker.map
```

它可以回答：

- 某个函数是否进入最终固件；
- 它来自哪个 `.o` 或 `.a`；
- 各段占用多少 Flash/RAM。

### 查看生成宏

```text
build/Pixhawk1/ap_config.h
build/Pixhawk1/hwdef.h
```

只读它们来排查结果，不直接修改。

## 15. 增量构建、clean 和 distclean

### 普通增量构建

```bash
./waf sub
```

Waf 只重做输入发生变化的任务，日常开发优先使用。

### `clean`

```bash
./waf clean
```

清理当前板卡目标文件，但保留 configure 选择。

### `distclean`

```bash
./waf distclean
```

清除所有板卡构建和配置。下一次必须重新 configure。

不要把 clean 当作每次编译的固定步骤。频繁全量编译会掩盖你对依赖关系的理解，并浪费时间。

## 16. 常见错误应该在哪一层排查

| 错误现象 | 优先检查 |
| --- | --- |
| `./waf: Permission denied` | `waf` 的 Linux 执行权限 |
| 找不到 `waf-light` | `modules/waf` 子模块 |
| 找不到 `arm-none-eabi-g++` | 工具链安装和 PATH |
| `Invalid board Pixhawk1` | hwdef 目录和 `boards.py` 动态发现 |
| 找不到 MAVLink 头文件 | mavlink/pymavlink 子模块和动态生成任务 |
| undefined reference | 库没有链接、函数无定义或功能宏不一致 |
| region `flash` overflowed | 固件超过板卡 Flash |
| APJ 板卡不匹配 | APJ board ID、hwdef 和实际硬件 |

## 17. 本章实际观察练习

### 练习一：确认 configure 产物

执行：

```bash
./waf configure --board Pixhawk1
./waf board
```

然后打开：

```text
build/Pixhawk1/ap_config.h
build/Pixhawk1/hwdef.h
build/Pixhawk1/ldscript.ld
```

回答：哪些来自功能选择，哪些来自硬件定义？

### 练习二：找到 Sub.cpp 的真实编译命令

在 `build/Pixhawk1/compile_commands.json` 中搜索：

```text
ArduSub/Sub.cpp
```

识别：

- 使用的编译器；
- Cortex-M4 参数；
- include 路径；
- 自动包含的 `ap_config.h`；
- 优化级别。

### 练习三：证明一个函数是否进入固件

在 `build/Pixhawk1/Linker.map` 中搜索：

```text
Sub::run_rate_controller
```

再选择一个被关闭功能的函数，比较它是否存在。

### 练习四：观察增量构建

在不修改源码的情况下连续执行两次：

```bash
./waf sub -j4
./waf sub -j4
```

比较第二次执行了哪些任务。然后只修改一个普通 `.cpp` 注释，再观察哪些目标重新生成。

## 18. 完整编译链总结

```text
git clone --recurse-submodules
    ↓ 获取 Waf、MAVLink、ChibiOS 等固定版本

./waf configure --board Pixhawk1
    ↓
waf → modules/waf/waf-light → 根 wscript
    ↓
boards.py 动态发现 Pixhawk1 hwdef
    ↓
chibios.py 选择 ARM 工具链并解析 hwdef
    ↓
生成 ap_config.h、hwdef.h、链接脚本和构建缓存

./waf sub
    ↓
生成 MAVLink / DroneCAN / 版本等动态源码
    ↓
ArduSub/wscript 建立 ArduSub_libs 和 ardusub 目标
    ↓
arm-none-eabi-g++ 编译 ArduSub 与 libraries
    ↓
ChibiOS Make 生成 libch.a
    ↓
链接器按 ldscript.ld 生成 ardusub ELF
    ↓ objcopy
ardusub.bin
    ↓ APJ 封装
ardusub.apj
    ↓ 可选合并 Bootloader
ardusub_with_bl.hex
```

## 本章验收问题

不看本文，尝试回答：

1. Waf 和 GCC 分别负责什么？
2. 为什么 Pixhawk1 需要交叉编译？
3. `configure` 生成了哪些关键文件？
4. `Pixhawk1` 板卡名来自哪里？
5. `sub` 和 `ardusub` 有什么区别？
6. `ArduSub/wscript` 为什么不是完整依赖清单？
7. MAVLink 头文件为什么必须先生成？
8. ChibiOS 最终以什么形式参与链接？
9. ELF、BIN 和 APJ 分别适合什么用途？
10. 怎样证明某个函数真的进入了固件？

全部能够独立回答后，下一步是“从平台 `main()` 到 ArduSub”，把本章的最终 ELF 与程序启动后的第一条运行调用链连接起来。
