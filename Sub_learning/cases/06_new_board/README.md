# 案例 06：迁移一块 Pixhawk 类 STM32F765/ChibiOS 飞控板

> 设计练习：当前极简分支只接受 Pixhawk1；新增板卡必须另建分支完成 bootloader、总线、存储和拆桨输出验证。

> 状态：教学设计，当前仓库没有新增 `SubF765` 板卡目录。本案例使用“SubF765”作为项目占位名，演示完整迁移方法；在取得真实原理图、BOM、PCB 版本和唯一 board ID 前，不提供可烧写的虚构 `hwdef`。

## 1. 需求边界

假设项目需要一块基于 STM32F765/F767、整体架构接近 Pixhawk4/FMUv5 的自研飞控板，继续运行 ArduSub 4.7.0，服务 ROV 和 Pixhawk 类实时飞控场景。

本案例解决：

- ChibiOS bootloader 和主固件的板级定义；
- MCU、晶振、flash、USB/SWD、UART/I²C/SPI/CAN、传感器、存储和 PWM 资源；
- ArduSub board build、烧写、bring-up、安全输出和回退。

本案例不解决：

- 为新板复制或改写 flight mode、PID、EKF 或 6DOF mixer；
- 把 Orange Pi/普通 Linux SBC 变成实时飞控；
- 在 `mode_*.cpp` 中根据板名写 GPIO 特例；
- 在没有原理图证据时猜引脚、旋转、DMA 或传感器型号。

## 2. 为什么选择 Pixhawk4/FMUv5 作参考

当前基线中的真实文件：

- [`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/hwdef.dat`](../../../libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/hwdef.dat)
- [`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/hwdef-bl.dat`](../../../libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/hwdef-bl.dat)
- [`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/README.md`](../../../libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/README.md)
- [`libraries/AP_HAL_ChibiOS/hwdef/fmuv5/hwdef.dat`](../../../libraries/AP_HAL_ChibiOS/hwdef/fmuv5/hwdef.dat)
- [`libraries/AP_HAL_ChibiOS/hwdef/fmuv5/hwdef-bl.dat`](../../../libraries/AP_HAL_ChibiOS/hwdef/fmuv5/hwdef-bl.dat)

当前源码证明：

| 项目 | Pixhawk4/FMUv5 基线 |
|---|---|
| MCU 定义 | `STM32F7xx STM32F767xx`；注释说明覆盖 F765 类 |
| 外部晶振 | `16 MHz` |
| Flash | `2048 KB` |
| Bootloader load offset | `32 KB` |
| Board ID | `TARGET_HW_PX4_FMU_V5` |
| USB/SWD | OTG1 与 SWD pins 已定义 |
| 存储 | microSD + SPI FRAM |
| 内部传感器 | 多 IMU、MS5611、IST8310 等，以 hwdef 为准 |
| 输出 | IOMCU MAIN + FMU AUX PWM，timer/DMA 受资源分组约束 |

Pixhawk4 的 `hwdef.dat` 通过 `include ../fmuv5/hwdef.dat` 复用公共定义，再用 `undef`/重定义调整 LED 和电池比例。这说明“选择参考板”必须基于电气兼容差异，而不是只看 MCU 型号相同。

## 3. 开始编码前的硬件资料包

没有下表就停止板卡编码：

| 资料 | 必须回答 |
|---|---|
| 原理图 PDF + 版本 | 每个 MCU pin 接到什么器件/连接器，电平和上拉是什么 |
| BOM | MCU 完整料号、传感器完整料号、flash/FRAM/SD、电源和收发器 |
| 时钟 | HSE/LSE 频率、负载、旁路/晶振方式 |
| 电源树 | 各 rail、enable、power-good、过流、ADC 分压和上电默认状态 |
| Pin-mux 表 | UART/I²C/SPI/CAN/USB/SDMMC/timer channel/ADC/GPIO/CS/DRDY |
| DMA 表 | timer、SPI、UART、SDMMC 等是否冲突 |
| 传感器表 | bus、CS/address、DRDY、orientation、供电控制 |
| PWM/IO | 每个输出对应 timer/channel，是否有 IOMCU，电平和安全默认 |
| 调试/恢复 | SWD connector、BOOT pin、NRST、独立供电和量产烧录方式 |
| 产品要求 | 需要的 ArduSub 功能、端口数量、输出协议、存储和 failsafe |

文档里的每个 pin 都要能回指原理图页/网络名。`SubF765` 只是教程占位符，不能把 fmuv5 引脚表直接烧进一块未知 PCB。

## 4. 先做差异审计

建立一张 reference-vs-target 表：

| 子系统 | Pixhawk4/FMUv5 | SubF765 实板 | 处理 |
|---|---|---|---|
| MCU/flash | F765/F767 类，2 MB | `[原理图/BOM]` | 相同才可复用 MCU/flash 定义 |
| HSE | 16 MHz | `[原理图]` | 不同则主/bootloader 同步修改 |
| USB | PA11/PA12 OTG1 | `[原理图]` | pin、VBUS、connector、ESD 都要核对 |
| SWD | PA13/PA14 | `[原理图]` | 必须保留恢复路径 |
| UART order | USB + 多路 UART | `[接口需求]` | `SERIAL_ORDER` 决定参数映射 |
| I²C order | I2C3/I2C1/I2C2/I2C4 | `[连接器/内部设备]` | internal/external 语义必须正确 |
| SPI devices | IMU/baro/FRAM | `[器件和 CS]` | 每个 SPIDEV、mode、speed 有 datasheet 证据 |
| Sensor rotation | 现有 ROTATION 值 | `[PCB 安装方向]` | 由坐标图审查，不靠试飞猜 |
| microSD/FRAM | 均有 | `[BOM]` | 参数和日志存储策略随硬件决定 |
| CAN | 2 路 | `[收发器/standby]` | silent/enable pin 和终端电阻 |
| PWM/IOMCU | MAIN + AUX | `[架构]` | timer、DMA、safety 和上电状态逐项证明 |
| Battery scale | Pixhawk4 特定比例 | `[电阻值]` | 重新计算并用仪表校准 |
| LED/safety/buzzer | 现有 pin/极性 | `[原理图]` | active-high/low 不可猜 |

只有高度电气兼容时才使用 include + 少量 override。差异广泛时应写清晰的新 hwdef，而不是堆几十个 `undef` 形成不可审查的继承层。

## 5. 目录与职责

候选目录：

```text
libraries/AP_HAL_ChibiOS/hwdef/SubF765/
├── hwdef-bl.dat
├── hwdef.dat
├── defaults.parm          # 只有产品确有安全默认需求时
└── README.md
```

| 文件 | 职责 |
|---|---|
| `hwdef-bl.dat` | bootloader 所需 MCU、晶振、flash、load offset、USB/UART、LED、SWD 和安全 CS 默认 |
| `hwdef.dat` | 完整固件 pin mux、bus order、设备、存储、ADC、PWM、DMA、feature 和 ROMFS |
| `defaults.parm` | 板级默认参数；不能隐藏硬件错误或改变控制产品定义 |
| `README.md` | 硬件版本、特性、connector pinout、SERIAL/I²C mapping、输出分组、烧写与已知限制 |

控制层通常无需修改。若新增板需要的通用硬件能力无法由 hwdef 表达，先证明至少有明确的可复用 HAL 接口需求，再单独修改 `AP_HAL/AP_HAL_ChibiOS`。

## 6. Board ID 和固件身份

新硬件不能因为 MCU 相同就盲目复用 `TARGET_HW_PX4_FMU_V5`：

- `.apj` board ID 用于防止把错误固件刷入不兼容硬件。
- bootloader 和主固件必须使用同一身份。
- 新 identity 需要在 [`Tools/AP_Bootloader/board_types.txt`](../../../Tools/AP_Bootloader/board_types.txt) 中按项目/上游规则分配，不能从别的板复制一个数字。
- 若硬件与 Pixhawk4 真正二进制兼容并计划共用 identity，需要正式的兼容性审计，而不是默认假设。
- README 必须记录支持的 PCB revision；不同 revision 若引脚/传感器不兼容，要有 detection 或独立板定义。

教程不提供一个虚构的 `TARGET_HW_SUBF765` 数字。

## 7. `hwdef-bl.dat` 开发顺序

Bootloader 是第一阶段，因为它建立恢复和主固件加载路径。逐项加入：

1. `MCU`、`OSCILLATOR_HZ`、`FLASH_SIZE_KB`。
2. 唯一 `APJ_BOARD_ID`。
3. `FLASH_RESERVE_START_KB 0` 和经过 flash layout 审计的 `FLASH_BOOTLOADER_LOAD_KB`。
4. SWD pins 与 NRST/BOOT 硬件恢复说明。
5. 最小 USB 或 recovery UART。
6. boot/activity LED 及其有效电平。
7. 所有 SPI CS、power-enable、IOMCU/外设 reset 的安全上电状态。

bootloader 首次烧写必须通过 SWD；在验证 USB/串口升级、board ID 和错误固件拒绝前，不能依赖 bootloader 自己作为唯一恢复手段。

## 8. `hwdef.dat` 开发顺序

推荐每次只打开一组硬件：

```text
MCU/clock/flash/SWD
 -> console/USB
 -> storage
 -> one bus at a time
 -> internal sensors and rotations
 -> external connectors
 -> ADC/power monitoring
 -> CAN
 -> PWM/timer/DMA
 -> safety/IOMCU/arming output chain
```

关键规则：

- `SERIAL_ORDER` 决定 `SERIAL0_*` 等参数对应哪个端口，README 必须同步。
- `I2C_ORDER` 决定 ArduPilot bus number；内部/外部传感器 probing 依赖这个语义。
- `SPIDEV` 的 mode、低/高速率、CS 和驱动声明必须来自器件资料和已有 driver 能力。
- IMU/compass rotation 由 PCB 坐标和 ArduPilot 机体系推导，并由静态六面测试验证。
- PWM timer/channel/alternate function 必须对照 STM32 datasheet/AF 表；DMA 冲突需由 hwdef 检查和目标板 build 证明。
- 电池比例先按电阻网络计算，再用可信电压/电流仪表标定。
- 输出 enable、safety 和 CS 在 bootloader 与主固件阶段都要处于安全状态。

## 9. 构建和产物

在 WSL 的独立板卡开发分支中，典型顺序是：

```bash
./waf configure --board SubF765
./waf bootloader
./waf sub
```

实际命令需以当前 4.7.0 `./waf --help` 和构建规则为准。每次记录：

- configure/build 完整命令和 commit；
- compiler/linker 的真实成功或失败；
- bootloader、`.apj`/`.bin` 产物路径和 hash；
- flash/RAM 使用量和功能裁剪；
- hwdef generator 的 pin/DMA/资源冲突；
- 目标 PCB revision 和 board ID。

不得以 sudo 运行 Waf，不修改 `modules/`，不把 `build/` 当源码提交。

## 10. Bring-up 阶梯

### 阶段 A：无外设/无动力

1. 限流电源上电，检查各 rail、电流、复位和 MCU 时钟。
2. SWD 连接、读芯片 ID、擦写和断点。
3. 烧写 bootloader，观察 LED/console。
4. 验证 USB/UART 升级、board ID 和错误固件拒绝。
5. 烧写最小主固件，检查 scheduler、console 和 reboot reason。

### 阶段 B：存储和总线

- flash layout、参数保存/重启/恢复默认。
- FRAM/flash storage 与 microSD 日志。
- 每条 I²C/SPI/UART/CAN 单独验证，包含断线、短时错误和恢复。
- 传感器 WHOAMI、采样率、DRDY、温度、方向和多实例。
- USB 与高负载日志同时运行，观察 scheduler 和 bus error。

### 阶段 C：控制输入但不接推进器

- RC/GCS/MAVLink、flight-mode 参数和 arming checks。
- safety switch、buzzer、LED、power-good、CAN silent 和 IOMCU 通信。
- `SERVOx_FUNCTION` 到物理 connector 的逐通道映射。
- 示波器检查 PWM/DSHOT 的频率、占空、分组和上电默认。

### 阶段 D：拆桨推进器台架

1. 准备独立断电和限流，推进器拆桨或物理隔离。
2. 逐通道 motor test，核对标签、方向、中性和范围。
3. 验证 disarm、safety、interlock、pilot/GCS failsafe 和 MCU reset 时输出。
4. 逐轴小请求核对 AP_Motors6DOF 到实物通道。
5. 台架全部通过后，才进入低功率系留水池。

## 11. 验证矩阵

| 层级 | 必须证明 | 失败时回到 |
|---|---|---|
| 静态 hwdef | pin 唯一、AF 正确、资源/DMA 无冲突 | 原理图和 pin-mux 表 |
| Bootloader build | 正确 MCU、clock、layout、ID 和 recovery interface | `hwdef-bl.dat` |
| SWD/boot | 可重复烧写和恢复 | 电源/时钟/SWD 硬件 |
| Board build | ArduSub 链接、功能/flash/RAM 可接受 | hwdef/feature 选择 |
| Console/USB/storage | 启动、参数、日志、升级可靠 | clock/flash/bus |
| Sensors | 型号、方向、health、断线恢复 | bus/device/rotation |
| Outputs | timer、通道、协议和安全默认正确 | PWM/DMA/safety 定义 |
| SITL regression | 控制软件基线未因板卡移植改变 | 非 hwdef 的越界改动 |
| 拆桨台架 | arming、failsafe、逐通道和 reset 安全 | 输出/电气设计 |
| 系留水池 | 真实方向、通信、电源和温度可靠 | 前一安全阶段 |

SITL 不模拟自研板的电气、clock、DMA 或输出电平；board build 也不能证明传感器方向和 failsafe 实物行为。两者都要做，但证明对象不同。

## 12. 最小提交拆分

建议按可审查、可回退的顺序提交：

1. `SubF765: add bootloader hardware definition`
2. `SubF765: add minimal board hardware definition`
3. `SubF765: enable storage and internal sensors`
4. `SubF765: add external buses and board documentation`
5. `SubF765: add verified PWM and safety outputs`
6. `SubF765: add validated defaults`（确有必要时）

每个提交都写明 AI 协助、硬件证据、已执行测试和未测试风险。不要把 bootloader、全部 bus、控制改动和产品默认值塞进一个巨型提交。

## 13. 验收与回退

板卡验收不是“能编译”或“USB 能亮”：

- 原理图到 hwdef 的每个关键信号可追溯。
- bootloader 和主固件身份、flash layout 一致。
- SWD 永远可恢复，错误固件能被拒绝。
- console、storage、参数、所有目标 bus/传感器均有真实记录。
- PWM/DSHOT、safety、arming、failsafe、reset 在示波器和拆桨台架通过。
- ArduSub 控制层没有板名特例，SITL 基线回归通过。
- PCB revision、固件 hash、参数、日志和测试日期可复现。

回退资产必须在接推进器前准备好：旧 bootloader、旧主固件、SWD 工具、参数备份、明确的 BOOT/NRST 步骤和独立断电。若 board ID、clock、flash offset 或 safety 输出尚不确定，停止在 SWD/无动力阶段，不进入推进器测试。
