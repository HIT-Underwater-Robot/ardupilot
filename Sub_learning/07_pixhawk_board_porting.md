# Pixhawk 类 STM32/ChibiOS 板卡移植

## 1. 先确认它属于本仓库边界

本仓库支持的扩展方向是 Pixhawk 类 STM32/ChibiOS 实时飞控板。Orange Pi、树莓派等 Linux 设备默认是 MAVLink 伴随计算机，不替代负责确定性调度、硬件 safety、传感器采样和推进器输出的飞控主控。把 Linux SBC 变成飞控属于另一项高风险架构，不在本仓库默认维护范围内。

## 2. 选择 MCU 与参考板

在写代码前完成硬件对照：

- MCU 型号、封装、flash、RAM 和可用 bank。
- 主/辅助晶振、PLL、USB 时钟和启动方式。
- 电源轨、brownout、复位、watchdog 和硬件 safety。
- SWD/JTAG、BOOT 引脚和不可启动时的恢复路径。
- IMU、磁罗盘、气压/深度、存储和传感器总线。
- UART、I2C、SPI、CAN、USB 的引脚和 DMA 冲突。
- PWM/DSHOT 定时器、通道、DMA、IO 电平和 ESC 接口。

选择同 MCU、相似时钟和相似传感器拓扑的现有 `hwdef` 作为参考，但逐项核对原理图，不能复制后靠试烧猜引脚。

## 3. 建立 board definition

新板卡目录位于：

```text
libraries/AP_HAL_ChibiOS/hwdef/<board>/
```

核心文件：

- `hwdef.dat`：应用固件的 MCU、时钟、总线、传感器、输出、存储和 feature 配置。
- `hwdef-bl.dat`：bootloader 所需的最小硬件配置。
- `defaults.parm`：仅在确实需要板级默认参数时添加，并逐项评审。
- `README.md`：记录硬件版本、已知限制、烧写和恢复方法。

[Pixhawk4 hwdef](../libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/) 同时包含 `hwdef.dat`、`hwdef-bl.dat`、默认参数和板卡说明，可作为当前目标板的证据入口。

## 4. 先构建 bootloader

```bash
./waf configure --board <board> --bootloader
./waf bootloader
```

确认生成文件、flash offset、USB VID/PID、board ID 和烧写方法。首次烧写前必须准备 SWD 恢复；不要在没有恢复手段时覆盖唯一可用 bootloader。

## 5. 再构建 ArduSub

```bash
./waf configure --board <board>
./waf sub -j"$(nproc)"
```

检查：

- 链接成功且 flash/RAM 有安全余量。
- 产物中 board ID、固件类型和版本正确。
- 没有通过关闭关键 failsafe、传感器或输出保护来勉强塞入 flash。
- feature guard 关闭时公共代码仍可编译。

## 6. 分阶段上电验证

推荐顺序：

1. SWD 连接、复位、bootloader 和 console。
2. 主时钟、USB、flash、参数存储和 SD 卡。
3. 每条 I2C/SPI/UART/CAN 总线的电平、时钟和设备枚举。
4. IMU 方向、采样率、DRDY、温度和振动数据。
5. 气压/深度、磁罗盘、GPS 或水下定位接口。
6. RC/MAVLink、hardware safety、arming 和 failsafe。
7. 仅用示波器验证 PWM/数字输出，推进器不接负载。
8. 逐通道空载推进器测试。
9. 受控水池中的低功率、低风险回归。

每阶段通过后再进入下一阶段；不要在传感器方向和 failsafe 尚未验证时连接全部推进器。

## 7. 输出与 DMA 审计

STM32 定时器资源可能同时被 PWM、DSHOT、蜂鸣器、输入捕获或其他外设使用。对每个输出列出：

- MCU pin 和 alternate function。
- timer 和 channel。
- DMA stream/request 及冲突。
- 输出电平、反相、上拉/下拉和隔离电路。
- SRV 功能、推进器编号和物理连接器。

示波器检查中性、最小、最大、更新率和 disarm/failsafe 波形。方向验证在参数、混控系数、ESC 和物理安装四层分别记录。

## 8. 板卡移植的交付物

完整移植至少交付：

- 原理图版本和参考板差异表。
- `hwdef.dat`、`hwdef-bl.dat` 和必要的板卡说明。
- bootloader 与 ArduSub 的真实构建命令、SHA 和尺寸。
- 总线/传感器/存储/USB/console 验证记录。
- 每个输出通道的示波器和方向记录。
- arming、hardware safety、通信中断和 failsafe 结果。
- 已知限制、参数备份、可用旧固件和 SWD 回退步骤。

HAL/board definition 应解决绝大多数硬件适配。只有当现有 HAL 抽象确实无法表达硬件能力时，才评估小范围、可复用的 HAL 扩展；不要在 ArduSub 控制层增加板卡名称判断或 GPIO 特例。
