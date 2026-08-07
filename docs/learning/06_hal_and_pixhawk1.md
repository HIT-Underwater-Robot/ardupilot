# 第 6 章：HAL 与 Pixhawk1 硬件映射

这一章把三个名称层次分开：

~~~text
车辆功能：MAVLink、GPS、深度计、推进器
        ↓
逻辑接口：SERIAL2、hal.i2c[1]、hal.rcout
        ↓
硬件资源：USART3、I2C1、STM32 定时器、IO MCU
~~~

车辆代码尽量停留在前两层；hwdef 和 AP_HAL_ChibiOS 负责最后一层。

## 1. 文件地图

1. **libraries/AP_HAL/HAL.h**：硬件接口集合；
2. **libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.h/.cpp**：ChibiOS 实现；
3. **libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/hwdef.dat**：Pixhawk1 差异；
4. **libraries/AP_HAL_ChibiOS/hwdef/fmuv3/fmuv3.inc**：继承的硬件定义；
5. **libraries/AP_SerialManager/**：串口协议分配；
6. **build/Pixhawk1/hwdef.h** 和 **hw.dat**：生成结果。

## 2. HAL 解决什么

若 ArduSub 直接调用 STM32 API，车辆逻辑会与某颗 MCU 和外设编号绑定。ArduPilot 改为调用 hal.serial、hal.i2c_mgr、hal.spi、hal.rcout、hal.scheduler 和 hal.storage 等抽象接口。

Pixhawk1 中它们指向 AP_HAL_ChibiOS；SITL 中指向 AP_HAL_SITL。因此同一份车辆控制代码可以在主机仿真，也可运行在不同飞控硬件上。

## 3. HAL 是接口对象的组合

~~~text
HAL
    ├─ scheduler：延时、线程和系统调度
    ├─ serial：UART/USB 驱动数组
    ├─ i2c_mgr：I2C 总线
    ├─ spi：SPI 设备
    ├─ gpio：数字输入输出
    ├─ rcin：遥控输入
    ├─ rcout：PWM/DShot 输出
    ├─ analogin：ADC
    ├─ storage：参数持久化
    └─ util：平台辅助和诊断
~~~

软件硬件分离不是无需硬件配置，而是车辆层通过稳定接口使用硬件，具体资源由板卡层提供。

## 4. Pixhawk1 文件为何很短

Pixhawk1/hwdef.dat include 了 ../fmuv3/fmuv3.inc。大部分 MCU、引脚、总线、定时器和存储定义来自 fmuv3.inc。

Pixhawk1 文件只补充或覆盖两组 IMU、板载 MS5611、板载罗盘、外部罗盘探测、2 MiB Flash 检查和 IO MCU DShot 固件。

## 5. hwdef 如何进入固件

~~~text
Pixhawk1/hwdef.dat
    ↓ include
fmuv3/fmuv3.inc
    ↓ chibios_hwdef.py
build/Pixhawk1/hwdef.h
build/Pixhawk1/hw.dat
链接脚本、DMA 配置和板卡宏
    ↓
AP_HAL_ChibiOS 驱动编译
~~~

hwdef 主要在构建阶段解析。不要编辑 build 目录生成文件，修改 hwdef 后通常应重新 configure。

## 6. Pixhawk1 基本资源

fmuv3.inc 指定：

- STM32F4xx / STM32F427xx；
- 24 MHz 外部晶振；
- 2048 KiB Flash；
- TIM5 系统计时器；
- TARGET_HW_CUBE_F4 board ID。

board ID 用于 Bootloader 和地面站判断固件适配性，不能当作显示名称随意更改。

## 7. SERIAL 参数到物理串口

关键定义：

~~~text
SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7
~~~

| 参数 | HAL 端口 | 常见接口 |
|---:|---|---|
| SERIAL0 | OTG1 | USB/console/主 MAVLink |
| SERIAL1 | USART2 | TELEM1 |
| SERIAL2 | USART3 | TELEM2 |
| SERIAL3 | UART4 | 主 GPS |
| SERIAL4 | UART8 | GPS2 |
| SERIAL5 | UART7 | 额外串口 |

所以：

~~~text
TELEM2 插座
    ↓
STM32 USART3
    ↓
hal.serial(2)
    ↓
SERIAL2_PROTOCOL / SERIAL2_BAUD
~~~

不能因为 MCU 外设叫 USART3，就修改 SERIAL3。SERIALn 是 HAL 数组顺序，不是 STM32 外设尾号。

## 8. SerialManager 按协议找端口

通用驱动通常通过 AP_SerialManager::find_serial(目标协议, instance) 查找端口。

SerialManager 遍历 SERIALx_PROTOCOL，找到第几个匹配协议的端口，再返回 UARTDriver。这样设备可从 TELEM2 换到其他串口，无需重新编译驱动。

注意两套编号：

- SERIAL2 是端口槽位；
- instance 0 是某协议的第一个实例。

## 9. 波特率与协议

SERIALx_PROTOCOL 决定谁使用端口和初始化策略。SERIALx_BAUD 决定常规速率，但有些协议使用固定速率。

SerialManager::init() 会读取 state、设置 options、根据协议选择缓冲区和 begin 参数、对 None 端口关闭可复用收发引脚，并把自行初始化的协议留给对应库。

## 10. I2C 也经过重排

~~~text
I2C_ORDER I2C2 I2C1
~~~

历史约定 HAL bus 0 为内部总线、bus 1 为外部总线。fmuv3 上 STM32 I2C2 是内部，I2C1 是外部，因此按上述顺序重排。

HAL 总线编号不一定等于 STM32 外设编号。

## 11. MS5837 放在哪里理解

Pixhawk1 的 BARO MS5611 SPI:ms5611 描述板载气压计。外接 MS5837 通过 AP_Baro 前端探测对应后端，ArduSub 初始化再寻找 AP_Baro::BARO_TYPE_WATER。

系统可同时拥有板载空气气压计和外接水压计。ArduSub 选择水压类型用于深度逻辑，而不是用 MS5837 替换板载定义。

## 12. SPI 设备

SPI 共享 SCK、MISO、MOSI，每个设备通常有独立 CS。hwdef 还需表达 SPIDEV 名称、模式、低速/高速频率和设备方向或探测信息。

驱动通过设备名取得 AP_HAL::SPIDevice，而不是让车辆代码直接控制 CS GPIO。

## 13. PWM 与 IO MCU

Pixhawk1 同时涉及 FMU 定时器输出、IO MCU 管理的 MAIN 输出、SRV_Channels 功能映射和 AP_HAL::RCOutput。

hwdef 中类似 PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50) 的行，把 STM32 引脚描述为定时器 PWM 通道并赋予 HAL 编号。

~~~text
六自由度控制量
    ↓
AP_Motors6DOF
    ↓
SRV_Channels
    ↓
AP_HAL::RCOutput
    ↓
FMU 定时器或 IO MCU
    ↓
PWM / DShot
~~~

## 14. 总线选择

| 总线 | 典型情况 | 关注点 |
|---|---|---|
| UART | 连续字节、设备主动发送 | 波特率、帧同步、超时、缓冲 |
| I2C | 短距离寄存器设备 | 地址、总线、探测、锁 |
| SPI | 高速板载传感器 | CS、模式、频率、DMA |
| CAN | 多节点、较长距离 | 节点、协议、终端、速率 |

水下设备还要考虑布线、隔离、错误恢复和驱动架构。

## 15. SERIAL2_PROTOCOL 的完整反查

~~~text
QGC 设置 SERIAL2_PROTOCOL
    ↓
AP_Param 载入 SerialManager state[2]
    ↓
AP_SerialManager::init()
    ↓
hal.serial(2)
    ↓
HAL_ChibiOS UARTDriver
    ↓
SERIAL_ORDER 第 2 项 USART3
    ↓
PD8/PD9/PD11/PD12
    ↓
ChibiOS USART3 驱动
~~~

这是“参数 → 公共库 → HAL → hwdef → MCU”的标准反查方法。

## 16. Pixhawk6 的差异放在哪里

换板时主要变化通常在 hwdef、MCU/容量、ChibiOS 启动、总线排布、板载传感器、PWM/IO 架构、Bootloader 和默认功能。

大部分 ArduSub 模式和控制器不应变化。如果换板需要大改 Mode，通常说明硬件差异泄漏到了车辆层。

## 17. 新板卡不能只靠复制

复制相近板卡可作起点，但要逐项核对原理图、MCU 数据手册、alternate function、DMA、时钟树、Flash、Bootloader、电源控制和传感器方向。

能编译不等于能安全驱动真实硬件。

## 18. 只读实验

### 实验 A：TELEM2 反查

找到 SERIAL2 → SERIAL_ORDER → USART3 → PD8/PD9 → UARTDriver 的源码证据。

### 实验 B：比较生成文件

对照 Pixhawk1/hwdef.dat、fmuv3/fmuv3.inc 和 build/Pixhawk1/hwdef.h，观察输入如何展开。不要编辑生成文件。

### 实验 C：SITL 对比

确认同样的 hal.serial 和 hal.rcout 接口在 SITL 中由 AP_HAL_SITL 提供。

## 19. 本章验收问题

1. HAL 为什么不是一个万能驱动？
2. 为什么还要阅读 fmuv3.inc？
3. hwdef 在何时解析？
4. SERIAL2 为什么对应 USART3？
5. protocol instance 与 SERIALn 有何区别？
6. I2C bus 0 为什么对应 I2C2？
7. MS5837 为什么不直接替换 MS5611？
8. SRV_Channels 与 RCOutput 职责有何不同？
9. 哪些差异应留在板卡目录？
10. 为什么成功编译不足以证明新板可用？
