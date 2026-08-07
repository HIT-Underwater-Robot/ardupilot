# 第 19 章：部署到现有 Pixhawk6 或新板卡

先区分两种任务：

~~~text
使用官方已支持板卡
    选择已有 --board，编译和验证

移植全新硬件
    新 hwdef + Bootloader + 板级验证
~~~

升级到 Pixhawk6C/6X 通常属于第一种，不需要复制 ArduSub 控制代码。

## 1. 使用已有 Pixhawk6

当前仓库已包含 Pixhawk6C、Pixhawk6X 及部分 bdshot/ODID 变体目录。

基本命令：

~~~sh
./waf list_boards
./waf configure --board Pixhawk6C
./waf sub
~~~

选择名称必须与实际硬件精确匹配。Pixhawk6C 与 Pixhawk6X、普通与 bdshot 目标不能仅因名称相近互换。

## 2. 切换板卡前先保存

- 参数文件；
- 当前固件版本和 commit；
- bootloader 版本；
- 接线图；
- SERIALx 映射；
- SERVOx_FUNCTION；
- 传感器方向；
- 日志基准；
- 原板固件备份/可恢复路径。

不要把 Pixhawk1 的全部参数无审查复制到新板，尤其是总线、输出协议和板载传感器参数。

## 3. 已有板卡编译验收

1. configure 成功；
2. sub 编译成功；
3. APJ board ID 正确；
4. Flash/RAM 有裕量；
5. bootloader 接受；
6. USB 枚举；
7. QGC 连接；
8. 板载 IMU/Baro；
9. 外部 MS5837；
10. TELEM 映射；
11. Safety；
12. PWM/DShot 无推进器验证。

## 4. 新板卡所需输入

真正新板必须有：

- 原理图；
- BOM 和传感器型号；
- MCU 完整型号；
- Flash/RAM；
- 晶振；
- 电源树；
- USB；
- UART/I2C/SPI/CAN；
- PWM 定时器；
- DMA；
- SD/FRAM；
- Safety/蜂鸣器/LED；
- SWD；
- Boot 引脚；
- Bootloader Flash 分区。

没有原理图不能可靠生成 hwdef。

## 5. 选择最近参考板

参考板应优先满足：

1. 同 MCU；
2. 相同时钟；
3. 相似 Flash；
4. 相似 IO MCU；
5. 相似传感器；
6. 相似输出。

只复制市场名称最像的目录不是可靠方法。

## 6. 新 hwdef 目录

通常包含：

~~~text
libraries/AP_HAL_ChibiOS/hwdef/MyBoard/
    hwdef.dat
    hwdef-bl.dat
    defaults.parm（可选）
~~~

可 include 公共 MCU/板型片段，但每一条 override 都应有硬件证据。

## 7. hwdef.dat 核心

逐项定义：

- MCU；
- oscillator；
- Flash size；
- APJ board ID；
- storage；
- USB；
- SERIAL_ORDER；
- I2C_ORDER；
- SPI bus 和 SPIDEV；
- CAN；
- ADC；
- PWM/timer；
- GPIO；
- IMU/BARO/COMPASS 及旋转；
- DMA 优先/禁止；
- ROMFS；
- compile defines。

生成后阅读 build/板卡/hwdef.h，检查解析结果而非只看输入文件。

## 8. Bootloader

hwdef-bl.dat 是 Bootloader 的精简硬件定义，通常至少需要：

- MCU/时钟；
- Flash 布局；
- USB 或上传接口；
- board ID；
- LED；
- 必要引脚。

主固件能编译不代表 Bootloader 正确。错误 Flash offset 可导致无法启动甚至需要 SWD 恢复。

## 9. board ID

board ID 连接：

~~~text
Bootloader 报告硬件 ID
    ↕
APJ 元数据
    ↕
地面站允许刷写
~~~

新公共硬件应按 ArduPilot/Bootloader 规则申请唯一 ID。私自复用其他板 ID 会让错误固件被接受。

## 10. 引脚复用和 DMA

STM32 一个引脚可能支持多个 alternate function，但原理图连接和 MCU 表共同限制选择。

DMA 冲突可能让单个驱动工作、多个驱动同时启用时失败。生成工具能帮助分配，但仍要检查：

- 高速 IMU；
- DShot；
- SDIO；
- UART；
- SPI；
- timer update。

## 11. 传感器方向

hwdef 中 rotation 描述芯片相对机体的安装。错误方向可能表现为：

- 静止姿态错误；
- 多 IMU 不一致；
- EKF 不健康；
- 控制方向反。

不能通过把控制器某个轴取负来补偿板载 IMU 朝向错误。

## 12. 输出验证

从逻辑到引脚逐层：

~~~text
SERVO function
→ HAL channel
→ timer/group 或 IO MCU
→ pin
→ connector
→ ESC
~~~

用示波器/逻辑分析仪确认频率、中点、端点和 Safety，不要第一次验证就连接可产生推力的设备。

## 13. 编译不等于移植成功

构建只验证：

- hwdef 可解析；
- 宏和驱动可编译；
- 链接容量。

还要验证：

- 时钟；
- USB；
- storage；
- reset/watchdog；
- 每条总线；
- 传感器；
- SD；
- CAN；
- 输出；
- Safety；
- 长时间稳定；
- brownout 和恢复。

## 14. 分阶段 Bring-up

1. SWD 可连接；
2. Bootloader 和 USB；
3. 空主固件启动；
4. console；
5. storage/参数；
6. 单个 IMU；
7. 其他板载传感器；
8. SD/log；
9. 外部总线；
10. RC input；
11. Safety；
12. 无负载 outputs；
13. SITL/硬件在环对照；
14. 受控整机。

## 15. 回滚

始终准备：

- SWD；
- 已知可用 Bootloader；
- 已知可用固件；
- 参数备份；
- 固件 hash；
- 恢复说明。

如果无法恢复，就不应在现场直接刷第一次自定义 Bootloader。

## 16. 本项目的现实路线

近期：

- Pixhawk1 作为基准构建；
- MS5837；
- TELEM2/伴随计算机；
- SITL。

升级：

- 选择官方 Pixhawk6C/6X 目标；
- 先不改车辆代码；
- 重新核对 SERIAL、I2C、输出和传感器；
- 对照相同测试矩阵。

只有研发自有控制板时，才进入全新 hwdef 和 Bootloader 移植。
