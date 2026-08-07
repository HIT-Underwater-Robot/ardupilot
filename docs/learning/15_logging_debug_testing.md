# 第 15 章：日志、调试与分层验证

对安全关键系统，“能够编译”只证明语法、类型和链接关系基本成立。完整证据链是：

~~~text
静态检查
→ 编译
→ 单元测试
→ SITL
→ MAVLink/日志核对
→ 无桨台架
→ 系留水池
→ 受控实航
~~~

## 1. 文件地图

1. **ArduSub/Log.cpp**：Sub 专属日志；
2. **ArduSub/Sub.cpp**：日志任务频率；
3. **libraries/AP_Logger/**：Frontend 和存储 Backend；
4. **Tools/autotest/ardusub.py**：SITL 集成测试；
5. **build/板卡/compile_commands.json**；
6. **build/板卡/Linker.map**；
7. **build/板卡/bin/ardusub**：ELF；
8. **build/板卡/bin/ardusub.apj**：烧录包。

## 2. 三种观察手段

### GCS_SEND_TEXT

适合低频、需要人立刻看到的状态和错误。不适合高频数据流。

### MAVLink Inspector

适合确认线上消息：

- 是否收到；
- 频率；
- system/component；
- 字段；
- 是否转发。

它不能单独证明内部控制器采用了该数据。

### DataFlash/AP_Logger

适合高频时间序列、控制器内部量、模式、failsafe、innovation 和输出，是事后分析主工具。

## 3. Logger 前后端

AP_Logger 前端提供统一 Write 接口，Backend 决定写到 SD 卡、文件或其他目标。

~~~text
Sub / EKF / Controller
    ↓ Write_xxx
AP_Logger
    ↓
Logger Backend
    ↓
SD/File
~~~

业务代码不应直接操作 FATFS 文件保存每帧控制数据。

## 4. 二进制日志结构

日志 packet 包含 LOG_PACKET_HEADER 和固定字段。LogStructure 描述：

- message ID；
- 长度；
- name；
- format；
- labels；
- units/multipliers。

写入结构、format 字符串和字段数量必须严格一致。改变结构而不更新描述会让日志解析错位。

## 5. ArduSub 记录什么

Sub 专属与公共日志可包括：

- attitude 与 target；
- rate/PID；
- control tuning；
- Guided target；
- RCIN/RCOUT；
- vibration/IMU；
- motors；
- mode 和 reason；
- failsafe error/event；
- position controller；
- VisualOdom/EKF。

实际是否记录受 LOG_BITMASK、编译开关、存储状态和调度预算影响。

## 6. 日志频率

Sub 任务表有：

- 10 Hz logging loop；
- 25 Hz logging；
- loop rate logging；
- AP_Logger periodic_tasks。

高频日志会消耗 CPU、RAM buffer 和 SD 带宽。新增日志要选择能回答问题的最低合理频率。

## 7. 一次问题需要哪些证据

例如“Guided 位置跳动”：

1. MAVLink Inspector：ODOMETRY 和 setpoint 是否稳定；
2. VisualOdom 日志：输入时间、位置、速度、quality；
3. EKF：source、innovation、variance、reset；
4. Guided target；
5. position estimate；
6. controller output；
7. motor/RCOUT；
8. mode/failsafe event。

只录最终 PWM 无法判断错误来自规划、估计、控制还是混控。

## 8. SITL 能验证什么

SITL 用主机程序替代 MCU 和真实物理环境，适合：

- 启动和参数；
- 模式切换；
- MAVLink；
- Mission；
- 控制状态机；
- failsafe；
- 外部里程计注入；
- 自动回归。

它不能完全证明：

- Pixhawk 时序和 DMA；
- 真实 I2C/UART 电气；
- 推进器水动力；
- 漏水传感器可靠性；
- 实际 ESC DShot；
- CPU/Flash 板级裕量。

## 9. 单元测试与 Autotest

单元测试适合纯函数、转换、解析器和边界条件。

Autotest 启动完整 SITL，通过 MAVLink 操作车辆并断言行为，适合模式、任务和 failsafe 集成。

新增功能的测试至少覆盖：

- 正常路径；
- 非法输入；
- 超时；
- 状态切换；
- 恢复；
- 重启后参数；
- 不启用功能时的兼容。

## 10. compile_commands.json

它记录每个源文件的真实编译命令：

- 编译器；
- include 路径；
- define；
- 优化；
- 目标文件。

当“头文件明明存在却找不到”或“某 ifdef 为什么没进入”时，用它比猜 Waf 配置更直接。

## 11. ELF、BIN、APJ

### ELF

包含 section、符号和调试信息。可用 nm、objdump、readelf 和 gdb 分析。

### BIN

接近要写入 Flash 的原始镜像。

### APJ

包含板卡 ID、镜像和元信息，供地面站/Bootloader 安全选择和上传。

不要用 APJ 文件大小直接代替链接器报告中的 Flash used。

## 12. Linker.map

Map 文件能回答：

- 某符号来自哪个目标文件/静态库；
- 哪些 section 占 Flash/RAM；
- 某驱动是否真的链接进固件；
- 删除编译开关前后体积变化来自哪里。

判断库是否“用到”应结合编译依赖和最终链接，而不是只看 libraries 目录存在。

## 13. 崩溃与内部错误

可关注：

- AP_InternalError；
- watchdog/reset reason；
- persistent_data 中最后任务和 MAVLink msgid；
- crash dump；
- ELF 符号；
- 上次日志尾部。

崩溃后不要只重刷固件清现场，应先保存参数、日志、固件版本和复现步骤。

## 14. Pixhawk 上机验证阶梯

1. 只编译；
2. 板卡通电不接推进器；
3. 检查传感器和参数；
4. 逻辑分析仪检查串口/PWM；
5. ESC 通电但推进器采取物理安全措施；
6. 单推进器低输出方向测试；
7. 全推进器无负载/受控测试；
8. 系留浅水；
9. 逐步增加模式；
10. 保存每次配置和日志。

每一级失败都退回当前级定位，不带着未知问题进入更危险环境。

## 15. 可复现问题报告

至少包含：

- commit hash；
- board 和 bootloader；
- 编译命令；
- 参数文件；
- 接线和传感器版本；
- 操作步骤；
- 预期与实际；
- 时间对应的日志；
- MAVLink 输入样本；
- 是否 SITL/台架/水池；
- 是否能稳定复现。

不能声称“测试通过”却没有实际执行记录。

## 16. 本项目的最低回归矩阵

文档/注释：

- Markdown links 和 fence；
- diff whitespace；
- 非注释代码语义不变；
- Pixhawk1 build。

车辆代码：

- SITL build；
- 相关单元测试；
- ArduSub autotest；
- Pixhawk1 build；
- 参数兼容；
- 目标功能场景；
- 关键 failsafe。

板卡代码：

- bootloader；
- board build；
- pin/bus 检查；
- 无推进器上电；
- 实际外设；
- Safety 和 RCOutput。

## 17. 验收问题

1. GCS text、Inspector 和 DataFlash 各适合什么？
2. 为什么高频任务不能大量发文本？
3. 日志 struct 和 format 不一致会怎样？
4. SITL 不能证明哪些硬件问题？
5. 何时写单元测试，何时写 Autotest？
6. compile_commands 能回答什么？
7. ELF、BIN、APJ 有何区别？
8. Map 文件怎样证明功能被链接？
9. 为什么上机验证要逐级增加风险？
10. 一份可复现报告必须包含什么？
