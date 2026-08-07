# ArduSub 从 C++ 入门到独立开发：学习地图

这套材料面向只掌握 C++ 基础语法、但不了解大型嵌入式工程、实时调度和飞控架构的学习者。

需要由新的智能体或 AI 编程工具接着开发时，请先阅读 [AI 接手开发指南与可复制提示词](AI_DEVELOPMENT_GUIDE.md)。它用于防止接手者丢失本仓库的教学目标、软硬件边界和“小步验证”工作方式。

目标不是让你记住每个文件，而是最终具备以下能力：

1. 能解释源码如何被编译成 Pixhawk 固件；
2. 能从 `main()` 一直跟踪到某个周期任务；
3. 能跟踪一条 MAVLink 消息如何进入车辆控制逻辑；
4. 能跟踪一个传感器数据如何进入状态估计；
5. 能跟踪一个运动目标如何变成每个推进器的 PWM/DShot 输出；
6. 能独立新增传感器、控制模式和 MAVLink 消息；
7. 能为新板卡建立 hwdef 并验证固件；
8. 能判断哪些功能可以裁剪，以及如何证明裁剪没有破坏系统。

## 为什么不能只靠源码行注释

大型系统有三种不同层次的知识：

- **概念知识**：交叉编译、静态库、回调、调度器、坐标系、状态估计；
- **系统关系**：一个文件为什么调用另一个文件，数据从哪里来、到哪里去；
- **局部代码**：某个类、函数和变量具体负责什么。

源码注释适合第三层，但无法单独讲清前两层。因此本仓库采用三层教学结构：

```text
docs/learning/*.md
    讲概念、完整调用链、图示、练习和检查问题
        ↓
核心源码中的中文注释
    说明当前文件在调用链中的位置、输入、输出和约束
        ↓
SITL / Pixhawk1 小实验
    用编译结果、日志、MAVLink 或硬件输出验证理解
```

## 学习时必须遵守的规则

- 一次只跟踪一条主线，不要同时阅读整个 `libraries/`。
- 遇到陌生类时，先判断它是“车辆层、公共库、HAL 还是第三方模块”。
- 先找对象由谁创建，再找函数由谁调用，最后才阅读函数内部算法。
- 每学完一章，必须能用自己的话画出调用链。
- 修改控制、解锁、失控保护或电机输出后，必须先编译和仿真。
- 不直接修改 `build/` 生成文件，也不直接修改 `modules/` 子模块。

## 完整课程结构

### 阶段一：具备阅读大型 C++ 工程的能力

#### 第 1 章：ArduSub 阅读所需的 C++

学习内容：

- `.h` 与 `.cpp`、声明与定义、编译单元；
- 类、继承、组合、访问控制；
- `virtual`、`override`、`final`；
- 指针、引用、`const` 和生命周期；
- 构造函数初始化列表；
- 静态对象、单例和全局入口；
- 宏、条件编译、回调和函数对象；
- `enum class`、位域和固定宽度整数。

对应源码：

- `ArduSub/Sub.h`
- `ArduSub/Sub.cpp`
- `libraries/AP_Vehicle/AP_Vehicle.h`
- `libraries/AP_HAL/AP_HAL_Main.h`

完成标准：能解释 `Sub sub` 是何时创建的，以及为什么 `Sub` 可以被 HAL 当作回调对象。

#### 第 2 章：源码如何变成 Pixhawk 固件

学习内容：

- 预处理、编译、静态库、链接、ELF、BIN、APJ；
- 原生编译与交叉编译；
- Waf 的 configure、build、target、group 和增量构建；
- hwdef、ChibiOS、MAVLink 代码生成；
- 如何使用构建目录反查依赖。

对应源码：

- `waf`
- `wscript`
- `ArduSub/wscript`
- `Tools/ardupilotwaf/boards.py`
- `Tools/ardupilotwaf/chibios.py`
- `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/hwdef.dat`

完成标准：能从 `./waf sub` 解释到 `ardusub.apj` 的每一步。

### 阶段二：掌握系统启动和实时运行骨架

#### [第 3 章：从平台 `main()` 到 ArduSub](03_startup_to_ardusub.md)

学习内容：

- `AP_HAL_MAIN_CALLBACKS(&sub)` 的展开逻辑；
- `HAL_ChibiOS::run()` 如何接管平台入口；
- 为什么车辆层不直接依赖 STM32 API；
- `AP_Vehicle::setup()` 与 `Sub::init_ardupilot()` 的职责边界。

对应源码：

- `libraries/AP_HAL/AP_HAL_Main.h`
- `libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp`
- `libraries/AP_Vehicle/AP_Vehicle.cpp`
- `ArduSub/system.cpp`

完成标准：能够画出启动时的函数调用顺序。

#### [第 4 章：实时调度器和主循环](04_scheduler_and_main_loop.md)

学习内容：

- 为什么主循环以 IMU 样本为节拍；
- FAST_TASK 与普通任务；
- 任务频率、优先级和预计执行时间；
- 时间预算不足时发生什么；
- ChibiOS 线程与 ArduPilot 任务的区别。

对应源码：

- `ArduSub/Sub.cpp::scheduler_tasks[]`
- `libraries/AP_Scheduler/AP_Scheduler.h`
- `libraries/AP_Scheduler/AP_Scheduler.cpp`

完成标准：能说明 400 Hz 主循环中，10 Hz 和 50 Hz 任务如何被调度。

#### [第 5 章：参数和编译开关](05_parameters_and_build_options.md)

学习内容：

- `AP_Param` 如何登记、保存和加载参数；
- `g`、`g2`、`var_info[]` 的关系；
- 为什么参数索引不能随意修改；
- 运行参数与 `#if AP_*_ENABLED` 编译开关的区别；
- QGroundControl 如何获得参数信息。

对应源码：

- `ArduSub/Parameters.h`
- `ArduSub/Parameters.cpp`
- `libraries/AP_Param/`
- `Tools/scripts/build_options.py`

完成标准：能安全新增一个参数，并说明它何时进入 Flash、何时生效。

#### [第 6 章：HAL 与 Pixhawk1 硬件映射](06_hal_and_pixhawk1.md)

学习内容：

- HAL 接口与 ChibiOS 实现为什么分开；
- UART、I2C、SPI、CAN、GPIO、PWM 的对象关系；
- `hwdef.dat` 如何定义 MCU、引脚、总线和设备；
- `SERIAL0`、`SERIAL1` 与实际接口的关系；
- Pixhawk1、Pixhawk1-1M 和未来 Pixhawk6 的差别位于哪里。

对应源码：

- `libraries/AP_HAL/`
- `libraries/AP_HAL_ChibiOS/`
- `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/`
- `libraries/AP_SerialManager/`

完成标准：能从 `SERIALx_PROTOCOL` 参数定位到具体 UART 驱动。

### 阶段三：掌握信息进入飞控后的数据流

#### [第 7 章：MAVLink 接收和发送](07_mavlink_runtime.md)

学习内容：

- XML、mavgen 和生成头文件的关系；
- UART 字节流如何被解析成 MAVLink 消息；
- 消息路由、公共处理和 ArduSub 专属处理；
- 消息发送频率和 stream；
- COMMAND、SETPOINT、ODOMETRY 和状态消息的差别。

对应源码：

- `modules/mavlink/message_definitions/`
- `libraries/GCS_MAVLink/`
- `ArduSub/GCS_MAVLink_Sub.h`
- `ArduSub/GCS_MAVLink_Sub.cpp`
- `ArduSub/GCS_Sub.cpp`

完成标准：能从 TELEM2 收到的一帧消息跟踪到某个车辆处理函数。

#### [第 8 章：传感器前端、后端和自动探测](08_sensor_frontend_backend.md)

学习内容：

- 为什么同一种传感器有许多型号驱动；
- Frontend/Backend 架构；
- 板载探测、外部总线探测和参数选择；
- update、health、instance、primary 的含义；
- MS5837 如何作为 `AP_Baro::BARO_TYPE_WATER` 进入 ArduSub。

对应源码：

- `ArduSub/sensors.cpp`
- `ArduSub/system.cpp`
- `libraries/AP_Baro/`
- `libraries/AP_InertialSensor/`
- `libraries/AP_Compass/`

完成标准：能设计一个 UART 传感器 Backend，并说明由谁创建和更新。

#### [第 9 章：坐标系、单位和状态估计](09_frames_and_state_estimation.md)

学习内容：

- 机体系、NED/NEU、地理坐标和旋转；
- 米、厘米、弧度、角度以及变量后缀；
- INS、AHRS 和 EKF 各自解决什么问题；
- 传感器原始值、融合状态和控制反馈的区别；
- DVL、USBL、视觉里程计和 MAVLink ODOMETRY 的接入位置。

对应源码：

- `ArduSub/inertia.cpp`
- `libraries/AP_AHRS/`
- `libraries/AP_NavEKF3/`
- `libraries/AP_VisualOdom/`
- `libraries/AP_ExternalAHRS/`

完成标准：能说明外部里程计的坐标、时间戳和协方差为何会影响 EKF。

### 阶段四：掌握运动控制和执行器输出

#### [第 10 章：模式框架](10_mode_framework.md)

学习内容：

- `Mode` 基类和各模式派生类；
- 模式编号、切换检查、`enter()`、`run()`、`exit()`；
- Manual、Stabilize、Acro、AltHold、PosHold、Guided、Auto；
- 模式与控制器的边界。

对应源码：

- `ArduSub/mode.h`
- `ArduSub/mode.cpp`
- `ArduSub/mode_*.cpp`

完成标准：能新增一个不改变安全逻辑的教学模式，并在 SITL 中切换。

#### [第 11 章：从目标到姿态和位置控制](11_control_cascade.md)

学习内容：

- 位置环、速度环、加速度环、姿态环和角速度环；
- PID、前馈、限幅和滤波；
- 不同模式如何给控制器设置目标；
- 控制器为何不直接操作 PWM。

对应源码：

- `ArduSub/Attitude.cpp`
- `libraries/AC_AttitudeControl/`
- `libraries/AC_PID/`
- `libraries/AC_WPNav/`

完成标准：能从 Guided 位置目标跟踪到六自由度归一化控制量。

#### [第 12 章：六自由度推进器混控和硬件输出](12_thruster_output.md)

学习内容：

- roll、pitch、yaw、throttle、forward、lateral 六个控制轴；
- 推进器方向矩阵和归一化；
- 饱和、限幅和解锁状态；
- `AP_Motors6DOF → SRV_Channels → AP_HAL::RCOutput`；
- PWM、DShot 和 IO MCU 的职责。

对应源码：

- `ArduSub/motors.cpp`
- `ArduSub/actuators.cpp`
- `libraries/AP_Motors/AP_Motors6DOF.*`
- `libraries/SRV_Channel/`
- `libraries/AP_HAL_ChibiOS/RCOutput.*`

完成标准：给定一个前进和偏航指令，能解释每个推进器输出为什么是正、负或零。

### 阶段五：任务、安全和工程验证

#### [第 13 章：Guided、Auto 与任务系统](13_guided_auto_mission.md)

学习内容：

- Guided 连续目标与 Auto 任务列表的区别；
- Mission item 的加载、启动、执行和完成判断；
- 树莓派/香橙派规划与 Pixhawk 任务执行如何并存；
- 无 GPS 和水面 GPS 场景的模式约束。

对应源码：

- `ArduSub/commands.cpp`
- `ArduSub/commands_logic.cpp`
- `ArduSub/mode_guided.cpp`
- `ArduSub/mode_auto.cpp`
- `libraries/AP_Mission/`

完成标准：能选择 Guided 或 Auto，并说明选择依据和失联后的行为。

#### [第 14 章：解锁、失控保护和健康状态](14_arming_and_failsafe.md)

学习内容：

- 解锁检查为何分层；
- pilot、GCS、EKF、电池、漏水、深度计和主循环失控保护；
- failsafe 检测与 failsafe 动作的区别；
- Safety Switch、armed 和 interlock 的区别。

对应源码：

- `ArduSub/AP_Arming_Sub.*`
- `ArduSub/failsafe.cpp`
- `ArduSub/system.cpp`
- `libraries/AP_Arming/`
- `libraries/AP_BattMonitor/`

完成标准：能为一种新传感器定义健康状态和合理的故障响应，而不绕过现有保护。

#### [第 15 章：日志、调试和测试](15_logging_debug_testing.md)

学习内容：

- `GCS_SEND_TEXT`、MAVLink Inspector 和 DataFlash 日志；
- `AP_Logger` 消息结构；
- SITL、MAVProxy、QGroundControl 的分工；
- 单元测试、autotest、台架测试和水池测试；
- ELF、Linker.map、反汇编和崩溃信息。

对应源码：

- `ArduSub/Log.cpp`
- `libraries/AP_Logger/`
- `Tools/autotest/`
- `build/<board>/Linker.map`
- `build/<board>/compile_commands.json`

完成标准：能够为一次模式切换或传感器异常提供可复现的测试和日志证据。

### 阶段六：完成四类二次开发

#### [第 16 章：新增传感器](16_add_sensor.md)

完成两条路径：

1. 传感器直接连接 Pixhawk UART/I2C/SPI，编写 Backend；
2. 传感器连接香橙派，由 MAVLink 写入 Pixhawk。

必须同时完成：参数、健康检查、日志、SITL 模拟和失效行为。

#### [第 17 章：新增控制模式](17_add_mode.md)

完成：模式编号、类、进入检查、运行逻辑、退出处理、参数、MAVLink/QGC 切换和 SITL 测试。

#### [第 18 章：新增 MAVLink 消息](18_add_mavlink_message.md)

完成：XML 定义、代码生成、发送端、Pixhawk 接收处理、转发、QGC/伴随计算机解析和兼容策略。

#### [第 19 章：部署到新板卡](19_new_board.md)

完成：MCU 与 Flash、时钟、引脚、UART/I2C/SPI/CAN、传感器探测、Bootloader、板卡 ID、编译和烧录验证。

#### [第 20 章：安全裁剪和长期维护](20_safe_trimming_maintenance.md)

完成：依赖分析、编译开关、固件尺寸对比、上游同步、回归编译和测试矩阵。只有在能证明某功能不在编译/运行路径中时才删除。

## 推荐的实际学习节奏

每一章按下面五步进行：

1. **概念课**：先用普通 C++/嵌入式语言建立心智模型；
2. **文件地图**：只列本章需要打开的文件；
3. **调用链**：从入口逐函数向下，不跳跃；
4. **小实验**：改一个安全、可观察、可恢复的点；
5. **验收问题**：不用看文档也能解释并画图，才进入下一章。

## 当前进度

- [x] 建立课程地图
- [x] 第 1 章：ArduSub 阅读所需的 C++
- [x] 第 2 章：源码如何变成 Pixhawk 固件
- [x] 第 3 章：从平台 `main()` 到 ArduSub
- [x] 第 4 章：实时调度器和主循环
- [x] 第 5 章：参数和编译开关
- [x] 第 6 章：HAL 与 Pixhawk1 硬件映射
- [x] 第 7 章：MAVLink 接收和发送
- [x] 第 8 章：传感器前端、后端和自动探测
- [x] 第 9 章：坐标系、单位和状态估计
- [x] 第 10 章：模式框架
- [x] 第 11 章：从目标到姿态和位置控制
- [x] 第 12 章：六自由度推进器混控和硬件输出
- [x] 第 13 章：Guided、Auto 与任务系统
- [x] 第 14 章：解锁、失控保护和健康状态
- [x] 第 15 章：日志、调试和测试
- [x] 第 16 章：新增传感器的两条路径
- [x] 第 17 章：新增控制模式
- [x] 第 18 章：新增 MAVLink 消息
- [x] 第 19 章：部署到新板卡
- [x] 第 20 章：安全裁剪和长期维护

不要因为列表很长而一次打开所有文件。前四章完成后，你就已经具备独立沿调用关系阅读其余模块的能力；后续课程是在这个骨架上逐层增加传感器、估计、控制和通信知识。
