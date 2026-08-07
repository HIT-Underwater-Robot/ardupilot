# 第 3 章：从 STM32 复位入口到 ArduSub

## 本章目标

上一章结束在 `ardusub.apj`。本章回答固件被写入 Pixhawk1 之后的第一个问题：上电后，程序怎样找到 `Sub`？

学完后应能独立画出：

```text
STM32 复位
    ↓
ChibiOS C/C++ 启动代码
    ↓
AP_HAL_MAIN_CALLBACKS 生成的 main()
    ↓
HAL_ChibiOS::run(&sub)
    ↓
AP_Vehicle::setup()
    ↓ 虚函数
Sub::init_ardupilot()
    ↓ 返回
HAL_ChibiOS 无限循环
    ↓
AP_Vehicle::loop()
```

本章只讨论“启动一次”的流程。任务如何按频率重复执行放在第 4 章。

## 1. Pixhawk 固件和普通电脑程序的启动差异

普通 Linux 程序启动前，操作系统已经准备好进程、虚拟内存、文件描述符和线程环境，然后调用程序的 `main()`。

STM32 上没有一个桌面操作系统替 ArduSub 完成全部准备。简化后需要经历：

1. CPU 从复位向量取得第一条指令；
2. 建立堆栈、时钟和中断向量；
3. 把有初值的全局数据从 Flash 复制到 RAM；
4. 把未初始化全局数据所在 BSS 清零；
5. 初始化 ChibiOS HAL 和内核；
6. 调用 C++ 全局对象构造函数；
7. 调用 `main()`。

因此在嵌入式程序中，`main()` 已经不是绝对意义上的第一段代码。

## 2. Bootloader 与应用固件的边界

Pixhawk1 上电后通常先运行 Bootloader。Bootloader 负责：

- 判断是否进入固件升级；
- 检查应用固件是否有效；
- 必要时通过 USB/串口接收新固件；
- 跳转到 ArduSub 应用固件的复位入口。

本章从 Bootloader 跳转完成之后开始。ArduSub 本身不会在 `Sub.cpp` 中重新实现 Bootloader。

构建产物中的关系：

```text
ardusub.apj
    地面站用于升级应用固件

ardusub_with_bl.hex
    特定烧录场景下同时包含 Bootloader 和应用固件
```

## 3. Cortex-M4 启动代码做了什么

Pixhawk1 使用 ARM Cortex-M4，对应启动文件：

```text
modules/ChibiOS/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S
```

`.S` 是带预处理的汇编源码。初学阶段不要求掌握 ARM 汇编，只需要认出几个关键调用。

### 3.1 `__early_init()`

启动汇编在 RAM 全部准备好之前调用：

```text
libraries/AP_HAL_ChibiOS/hwdef/common/board.c::__early_init()
```

它主要处理：

- GPIO 的极早期状态；
- STM32 时钟；
- 某些 MCU 的 Cache/TCM 设置。

此时不能依赖普通全局变量已经初始化。源码注释明确提醒：DATA/BSS 可能尚未准备完成。

### 3.2 初始化 DATA 和 BSS

理解两个区域：

```cpp
int a = 5;       // 初值保存在 Flash，启动时复制到 RAM 的 DATA
static int b;    // 启动时清零的 BSS
```

启动代码完成这些操作后，普通 C/C++ 全局数据才具备预期值。

### 3.3 `__late_init()`

随后进入：

```text
libraries/AP_HAL_ChibiOS/hwdef/common/board.c::__late_init()
```

关键调用：

```c
halInit();
chSysInit();
```

- `halInit()`：初始化 ChibiOS 的硬件抽象和已配置驱动；
- `chSysInit()`：初始化 ChibiOS 内核，使当前执行流进入 RTOS 管理。

这里还会处理看门狗复位原因、堆、USB 字符串等板级功能。

### 3.4 调用全局 C++ 构造函数

启动汇编遍历链接器生成的构造函数数组：

```text
__init_array_base__ → __init_array_end__
```

这时才会构造文件作用域的 C++ 对象，例如 HAL 驱动对象和全局 `Sub sub`。

重要规则：

- 同一个编译单元内，全局对象按定义顺序初始化；
- 不同编译单元之间的动态初始化顺序不应被普通业务代码随意假定；
- 两个对象都在 `main()` 前构造，不代表可以依赖它们跨文件的固定先后顺序。

### 3.5 调用 `main()`

构造函数阶段结束后，启动汇编执行：

```asm
bl main
```

这个 `main` 正是 ArduSub 最终通过宏生成的函数。

## 4. ArduSub 在 `main()` 前创建了什么

`ArduSub/Sub.cpp` 末尾：

```cpp
Sub *Sub::_singleton = nullptr;

Sub sub;
AP_Vehicle& vehicle = sub;

AP_HAL_MAIN_CALLBACKS(&sub);
```

逐句解释。

### 4.1 `_singleton`

```cpp
Sub *Sub::_singleton = nullptr;
```

这是 `Sub` 类静态指针的唯一存储定义。程序加载初期为空，`Sub` 构造函数随后把它设为 `this`。

### 4.2 全局车辆对象

```cpp
Sub sub;
```

真正创建唯一的 ArduSub 车辆对象。它在 `main()` 之前构造，并一直存活到系统复位。

构造过程中会连接：

- `motors`；
- `attitude_control`；
- `pos_control`；
- `wp_nav`、`loiter_nav`、`circle_nav`；
- 参数加载器和初始模式对象。

注意：构造对象不等于完成硬件初始化。构造函数主要建立对象关系，真正访问传感器、串口和参数的工作应放在后面的 `setup()/init()`。

### 4.3 公共车辆引用

```cpp
AP_Vehicle& vehicle = sub;
```

这是同一个对象的基类引用，不会复制一份车辆。公共库可以使用 `vehicle`，而不必依赖具体的 `Sub` 类型。

## 5. `HAL::Callbacks` 是什么

位置：

```text
libraries/AP_HAL/HAL.h
```

简化定义：

```cpp
struct Callbacks {
    virtual void setup() = 0;
    virtual void loop() = 0;
};
```

它只规定两个动作：

- `setup()`：启动时执行一次；
- `loop()`：启动完成后重复执行。

HAL 不需要知道 ArduSub 有哪些模式、推进器或传感器。它只保存一个 `Callbacks*`，在合适的时机调用这两个接口。

类型关系：

```text
Sub
  继承 AP_Vehicle
      继承 AP_HAL::HAL::Callbacks
```

所以 `Sub*` 可以隐式转换为 `Callbacks*`。

## 6. 宏怎样生成 `main()`

`AP_HAL_MAIN_CALLBACKS(&sub)` 定义在：

```text
libraries/AP_HAL/AP_HAL_Main.h
```

对 Pixhawk1 来说，`AP_MAIN` 默认就是 `main`。把宏简化展开后：

```cpp
extern "C" int main(int argc, char* const argv[])
{
    hal.run(argc, argv, &sub);
    return 0;
}
```

### 为什么使用 `extern "C"`

C++ 会把函数参数类型编码进符号名，称为 name mangling。启动代码只寻找标准 C 符号 `main`，因此用 `extern "C"` 禁止 C++ 名字改编。

### `hal` 从哪里来

`Sub.cpp` 中：

```cpp
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
```

Pixhawk1 编译选择 ChibiOS 后，`AP_HAL::get_HAL()` 返回：

```cpp
static HAL_ChibiOS hal_chibios;
```

所以虚函数调用 `hal.run(...)` 最终进入 `HAL_ChibiOS::run(...)`。

## 7. `HAL_ChibiOS` 如何组合硬件驱动

位置：

```text
libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp
```

文件中建立一组静态驱动对象：

```text
UARTDriver       串口
I2CDeviceManager I2C 总线设备
SPIDeviceManager SPI 总线设备
AnalogIn         模拟量
Storage          参数存储
GPIO             数字输入输出
RCInput          遥控输入
RCOutput         PWM/DShot 输出
Scheduler        ChibiOS 平台调度与线程服务
Util             板级工具和持久化故障数据
```

`HAL_ChibiOS` 构造函数把这些对象地址传给 `AP_HAL::HAL` 基类。

这就是软件层和硬件层分离的关键：

```text
车辆层只调用 hal.rcout、hal.serial、hal.scheduler
                     ↓
Pixhawk1 构建时这些指针实际指向 ChibiOS 实现
SITL 构建时则指向 SITL 实现
```

ArduSub 不需要在控制代码里写 STM32 寄存器。

## 8. `HAL_ChibiOS::run()` 做什么

`main()` 调用：

```cpp
hal.run(argc, argv, &sub);
```

由于 `run()` 是虚函数，实际执行 `HAL_ChibiOS::run()`。

主要过程：

1. 必要时尽早启动看门狗；
2. 初始化 USB 或标准输出串口；
3. 把 `&sub` 保存到 `g_callbacks`；
4. 进入 `main_loop()`。

`argc/argv` 对嵌入式固件通常没有桌面程序那么重要，但统一接口让不同 HAL 可以共用入口形式。

## 9. ChibiOS `main_loop()` 的初始化阶段

不要与 `AP_Scheduler` 的车辆任务表混淆。这里是平台线程中的外层无限循环。

### 9.1 取得当前线程并提高优先级

```cpp
daemon_task = chThdGetSelfX();
chThdSetPriority(APM_MAIN_PRIORITY);
```

当前 `main()` 已处在 ChibiOS 管理的线程中。保存线程句柄后，可以调整优先级和进行看门狗管理。

### 9.2 初始化 HAL 基础能力

根据板卡功能执行：

- 清理可能卡死的 I2C 总线；
- 初始化共享 DMA；
- 打开外设电源；
- 启动控制台串口；
- 初始化模拟量；
- 初始化 ChibiOS Scheduler 实现。

这里仍属于“平台准备”，还没有进入 ArduSub 模式或控制器。

### 9.3 降低优先级执行 `setup()`

初始化传感器可能包含等待和校准。系统临时使用较低的启动优先级：

```cpp
hal_chibios_set_priority(APM_STARTUP_PRIORITY);
g_callbacks->setup();
```

降低优先级不是降低控制精度，而是确保初始化期间后台驱动线程仍有机会运行。

由于 `g_callbacks` 指向 `sub`，虚函数分派进入：

```text
Sub 继承到的 AP_Vehicle::setup()
```

`Sub` 不能覆盖它，因为 `AP_Vehicle::setup()` 是 `final`。

## 10. `AP_Vehicle::setup()` 为什么由公共基类控制

所有车辆都必须完成参数、通信、板卡、安全和公共库初始化。如果每种车辆自行复制一份，很容易出现顺序差异或遗漏。

因此公共基类固定总体顺序，只在明确位置调用车辆虚函数。

### 阶段 A：参数默认值和持久化参数

```text
AP_Param::setup_sketch_defaults()
AP_Param::check_var_info()
load_parameters()
```

- 先建立编译时默认值；
- 校验参数表；
- 再从板载存储覆盖为用户保存值。

如果顺序反过来，默认值可能错误覆盖用户配置。

### 阶段 B：取得并初始化任务表

```text
get_scheduler_tasks(...)
    ↓ 虚函数
Sub::get_scheduler_tasks(...)
    ↓
AP_Scheduler::init(...)
```

此时只是注册任务和循环参数，不是立刻运行这些任务。

### 阶段 C：建立 GCS、串口和延时回调

主要包含：

- GCS 单例和 MAVLink system ID；
- SerialManager 与具体串口协议；
- 控制台；
- 脚本虚拟串口；
- Scheduler delay callback。

通信较早初始化，是为了后面的启动过程能够输出诊断信息。

### 阶段 D：板卡和公共传感器入口

主要包含：

- ExternalAHRS；
- `BoardConfig.init()`；
- CANManager；
- MSP；
- Logger；
- Beacon 等公共对象。

这里的 `BoardConfig` 是运行时板卡配置，不是 Waf configure。两个“configure/init”发生在完全不同阶段。

### 阶段 E：进入车辆专属初始化

核心语句：

```cpp
init_ardupilot();
```

由于它是纯虚函数，当前对象实际是 `Sub`，因此进入：

```text
ArduSub/system.cpp::Sub::init_ardupilot()
```

这就是公共车辆层进入 ArduSub 车辆层的正式接口。

### 阶段 F：公共后置初始化

`Sub::init_ardupilot()` 返回后，基类继续初始化需要更晚执行的组件，例如：

- Scripting；
- SRV_Channels；
- Gyro FFT；
- VisualOdom；
- 温度传感器公共前端；
- Fence、滤波器、RPM、Arming；
- DDS 等。

最后发送：

```text
ArduPilot Ready
```

所以 `init_ardupilot()` 返回不代表整个 `AP_Vehicle::setup()` 已完全结束。

## 11. `Sub::init_ardupilot()` 的六个阶段

位置：

```text
ArduSub/system.cpp
```

### 11.1 基础状态与供电

- Notify；
- Battery；
- Barometer 前端；
- 板卡类型相关的外部气压计总线默认值。

### 11.2 通信和驾驶输入输出

- `gcs().setup_uarts()`；
- RC 通道；
- `init_rc_in()`；
- `init_rc_out()`；
- Joystick；
- Relay/OSD。

注意 `init_rc_out()` 只是建立安全输出配置，不代表推进器已经解锁旋转。

### 11.3 独立主循环的失控保护

```cpp
hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);
```

这是平台定时保护回调。即使车辆主循环卡死，也需要有机会检测并采取保护。

### 11.4 导航传感器和深度传感器

- GPS 对象；
- Compass；
- OpticalFlow；
- Camera/Mount；
- Barometer 校准；
- 在多个 AP_Baro 实例中寻找 `BARO_TYPE_WATER`。

找到水压传感器后：

```text
设为 primary barometer
记录 depth_sensor_idx
设置 depth_sensor_present
初始化深度健康状态
降低高度量测噪声
```

未找到时退回板载气压计，并增大量测噪声，因为空气气压计不能代表水下深度。

### 11.5 任务、日志和 INS

- Mission；
- Logger 启动回调；
- `startup_INS_ground()`；
- AHRS 车辆类型设为 SUBMARINE；
- INS 预热和陀螺仪零偏校准；
- AHRS reset。

### 11.6 执行器、参数迁移和完成标志

- 启用主循环失控保护；
- 迁移旧执行器/灯光参数；
- 初始化通用 Actuators；
- 更新漏水和 Relay 引脚；
- 设置 `ap.initialised = true`。

`ap.initialised` 是 ArduSub 专属初始化完成标志，不等同于 armed。

## 12. `setup()` 返回后发生什么

控制权回到 `HAL_ChibiOS::main_loop()`：

1. 应用持久化参数；
2. 配置 Flash 保护；
3. 启动或检查 Watchdog；
4. 标记 HAL 系统初始化完成；
5. 把线程名设为固件目标名；
6. 恢复主循环高优先级；
7. 进入无限循环。

简化代码：

```cpp
while (true) {
    g_callbacks->loop();
    // 必要时让出少量 CPU 给低优先级驱动
    schedulerInstance.watchdog_pat();
}
```

`g_callbacks->loop()` 再次通过虚函数接口进入 `AP_Vehicle::loop()`。

## 13. 为什么 `main()` 永远不会正常返回

`HAL_ChibiOS::main_loop()` 是无限循环。只有这些情况会离开正常运行：

- 硬件复位；
- Watchdog 复位；
- 崩溃或异常；
- Bootloader/固件升级流程触发重启。

因此宏生成的 `main()` 中 `return 0` 只是满足函数签名，正常飞控运行不会走到那里。

## 14. 启动链中最容易混淆的三个“调度器”概念

### ChibiOS 内核调度

负责线程优先级、上下文切换、信号量和定时器。

### `AP_HAL::Scheduler`

给 ArduPilot 提供跨平台的线程、延时、定时失控保护等 HAL 接口；Pixhawk1 上由 ChibiOS 版本实现。

### `AP_Scheduler`

车辆主循环内部的任务表调度器，负责 FAST_TASK、50 Hz、10 Hz 等任务。

关系：

```text
ChibiOS 调度多个 RTOS 线程
    └─ ArduPilot 主线程
         └─ AP_Scheduler 在每个 IMU 节拍安排车辆任务
```

不要把 `hal.scheduler->init()` 和 `AP::scheduler().init(tasks...)` 当成同一个函数。

## 15. 完整启动时序图

```text
Bootloader
    ↓ 跳转到应用复位向量
Cortex-M4 crt0_v7m.S
    ├─ CPU/FPU/堆栈/中断向量
    ├─ __early_init()：GPIO、时钟
    ├─ DATA 复制、BSS 清零
    ├─ __late_init()：halInit、chSysInit
    ├─ 全局 C++ 构造函数
    │    ├─ HAL/驱动对象
    │    └─ Sub sub
    └─ main()
         ↓ AP_HAL_MAIN_CALLBACKS 展开
HAL_ChibiOS::run(..., &sub)
    ├─ 保存 g_callbacks = &sub
    └─ main_loop()
         ├─ HAL 基础驱动初始化
         ├─ 降低到启动优先级
         └─ g_callbacks->setup()
              ↓ 虚函数
         AP_Vehicle::setup()
              ├─ 参数和任务表
              ├─ GCS、串口和板卡
              ├─ 公共前置组件
              ├─ Sub::init_ardupilot()
              │    ├─ 输入输出
              │    ├─ 深度/导航传感器
              │    ├─ Mission/Logger
              │    └─ AHRS/INS
              └─ 公共后置组件、Arming、Ready
         ↓ setup 返回
         恢复主循环优先级
         while (true)
              └─ g_callbacks->loop()
                   ↓
                 AP_Vehicle::loop()
                   ↓
                 AP_Scheduler::loop()
```

## 16. 不修改代码的观察练习

### 练习一：证明 ELF 中存在真正的 `main`

```bash
arm-none-eabi-nm -C build/Pixhawk1/bin/ardusub | grep ' main$'
```

预期能看到类型为 `T` 的 `main` 符号。`T` 表示它位于代码段。

### 练习二：观察关键函数地址

```bash
arm-none-eabi-nm -C build/Pixhawk1/bin/ardusub \
  | grep -E 'HAL_ChibiOS::run|AP_Vehicle::setup|AP_Vehicle::loop|Sub::Sub'
```

它证明这些函数已经进入最终 ELF，但地址大小关系不代表运行顺序，运行顺序仍由调用关系决定。

### 练习三：手工展开宏

把 `AP_HAL_MAIN_CALLBACKS(&sub)` 按本章内容写成普通 `main()`，但不要修改仓库。回答：

- `CALLBACKS` 被替换成什么？
- `hal.run()` 为什么执行 ChibiOS 实现？
- 为什么 `&sub` 满足 `Callbacks*` 类型？

### 练习四：给启动阶段分类

把 `AP_Vehicle::setup()` 中每个 `init()` 放入下面一类：

```text
参数
调度
通信
板卡/HAL
车辆专属
公共后置功能
安全与解锁
```

不需要理解每个库内部实现，先理解为什么它必须出现在车辆初始化之前或之后。

## 本章验收问题

1. 为什么 STM32 上的 `main()` 不是第一段执行代码？
2. `__early_init()` 为什么不能依赖普通全局变量？
3. DATA 和 BSS 有什么区别？
4. ChibiOS 在全局 C++ 构造函数之前还是之后初始化？
5. 为什么不能随意依赖不同 `.cpp` 全局对象的构造先后？
6. `Sub*` 为什么能传给 `Callbacks*`？
7. `extern "C"` 对 `main` 有什么作用？
8. `HAL_ChibiOS` 怎样把抽象 HAL 接口连接到具体驱动？
9. `AP_Vehicle::setup()` 为什么是 `final`？
10. `Sub::init_ardupilot()` 前后分别还有哪些公共初始化？
11. `ap.initialised`、HAL system initialized 和 armed 是否是同一状态？
12. ChibiOS 调度器、AP_HAL Scheduler 和 AP_Scheduler 有什么区别？

能够不看文档画出完整启动时序后，再进入第 4 章：实时调度器和主循环。
