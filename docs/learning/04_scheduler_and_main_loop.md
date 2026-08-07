# 第 4 章：实时调度器和主循环

这一章回答一个最关键的问题：

> ArduSub 没有在一个巨大的 while 循环里依次写完所有功能，姿态控制、通信、传感器和日志为什么仍然能够按照不同频率运行？

先给出结论：

~~~text
ChibiOS 主线程
    ↓
HAL_ChibiOS::main_loop()
    ↓ 反复调用
AP_Vehicle::loop()
    ↓ 每轮调用一次
AP_Scheduler::loop()
    ├─ 等待新的 IMU 样本
    ├─ 计算本轮时间预算
    ├─ FAST_TASK：每轮必须运行
    └─ 普通任务：到期且预算足够才运行
~~~

ArduPilot 调度器不是桌面操作系统那种“每个功能一个线程”的模型。车辆控制主线中的大多数任务都在同一个主循环线程里顺序执行，因此我们阅读控制数据流时，可以按照任务表和函数调用顺序跟踪，而不必先处理大量线程并发问题。

## 1. 本章只读这些文件

第一遍按下面顺序阅读：

1. **ArduSub/Sub.cpp**：ArduSub 自己的任务表；
2. **libraries/AP_Vehicle/AP_Vehicle.cpp**：主循环入口和公共任务表；
3. **libraries/AP_Scheduler/AP_Scheduler.h**：任务的数据结构和宏；
4. **libraries/AP_Scheduler/AP_Scheduler.cpp**：频率、优先级和预算算法；
5. **libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp**：最外层无限循环。

此时不要展开每个任务内部的控制算法。我们的目标只是弄清楚“谁在什么时间调用谁”。

## 2. 先区分两层调度

### 2.1 ChibiOS 线程调度

ChibiOS 是实时操作系统。它管理主线程、驱动线程或中断、定时器、锁、信号量、线程优先级，以及 SD 卡和 CAN 等后台活动。这是操作系统级调度。

### 2.2 AP_Scheduler 任务调度

AP_Scheduler 管理的是 ArduPilot 自己定义的函数，例如读取惯性传感器、运行姿态控制器、更新模式、收发 MAVLink、检查失控保护和写日志。

这些 Task 不是 ChibiOS 线程。Task 本质上只是：

~~~cpp
struct Task {
    task_fn_t function;
    const char *name;
    float rate_hz;
    uint16_t max_time_micros;
    uint8_t priority;
};
~~~

其中 function 是一个可以稍后调用的函数对象。调度器遍历任务表，并在合适时执行 task.function()。

> AP_Scheduler 的“任务”通常只是同一线程内被安排执行的 C++ 函数，不等于 RTOS 线程。

## 3. 任务表是怎样写出来的

ArduSub 在 **Sub.cpp** 中定义：

~~~cpp
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    FAST_TASK_CLASS(AP_InertialSensor, &sub.ins, update),
    FAST_TASK(run_rate_controller),
    FAST_TASK(motors_output),
    ...
    SCHED_TASK(fifty_hz_loop, 50, 75, 3),
    ...
};
~~~

宏只是帮助填写 Task 结构体，并不是一种新的 C++ 调度语法。

### 3.1 SCHED_TASK 的四个信息

以 SCHED_TASK(update_altitude, 10, 100, 18) 为例：

| 字段 | 值 | 含义 |
|---|---:|---|
| function | update_altitude | 到期时调用这个成员函数 |
| rate_hz | 10 | 目标频率是每秒 10 次 |
| max_time_micros | 100 | 预计最多使用 100 微秒 |
| priority | 18 | 数字越小越先考虑 |

max_time_micros 不是硬件定时器，也不会在 100 微秒时强制终止函数。它用于调度前判断预算是否容得下任务，并在调度后检查实际耗时是否超出预计值。

### 3.2 SCHED_TASK_CLASS 为什么多两个参数

~~~cpp
SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200, 6)
~~~

它明确给出类 AP_GPS、对象地址 &sub.gps 和成员函数 AP_GPS::update。普通 SCHED_TASK 已经通过 ArduSub 自己定义的宏，默认补成 Sub 类和 &sub 对象。

### 3.3 FAST_TASK 的特殊之处

FAST_TASK 的 rate_hz 和 max_time_micros 都填写为零，并使用特殊的快速任务优先级。

它不是“不需要时间”，而是每一个主循环节拍都必须执行；普通任务预算耗尽后，调度器仍会继续寻找任务表中的快速任务。

ArduSub 的快速链路大致是：

~~~text
INS update
    ↓
run_rate_controller
    ↓
motors_output
    ↓
read_AHRS
    ↓
read_inertia
    ↓
update_flight_mode
~~~

阅读时要注意当前源码的实际顺序，不要把任务表当作可以任意交换的功能清单。

## 4. 任务表什么时候交给调度器

~~~text
AP_Vehicle::setup()
    ↓
Sub::get_scheduler_tasks(...)
    ↓
取得 Sub::scheduler_tasks 和任务数量
    ↓
AP_Scheduler::init(...)
    ├─ 保存车辆任务表
    └─ 取得 AP_Vehicle 公共任务表
~~~

最终参与调度的有 ArduSub 车辆任务表和 AP_Vehicle 公共任务表。

调度器没有把两张表复制成第三张大表，而是在 run() 中用两个下标按优先级合并遍历。如果两个任务优先级相同，当前代码先选择车辆任务。两张原始任务表内部都必须保持优先级数字不下降，否则 init() 会记录内部错误。

## 5. 每一轮为什么先等 IMU

AP_Scheduler::loop() 的第一件核心工作是 AP::ins().wait_for_sample()。这让控制主循环与新的惯性测量同步。

如果主循环目标是 400 Hz，那么理想周期是：

~~~text
1 秒 ÷ 400 = 0.0025 秒 = 2500 微秒
~~~

每轮大致过程如下：

~~~text
等待新 IMU 样本
    ↓
记录本轮开始时间
    ↓
tick 计数加一
    ↓
计算 2500 微秒中还剩多少
    ↓
运行快速任务和到期普通任务
    ↓
回到下一次 wait_for_sample()
~~~

这比单纯用 delay 周期唤醒更适合控制系统，因为控制器使用的是刚到达的 IMU 数据。

## 6. 10 Hz 和 50 Hz 怎样从 400 Hz 得到

普通任务的间隔近似为：

~~~text
interval_ticks = 主循环频率 ÷ 任务目标频率
~~~

在 400 Hz 主循环下：

| 任务频率 | interval_ticks | 理想执行节奏 |
|---:|---:|---|
| 400 Hz | 1 | 每轮 |
| 200 Hz | 2 | 每 2 轮 |
| 50 Hz | 8 | 每 8 轮 |
| 20 Hz | 20 | 每 20 轮 |
| 10 Hz | 40 | 每 40 轮 |
| 1 Hz | 400 | 每 400 轮 |
| 0.1 Hz | 4000 | 每 4000 轮 |

代码通过当前 tick 减去上次运行 tick 判断任务是否到期。若 dt 小于 interval_ticks，本轮跳过；否则任务已经到期。

对于不能整除的频率，当前实现会把结果转换成整数 tick，因此实际频率会有小的量化误差。例如 400 除以 3 得到约 133 个 tick。

## 7. 到期不代表本轮一定执行

普通任务到期后，还要比较任务声明的 max_time_micros 与本轮剩余 time_available。

如果预算不足，调度器不会把任务拆成两半，也不会立即阻塞等待，而是跳过它，继续看看后面是否有更短的任务能放入剩余预算。

### 7.1 slipped

如果任务距离上次运行已经超过两个目标周期，性能统计会把它记为 slipped。

### 7.2 overrun

如果函数实际耗时超过声明的 max_time_micros，会记为 overrun。函数已经执行完毕；overrun 是检测和记录，不是提前杀死函数。

### 7.3 extra_loop_us

若任务长时间得不到运行机会，调度器会逐步增加额外预算，当前代码最高增加到 5000 微秒。

额外预算能让积压任务获得机会，但代价是主循环有效频率可能降低。它是一种过载退让机制，不是凭空增加 CPU 性能。

## 8. 一轮主循环的简化示例

~~~text
新 IMU 样本到达
    ↓
FAST: INS update              必须运行
    ↓
FAST: rate controller         必须运行
    ↓
FAST: motors output           必须运行
    ↓
FAST: AHRS / mode             必须运行
    ↓
50 Hz 任务是否到期？           每 8 轮一次
    ↓
10 Hz 任务是否到期？           每 40 轮一次
    ↓
GCS receive/send 是否到期？    目标 400 Hz，但仍受预算判断
    ↓
记录剩余时间和任务性能
~~~

“目标 400 Hz”不自动等于无论负载多高都稳定执行 400 次。FAST_TASK 才具有每轮调度语义；普通任务仍要通过预算判断。

## 9. AP_Vehicle::loop() 还做了什么

第一次任务调度完成后，它调用 BoardConfig.init_safety() 初始化硬件 Safety。这个动作故意延后，先让舵机和推进器输出获得正确初值，以降低输出初始化瞬间跳变的风险。

它还检查 AP_InternalError 的变化，并把新错误写入日志、发送到地面站。

## 10. 调度器没有自动保证什么

调度器只决定哪个函数何时获得执行机会，它不会自动保证：

- 函数内部算法正确；
- 共享数据一定没有竞争；
- 传感器时间戳正确；
- 控制器单位和坐标系正确；
- 任务一定能在预算内完成；
- 电机输出一定安全。

这些需要各模块设计、日志、SITL 和硬件测试共同保证。

## 11. 阅读周期任务的固定方法

1. 在任务表确认它是 FAST_TASK 还是普通任务；
2. 记录目标频率、预算和优先级；
3. 确认属于 Sub 表还是 AP_Vehicle 公共表；
4. 找函数定义；
5. 列出函数读取的状态；
6. 列出函数写入的状态或调用的下一层；
7. 查看编译开关；
8. 查看日志或健康状态。

例如 update_batt_compass：

~~~text
任务表：10 Hz，120 us，priority 12
    ↓
battery.read()
    ↓
若罗盘可用
    ├─ 写入当前 throttle 用于电机干扰补偿
    └─ compass.read()
~~~

## 12. 本章只读实验

### 实验 A：手工算调度间隔

在 400 Hz 主循环下，计算 fifty_hz_loop、update_altitude、three_hz_loop 和 one_hz_loop 的间隔，再对照 AP_Scheduler::run()。

### 实验 B：标记快速控制链

只在纸上画：

~~~text
wait_for_sample
→ INS update
→ rate controller
→ motors_output
→ AHRS
→ current mode
~~~

不要修改顺序。下一阶段会结合上一轮目标、当前轮反馈和流水线时序解释这个排列。

### 实验 C：观察性能

运行时 PERF/PM 日志可以观察主循环性能。不要用 printf 插入高频任务，它本身会严重改变时序。

## 13. 本章验收问题

1. AP_Scheduler Task 与 ChibiOS 线程有什么区别？
2. 400 Hz 主循环中 50 Hz 任务理论上每几轮到期？
3. FAST_TASK 为什么不等于普通 400 Hz SCHED_TASK？
4. max_time_micros 会不会强行终止函数？
5. 到期任务为什么可能本轮不运行？
6. priority 数字越大还是越小越先考虑？
7. 两张任务表如何一起运行？
8. 为什么以 IMU 样本作为节拍？
9. slipped 和 overrun 分别说明什么？
10. extra_loop_us 的代价是什么？
