# 第 1 章：阅读 ArduSub 必须掌握的 C++

## 本章目标

学完后，你应该能够读懂下面这条关系，而不是把它当作一串陌生语法：

```text
Sub 继承 AP_Vehicle
    ↓
AP_Vehicle 实现 HAL::Callbacks
    ↓
全局对象 Sub sub 在启动前构造
    ↓
AP_HAL_MAIN_CALLBACKS(&sub) 生成 main()
    ↓
HAL 调用 sub.setup() / sub.loop()
    ↓
虚函数把公共流程重新分派到 ArduSub 专属实现
```

本章不要求掌握高级模板元编程，但必须理解类、对象、继承、指针、引用、虚函数、宏和对象生命周期。

## 1. `.h` 和 `.cpp` 分别是什么

初学 C++ 时经常把一个程序写在单个 `.cpp` 中。ArduSub 必须拆成许多文件，因为不同模块需要分别编译并复用。

### 头文件 `.h`

头文件主要告诉其他编译单元：

- 某个类叫什么；
- 它有哪些成员和函数；
- 函数参数与返回值是什么；
- 其他代码怎样使用它。

例如 `ArduSub/Sub.h` 声明 `class Sub`，但绝大多数函数的具体代码位于不同 `.cpp` 中。

### 实现文件 `.cpp`

实现文件提供函数定义。例如：

```cpp
void Sub::run_rate_controller()
{
    // 函数实现
}
```

`Sub::` 表示这个函数属于 `Sub` 类。

### 编译单元

编译器不会一次理解整个仓库。每个 `.cpp` 与它包含的头文件经过预处理后，形成一个编译单元，再独立生成 `.o` 目标文件：

```text
Sub.cpp + include 的头文件 → Sub.cpp.o
system.cpp + include 的头文件 → system.cpp.o
motors.cpp + include 的头文件 → motors.cpp.o
```

最后链接器把大量 `.o` 和 `.a` 静态库合并为一个 ELF 程序。

这解释了为什么：

- 头文件改动可能导致许多 `.cpp` 重新编译；
- 只修改一个 `.cpp` 通常只重新编译少量文件；
- “文件存在于仓库”不代表它一定被链接进固件。

## 2. `#include` 不是运行时调用

```cpp
#include <AP_Scheduler/AP_Scheduler.h>
```

它发生在编译前，作用近似为“把头文件内容提供给当前编译单元”。它不是程序运行时调用 `AP_Scheduler`。

看到 `#include` 时先问：

1. 当前文件需要这个类型的完整定义，还是只需要声明？
2. 这个库是否真的由 `wscript` 加入构建？
3. 它是否被条件编译宏包围？

## 3. 类的继承与组合

ArduSub 同时大量使用继承和组合。

### 继承：表示“它是一种什么”

```cpp
class Sub : public AP_Vehicle
```

意思是：`Sub` 是一种 `AP_Vehicle`，因此拥有 `AP_Vehicle` 的公共接口，并可以交给需要 `AP_Vehicle` 的代码。

继续向上看：

```cpp
class AP_Vehicle : public AP_HAL::HAL::Callbacks
```

因此 `Sub` 最终也是一种 `HAL::Callbacks`。这就是 `&sub` 能传给 HAL 的类型原因。

### 组合：表示“它拥有什么”

`Sub.h` 中包含：

```cpp
AP_Motors6DOF motors;
AC_AttitudeControl_Sub attitude_control;
AC_PosControl pos_control;
AC_WPNav wp_nav;
GCS_Sub _gcs;
```

意思是一个 `Sub` 对象内部拥有推进器、姿态控制器、位置控制器、导航器和通信对象。

阅读原则：

- 看到 `class A : public B`，沿继承关系寻找公共生命周期；
- 看到类成员 `C object;`，沿组合关系寻找数据和功能由谁提供。

## 4. `public`、`protected` 和 `private`

- `public`：类外部可以调用；
- `protected`：当前类和派生类可以访问；
- `private`：只有当前类及其友元可以访问。

`Sub` 的大部分内部对象是 `private`，因为外部代码不应任意修改飞控状态。

`friend class GCS_MAVLINK_Sub;` 表示指定类可以访问 `Sub` 的私有成员。它不是继承，也不会创建对象，只是授予访问权限。

看到 `friend` 时要问：为什么这个模块需要直接进入车辆内部？这种权限是否扩大了耦合？

## 5. 虚函数、`override` 和 `final`

### 虚函数解决什么问题

公共框架知道“车辆需要初始化”，但不知道 ArduSub、Copter 或 Plane 分别如何初始化。因此基类声明接口，派生类提供实现：

```cpp
virtual void init_ardupilot() = 0;
```

`= 0` 表示纯虚函数：`AP_Vehicle` 只规定必须存在，不提供可直接使用的实现。

在 `Sub.h` 中：

```cpp
void init_ardupilot() override;
```

`override` 要求编译器检查它确实覆盖了基类虚函数。函数名或参数写错时会直接编译失败。

### `final` 的作用

`AP_Vehicle::setup()` 和 `loop()` 标记为 `final`，表示 `Sub` 不能重新定义公共生命周期。ArduSub 只能通过规定的扩展点加入自己的逻辑：

- `init_ardupilot()`；
- `get_scheduler_tasks()`；
- `set_mode()` 等车辆接口。

这个设计避免不同车辆绕过参数、调度器和安全初始化流程。

## 6. 对象、指针和引用

### 对象

```cpp
Sub sub;
```

这里真正创建了一个 `Sub` 对象，并占用静态存储空间。

### 指针

```cpp
Sub *Sub::_singleton = nullptr;
```

指针保存对象地址。`nullptr` 表示当前不指向对象。

构造函数中：

```cpp
_singleton = this;
```

`this` 是当前对象地址，因此单例指针开始指向全局 `sub`。

### 引用

```cpp
AP_Vehicle& vehicle = sub;
```

引用是已有对象的别名，不会复制 `Sub`。这里把同一个 `sub` 以基类 `AP_Vehicle` 的身份暴露给公共库。

### 指针的引用

调度器接口中有一个初学者容易混淆的类型：

```cpp
const AP_Scheduler::Task *&tasks
```

从变量名向外读：

1. `tasks` 是引用 `&`；
2. 它引用一个指针 `*`；
3. 指针指向只读的 `Task`。

为什么不用普通指针？因为函数需要修改调用者持有的指针，让它指向 `Sub::scheduler_tasks[]`，但不允许通过该指针修改任务表内容。

## 7. `const` 应该怎样读

常见形式：

```cpp
const AP_Scheduler::Task *tasks;
```

表示不能通过 `tasks` 修改它指向的 `Task`。

```cpp
float Sub::get_alt_msl() const;
```

函数末尾的 `const` 表示该成员函数承诺不修改 `Sub` 的普通成员状态。

```cpp
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
```

表示 `hal` 是 HAL 单例的只读引用，不会复制 HAL 对象。

## 8. 构造函数初始化列表

`Sub::Sub()` 中冒号之后的部分：

```cpp
Sub::Sub()
    : motors(MAIN_LOOP_RATE),
      attitude_control(ahrs_view, motors),
      pos_control(ahrs_view, motors, attitude_control)
```

不是普通赋值，而是在成员对象创建时调用它们的构造函数。

依赖关系从这里非常清楚：

- `motors` 需要主循环频率；
- `attitude_control` 需要状态估计视图和推进器对象；
- `pos_control` 需要状态估计、推进器和姿态控制器。

注意：成员真实初始化顺序由它们在 `Sub.h` 中的声明顺序决定，不是初始化列表的书写顺序。这是大型 C++ 工程中常见的隐患。

## 9. 静态存储期与启动顺序

全局对象：

```cpp
Sub sub;
```

在进入 `main()` 前构造，在整个固件运行期间一直存在。

简化后的顺序：

```text
ChibiOS 完成 DATA/BSS 和内核初始化
    ↓
调用各编译单元的全局 C++ 构造函数
    ├─ 构造 HAL/驱动对象
    └─ 构造全局 Sub sub
       （不要依赖不同 .cpp 之间可移植的固定构造先后）
    ↓
进入 AP_HAL_MAIN_CALLBACKS 生成的 main()
    ↓
hal.run(..., &sub)
```

飞控大量使用静态对象，是为了避免运行时随意分配内存并明确对象生命周期。同一个编译单元内按定义顺序构造，但不同编译单元间的动态初始化顺序不应被普通业务代码假定，因此许多库采用 `get_singleton()` 延后获取对象地址。

## 10. 单例模式

典型结构：

```cpp
static Sub *_singleton;
```

构造函数检查：

```cpp
if (_singleton != nullptr) {
    AP_HAL::panic("Can only be one Sub");
}
```

这保证系统里只有一个顶层车辆对象。

常见访问方式：

```cpp
AP::scheduler()
AP::ins()
AP::baro()
AP::vehicle()
```

这些函数通常返回对应单例的引用。看到 `AP::xxx()` 时，不要把它理解为创建新对象，而应寻找该库的 `get_singleton()` 和对象持有者。

## 11. `static` 的三种常见含义

### 函数内部静态变量

只初始化一次，生命周期贯穿程序运行。

### 类静态成员

```cpp
static const AP_Scheduler::Task scheduler_tasks[];
```

任务表属于整个 `Sub` 类型，而不是每个对象各保存一份。

### 文件内静态函数

```cpp
static void failsafe_check_static()
```

该函数只在当前 `.cpp` 可见，避免与其他编译单元同名函数冲突。

## 12. 宏和条件编译

### 普通宏

```cpp
#define FAST_TASK(func) FAST_TASK_CLASS(Sub, &sub, func)
```

预处理器在编译前进行文本展开。宏没有普通函数那样完整的类型检查，因此阅读时要找到其最终展开目标。

### 条件编译

```cpp
#if AP_OPTICALFLOW_ENABLED
    AP_OpticalFlow optflow;
#endif
```

宏为 0 时，这段代码在预处理后根本不存在，不是“运行时跳过”。

对比运行时判断：

```cpp
if (optflow.enabled()) {
    // 对象已经编译进固件，只是运行时决定是否使用
}
```

必须区分：

- 编译开关决定代码是否进入固件，影响 Flash/RAM；
- 参数决定已编译功能在运行时是否启用。

## 13. 回调是什么

回调的核心是：A 模块保存一个“以后要调用的函数或对象”，但不需要知道具体实现。

`AP_HAL_MAIN_CALLBACKS(&sub)` 把 `sub` 交给 HAL：

```text
HAL 知道何时调用 setup/loop
Sub 知道 setup/loop 最终应该完成什么车辆工作
```

任务库同样使用回调：`AP_Mission` 不应该依赖 `Sub` 的全部实现，因此构造时接收开始任务、验证任务和退出任务的函数对象。

看到以下结构时，应把它们归入“回调/函数对象”：

- `FUNCTOR_BIND_MEMBER(...)`；
- `SCHED_TASK_CLASS(...)`；
- `Callbacks*`；
- 保存成员函数的 `task_fn_t`。

## 14. `enum class` 和模式编号

```cpp
Mode::Number control_mode;
```

`enum class` 是有作用域的枚举。必须写 `Mode::Number::MANUAL`，避免不同模块的 `MANUAL` 名字冲突。

模式编号会经过参数、MAVLink 和地面站传播，因此不能因为调整源码顺序而随意改变既有数值。

## 15. 位域和状态标志

`Sub.h` 中：

```cpp
uint8_t at_bottom : 1;
uint8_t at_surface : 1;
```

冒号后的 `1` 表示只占一个 bit。多个布尔状态可紧凑存储，但要注意：

- 它们不是普通 `bool`；
- 不应取得位域成员地址；
- 修改状态时要理解它是否由其他任务异步读取。

## 16. 固定宽度整数和嵌入式约束

代码经常使用：

- `uint8_t`：无符号 8 位；
- `int16_t`：有符号 16 位；
- `uint32_t`：无符号 32 位；
- `uint64_t`：无符号 64 位。

嵌入式系统必须明确范围、内存和溢出行为。例如毫秒时间通常用 `uint32_t`，通过无符号减法处理计数回绕。

看到类型时要同时问：

- 它的物理单位是什么？
- 最大值和最小值是什么？
- 是否会溢出？
- 它是否来自中断或其他线程？

## 17. 如何阅读一个陌生函数

不要从函数第一行直接陷入算法。按下面顺序：

1. **谁拥有它**：类名和文件位置；
2. **谁调用它**：直接搜索函数名和任务表；
3. **何时调用**：启动、主循环、某频率任务、消息回调还是中断；
4. **输入是什么**：参数、成员状态、传感器或消息；
5. **输出是什么**：返回值、修改成员、发送消息或写执行器；
6. **失败会怎样**：返回 false、设置 health、触发 failsafe 还是静默退出；
7. **受什么宏控制**：该函数是否一定存在于当前固件。

## 18. 本章实际追踪练习

### 练习一：证明 `Sub` 是 HAL 回调对象

按顺序打开：

1. `ArduSub/Sub.h`；
2. `libraries/AP_Vehicle/AP_Vehicle.h`；
3. `libraries/AP_HAL/HAL.h` 中的 `Callbacks`；
4. `ArduSub/Sub.cpp` 末尾；
5. `libraries/AP_HAL/AP_HAL_Main.h`。

画出继承关系，并说明 `&sub` 为什么能传给 `hal.run()`。

### 练习二：找到任务表的真实对象

回答：

1. `scheduler_tasks` 在哪里声明？
2. 在哪里定义？
3. 为什么它是 `static`？
4. `get_scheduler_tasks()` 返回的是复制品还是原数组地址？
5. 谁调用 `get_scheduler_tasks()`？

### 练习三：区分编译开关和运行参数

选择 `AP_OPTICALFLOW_ENABLED`：

1. 找到它保护的成员和任务；
2. 找到光流运行时的 enable 参数；
3. 分别说明关闭宏和关闭参数对固件的不同影响。

## 本章验收问题

不看本文，尝试回答：

1. `Sub.h` 和 `Sub.cpp` 为什么分开？
2. 继承和组合在 `Sub` 中分别解决什么问题？
3. 为什么 `AP_Vehicle::loop()` 是 `final`？
4. `const AP_Scheduler::Task *&tasks` 怎样从变量名向外阅读？
5. 全局 `Sub sub` 在什么时候构造？
6. `AP::ins()` 是创建对象还是取得单例？
7. `#if FEATURE_ENABLED` 和 `if (feature.enabled())` 有何区别？
8. 回调怎样减少 HAL 对车辆实现的依赖？

能够独立回答以上问题后，再进入[第 2 章：编译链](02_build_chain.md)。
