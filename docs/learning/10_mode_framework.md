# 第 10 章：ArduSub 模式框架

模式不是一套独立控制器，而是决定：

- 当前接受哪类输入；
- 需要哪些状态估计；
- 怎样产生位置、速度、姿态或直接推力目标；
- 失效和未解锁时怎样处理。

~~~text
驾驶输入 / MAVLink setpoint / Mission
    ↓
当前 Mode::run()
    ↓
位置控制器、姿态控制器或直接 Motors 输入
~~~

## 1. 文件地图

1. **ArduSub/mode.h**：模式编号、基类与派生类；
2. **ArduSub/mode.cpp**：编号映射、切换和统一运行入口；
3. **ArduSub/mode_manual.cpp**；
4. **ArduSub/mode_stabilize.cpp**；
5. **ArduSub/mode_acro.cpp**；
6. **ArduSub/mode_althold.cpp**；
7. **ArduSub/mode_poshold.cpp**；
8. **ArduSub/mode_guided.cpp**；
9. **ArduSub/mode_auto.cpp**。

第一遍先读 mode.h、mode.cpp 和一个简单模式，不要同时打开所有 mode 文件。

## 2. Mode 基类是什么

Mode 定义所有模式必须遵守的接口：

~~~cpp
virtual bool init(bool ignore_checks);
virtual void run() = 0;
virtual bool requires_GPS() const = 0;
virtual bool requires_altitude() const = 0;
virtual bool allows_arming(bool from_gcs) const = 0;
virtual const char *name() const = 0;
virtual const char *name4() const = 0;
virtual Mode::Number number() const = 0;
~~~

纯虚函数迫使每个具体模式说明自己的能力和需求。

Mode 构造函数还把 sub 中常用对象保存为引用或指针，例如 g、ahrs、motors、position_control 和 attitude_control。它们不是新的控制器副本，而是指向 Sub 已有对象。

## 3. 模式对象何时创建

Sub 类中直接包含：

- mode_manual；
- mode_stabilize；
- mode_acro；
- mode_althold；
- mode_guided；
- mode_auto 等。

这些是静态生命周期车辆对象的成员，不是在每次切模式时 new。

mode_from_mode_num() 只是把枚举编号映射到已有对象地址：

~~~text
Mode::Number::GUIDED
    ↓
&mode_guided
~~~

因此模式切换不会销毁和重建整个控制系统，但 init() 必须把本次进入所需状态正确重置。

## 4. 模式编号是外部接口

当前编号包括：

| 编号 | 模式 |
|---:|---|
| 0 | Stabilize |
| 1 | Acro |
| 2 | AltHold / Depth Hold |
| 3 | Auto |
| 4 | Guided |
| 7 | Circle |
| 9 | Surface |
| 16 | PosHold |
| 19 | Manual |
| 20 | MotorDetect |
| 21 | SurfTrak |

编号会出现在 MAVLink custom_mode、参数、日志和地面站中。新增模式不能只找一个空数字，还要检查协议兼容、QGC 显示和上游约定。

## 5. set_mode 的真实顺序

~~~text
请求 Mode::Number + ModeReason
    ↓
已经是该模式？
    ├─ 是：只更新 reason，返回成功
    └─ 否
        ↓
mode_from_mode_num
        ↓
检查对象存在
        ↓
检查 requires_GPS / position_ok
        ↓
检查 requires_altitude / depth estimate
        ↓
new_flightmode->init(false)
        ↓ 成功后
Sub::exit_mode(old, new)
        ↓
记录 previous/current/reason
        ↓
写模式日志、发 HEARTBEAT、更新 Notify
~~~

一个关键安全性质是：新模式检查或 init 失败时，不会先退出旧模式。

## 6. 当前版本没有派生 Mode::exit

本仓库 Mode 基类没有虚拟 exit()。旧模式的高层清理由 Sub::exit_mode() 统一完成，例如：

- 重置相机云台模式；
- 恢复最大 throttle。

另一个接受 Mode::Number 的 exit_mode 版本还包含离开 AUTO 时停止 mission 的逻辑。

阅读或新增模式必须以当前实际调用版本为准，不能从别的车辆或旧教程假设每个 Mode 都会自动收到 exit 回调。

## 7. requires_GPS 的名称是历史语义

Guided、Auto、PosHold 等返回 requires_GPS，但 set_mode 实际调用 position_ok()。

在无 GPS 但 ExternalNav 正常的系统中，position_ok 仍可能满足。判断重点是“是否有可信位置估计”，不应仅按函数名字断定一定要物理 GPS。

requires_altitude 则保护需要垂向状态的模式。

## 8. 模式怎样周期运行

Sub 任务表把 update_flight_mode 设为 FAST_TASK：

~~~cpp
void Sub::update_flight_mode()
{
    flightmode->run();
}
~~~

虚函数分派会调用当前对象的 run。切换只是改变 flightmode 指针和 control_mode。

## 9. 各模式的控制层次

### Manual

驾驶输入直接形成推进器轴向输入，没有姿态稳定。适合最直接操纵，也最依赖驾驶员。

### Stabilize

roll/pitch/yaw 进入姿态控制；垂向推力仍由驾驶员直接给出。

### Acro

驾驶输入代表机体系角速度目标，而不是姿态角目标。松杆后主要目标是零角速度，不等于自动回到水平。

### AltHold

姿态由驾驶员控制，垂向经过深度/位置控制器保持深度。

### PosHold

在深度保持基础上增加水平位置保持，并允许驾驶员覆盖运动目标。

### Guided

接受外部即时位置、速度、位置速度或姿态目标。伴随计算机持续规划下一目标点时主要使用它。

### Auto

从 AP_Mission 的任务列表取命令并驱动导航，也能进入 Auto_NavGuided 子状态。适合水面航线和预装任务。

### Circle

运行圆形导航目标。

### Surface

自动向水面运动，同时保留相应水平控制能力。

### MotorDetect

用于推进器方向识别，不是普通的单电机台架测试。它的输出路径会绕开常规 motors_output 分支。

### SurfTrak

基于朝下测距保持离底高度，与仅保持压力深度不同。

## 10. Guided 内部还有子模式

~~~text
Guided_WP       位置目标
Guided_Velocity 速度目标
Guided_PosVel   位置 + 速度
Guided_Angle    姿态 + 爬升率
~~~

MAVLink handler 根据有效字段调用 guided_set_destination、guided_set_velocity、guided_set_destination_posvel 或 guided_set_angle。这些函数也会切换 guided_mode。

ModeGuided::run() 再根据 guided_mode 调用相应控制函数。

## 11. Guided 超时

当前速度、位置速度和姿态 Guided 分支包含约 3 秒更新超时处理：

- 速度目标超时后归零；
- 位置速度中的速度部分归零；
- 姿态目标超时后 roll/pitch 和 climb rate 归零。

具体行为不是所有目标统一“立刻切模式”。外部规划器必须：

- 稳定周期发送；
- 监控 HEARTBEAT 和当前模式；
- 设计链路中断后的 Pixhawk failsafe；
- 不把单次 setpoint 当成永久租约。

## 12. Auto 与 Guided 如何并存

推荐理解：

~~~text
Guided
    外部计算机保存计划并持续给即时目标

Auto
    Pixhawk 保存 Mission 并自行推进任务项
~~~

保留 Auto 不妨碍大小脑分离：

- 水下可使用 Guided；
- 水面 GPS 可执行 Auto 航线；
- 某些任务项还能触发 Auto_NavGuided；
- 失联行为可按模式分别设计。

## 13. 解锁能力与模式切换不同

allows_arming() 回答“是否允许在该模式下解锁”，set_mode 检查回答“现在能否进入模式”。

最终能否解锁还要经过 AP_Arming_Sub 和公共检查。某模式返回 true 不等于跳过传感器、Safety、漏水和 EKF 检查。

## 14. 新模式最小设计清单

1. 分配稳定编号；
2. 在 mode.h 声明类；
3. 在 Sub 中创建对象；
4. 在 mode_from_mode_num 加映射；
5. 实现 init 和 run；
6. 明确位置、深度和解锁要求；
7. 确定输入来源与超时；
8. 未解锁时设置安全 spool 状态并 relax 控制器；
9. 进入时初始化控制目标；
10. 需要时补统一退出清理；
11. 加日志和地面站显示；
12. SITL 测试切入、运行、切出和拒绝路径。

## 15. 验收问题

1. Mode 是控制器还是目标生成策略？
2. 模式对象每次切换都会 new 吗？
3. 模式编号为什么不能随意改变？
4. set_mode 为什么先 init 新模式再退出旧模式？
5. 当前 Mode 是否有虚拟 exit？
6. requires_GPS 是否必然要求物理 GPS？
7. Guided 的四个子模式是什么？
8. Guided 超时一定切回 Manual 吗？
9. Auto 与 Guided 适合哪些规划边界？
10. allows_arming 为 true 是否足以解锁？
