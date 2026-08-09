# ArduSub 4.7.0 系统架构、源码与工程总览

本篇合并原 00–08 的通用学习内容，作为 `Sub_learning` 的“总”部分。它回答四类问题：

1. ArduSub 从哪里启动，实时任务怎样运行。
2. 输入、状态估计、模式、控制器、混控和硬件输出怎样分层。
3. `ArduSub/` 每个文件以及控制主线共享库分别负责什么。
4. 二次开发应怎样管理参数、依赖、构建、测试、板卡适配和安全回退。

本篇只建立稳定、可复用的系统知识，不绑定某个新传感器或实验功能。后续教学全部从具体需求出发，放在独立案例文档中。

## 推荐学习顺序

| 阶段 | 阅读范围 | 学习成果 |
|---|---|---|
| 建立总图 | 第 1–5 节 | 能画出生命周期、跨周期调度和六层控制数据流 |
| 理解控制 | 第 6–9 节 | 能解释模式递进、姿态/位置控制、轨迹整形和推进器状态机 |
| 掌握工程方法 | 第 10–13 节 | 能判断代码归属，完成构建测试、安全审查和 Pixhawk 类板卡适配 |
| 开始案例 | [案例候选表](10_case_catalog.md) | 选择一个小需求，沿完整链路完成实现与验证 |

## 1. 生命周期入口

ArduSub 的全局车辆对象和 HAL 生命周期入口位于 [ArduSub/Sub.cpp](../ArduSub/Sub.cpp)：

```cpp
Sub sub;
AP_Vehicle& vehicle = sub;
AP_HAL_MAIN_CALLBACKS(&sub);
```

`AP_HAL_MAIN_CALLBACKS(&sub)` 把 `Sub` 接入 AP_HAL/AP_Vehicle 的初始化和循环框架。分析启动、调度或板级行为时，应继续追踪 AP_Vehicle 与对应 HAL，而不是在 `ArduSub/` 中寻找一个传统的 `main()`。

## 2. Fast loop 的关键顺序

[Sub::scheduler_tasks](../ArduSub/Sub.cpp) 中与控制链最相关的 fast tasks 顺序是：

```text
AP_InertialSensor::update
    -> Sub::run_rate_controller
    -> Sub::motors_output
    -> Sub::read_AHRS
    -> Sub::read_inertia
    -> Sub::check_ekf_yaw_reset
    -> Sub::update_flight_mode
```

这是一条持续循环的实时流水线。`run_rate_controller()` 在非 Manual、非 Motor Detect 模式下调用 `attitude_control.rate_controller_run()`；`motors_output()` 再进入推进器库。不要只根据一个循环内的文字排列，武断地把某个目标值和某个 PWM 视为同一步计算，调试时应结合时间戳、控制器状态和日志确认。

`fifty_hz_loop()` 还执行驾驶员输入 failsafe 检查、读取遥控输入和模式开关。改变输入处理时，必须同时审计 fast loop 与 50 Hz 路径。

## 3. 飞行模式是运行时分派

[Sub::update_flight_mode()](../ArduSub/mode.cpp) 的核心只有：

```cpp
flightmode->run();
```

这里不是“调用整个 `mode_manual.cpp`”，而是通过 `Mode` 基类虚函数分派到当前对象的 `run()`。[Sub::mode_from_mode_num()](../ArduSub/mode.cpp) 返回 `ModeManual`、`ModeStabilize`、`ModeAlthold`、`ModePoshold`、`ModeGuided` 等静态模式对象；这些对象在 [ArduSub/Sub.h](../ArduSub/Sub.h) 中由 `Sub` 持有。

模式切换由 `Sub::set_mode()` 负责，当前源码会依次处理：

1. 是否已经处于目标模式。
2. 模式号能否映射为有效对象。
3. 目标模式需要的位置估计是否可用。
4. 从不要求高度的模式切换到要求高度的模式时，高度估计是否有效。
5. 新模式 `init(false)` 是否成功。
6. 旧模式退出清理、当前模式指针更新、日志和 heartbeat 通知。

因此，增加或修改模式不能只改 `run()`；还要检查模式选择、初始化条件、退出清理、failsafe 回退和 GCS/MAVLink 入口。

## 4. 实际的控制与输出对象

[ArduSub/Sub.h](../ArduSub/Sub.h) 中的实际推进器对象是：

```cpp
AP_Motors6DOF motors;
```

这决定了阅读 `AP_Motors` 时只需沿 `AP_Motors6DOF -> AP_MotorsMulticopter -> AP_Motors` 的实际继承路径追踪，不需要横向读完 Heli 等无关实现。

主要控制对象包括：

- `attitude_control`：姿态外环目标与角速度内环。
- `pos_control`：North-East 和垂直方向的位置/速度/加速度控制与目标整形。
- `wp_nav`：航点、样条和轨迹约束。
- `inertial_nav`、`ahrs`：状态估计接口。
- `motors`：推进器许可状态、6DOF 力分配和最终输出转换。

## 5. 分层职责

本节采用两级表格。第一级先说明系统层次和交接关系；第二级完整枚举当前 `ArduSub/` 顶层文件，并列出控制主线需要继续阅读的共享库入口。

这里的“完整枚举”范围是当前 4.7.0 的 `ArduSub/` 顶层文件。`libraries/` 中存在大量由 Waf 传递引入的通用实现，不能把它们全部视为 ArduSub 专有文件；本节列出 `ArduSub/wscript` 的直接库和输入—估计—控制—输出主线的关键入口。需要裁剪或修改依赖时，仍必须重新计算实际 dependency closure。

### 5.1 第一级：系统层级总览

| 一级层 | 二级子层 | ArduSub 车辆层入口 | 向下一层交付什么 |
|---|---|---|---|
| 车辆框架 | 生命周期、对象组合、调度、参数与编译配置 | `Sub.cpp`、`Sub.h`、`system.cpp`、`Parameters.*`、`config.h`、`defines.h`、`wscript` | 已初始化的车辆对象、按频率执行的任务、参数和功能开关 |
| 输入与通信 | RC、joystick、MAVLink、GCS、命令入口 | `radio.cpp`、`RC_Channel_Sub.*`、`joystick.cpp`、`GCS_*`、`commands*.cpp` | 归一化驾驶输入、模式请求、Guided/mission 目标和外部观测 |
| 传感器与安全状态 | 传感器读取、估计状态同步、arming、failsafe、环境状态 | `sensors.cpp`、`inertia.cpp`、`AP_Arming_Sub.*`、`failsafe.cpp`、`surface_bottom_detector.cpp` | 姿态/位置/深度状态、健康标志、解锁许可和失效动作 |
| 模式策略 | 模式选择、初始化、运行、退出和任务策略 | `mode.cpp`、`mode.h`、全部 `mode_*.cpp`、`commands_logic.cpp` | 姿态、角速度、位置、速度、加速度目标或直接六自由度请求 |
| 控制与坐标转换 | 驾驶员目标换算、姿态/位置/轨迹闭环 | `Attitude.cpp` 及 `AC_AttitudeControl`、`AC_PosControl`、`AC_WPNav` | roll/pitch/yaw 控制量和 throttle/forward/lateral 请求 |
| 推进器与辅助执行器 | spool 状态、6DOF 混控、电机测试和辅助通道 | `motors.cpp`、`actuators.*`、`AP_Motors6DOF` | 各 motor/actuator 的受限通道输出 |
| 硬件输出 | Servo function、HAL、ChibiOS 定时器与板级资源 | `SRV_Channel`、`AP_HAL`、`AP_HAL_ChibiOS` | PWM 或相应数字输出协议 |
| 诊断与工程资料 | 日志、版本、发布记录、兼容构建入口 | `Log.cpp`、`version.h`、`ReleaseNotes.txt`、`Makefile.waf` | 可追溯日志、版本信息和构建入口 |

### 5.2 第二级 A：车辆框架、配置与构建文件

| 子层 | 文件 | 具体职责和功能 | 阅读或修改重点 |
|---|---|---|---|
| 中心对象 | [`Sub.h`](../ArduSub/Sub.h) | 声明 `Sub : AP_Vehicle`，组合 AHRS、控制器、motors、mission、GCS、传感器、参数、状态和静态模式对象；同时声明车辆任务与辅助函数 | 判断一个对象的真实动态类型、所有权和生命周期时先看这里；不要重复创建已有单例或控制器 |
| 生命周期与调度 | [`Sub.cpp`](../ArduSub/Sub.cpp) | 创建全局 `Sub sub`，接入 `AP_HAL_MAIN_CALLBACKS`，定义 `scheduler_tasks`，运行 rate controller、AHRS/高度更新、周期日志、电池/罗盘和统计任务 | 调度顺序是跨周期流水线；修改频率或任务预算前要审计前后任务、CPU 预算和数据时序 |
| 系统初始化 | [`system.cpp`](../ArduSub/system.cpp) | `init_ardupilot()` 初始化车辆子系统，执行 INS ground startup，提供 `position_ok()`、`ekf_position_ok()`、`optflow_position_ok()` 和日志允许条件 | 模式进入条件里的“位置有效”最终会回到这里；不要把函数名误解为只支持 GPS |
| 参数声明 | [`Parameters.h`](../ArduSub/Parameters.h) | 定义持久化参数枚举索引、`Parameters` 和 `ParametersG2` 成员及类型 | 已有索引属于存储 ABI，禁止重排或复用 |
| 参数定义与迁移 | [`Parameters.cpp`](../ArduSub/Parameters.cpp) | 定义 `var_info`、参数元数据、默认值、子组绑定，加载参数并转换旧参数；还更新 leak/relay pins、joystick actuator 和 lights 配置 | 新参数要补完整文档、选择未占用索引并验证旧参数迁移；不能只改 GCS 显示文字 |
| 编译配置 | [`config.h`](../ArduSub/config.h) | 定义主循环频率、feature guard、模式/传感器开关和车辆默认值 | 保持 guard 依赖方向；核心功能不能反向依赖可选组件 |
| 公共常量 | [`defines.h`](../ArduSub/defines.h) | 定义 yaw/Surface 状态、日志位、failsafe 动作、MAVLink target mask 等车辆级枚举与常量 | 新值要审计参数、日志、MAVLink 和地面站兼容性 |
| 当前状态占位 | [`AP_State.cpp`](../ArduSub/AP_State.cpp) | 当前基线只包含 `Sub.h`，没有独立状态逻辑 | 不要在这里寻找隐藏状态机；新增状态前先判断是否应属于 `Sub.h`、模式或公共库 |
| 版本 | [`version.h`](../ArduSub/version.h) | 声明 ArduSub 固件版本、名称和版本类型宏 | 发布时与稳定基线、生成固件和 Git tag 一起核对 |
| Waf 车辆规则 | [`wscript`](../ArduSub/wscript) | 声明 ArduSub 静态库、`ardusub` 程序、直接 `ap_libraries` 和构建组 | 判断直接依赖和车辆构建入口的首要证据；源文件收集和传递依赖仍由 Waf 扩展处理 |
| Make 兼容入口 | [`Makefile.waf`](../ArduSub/Makefile.waf) | 将传统 `make` 入口转发到仓库根目录 `Makefile.waf sub` | 不是独立构建系统；正式构建仍从仓库根运行 `./waf` |
| 发布记录 | [`ReleaseNotes.txt`](../ArduSub/ReleaseNotes.txt) | 保存历次 ArduSub 功能和兼容性变化的历史说明 | 用于理解历史原因，不能替代当前源码作为行为证据 |
| 传统用户 hook | [`UserCode.cpp`](../ArduSub/UserCode.cpp) | 提供受 `USERHOOK_*` 宏控制的 init/fast/50Hz/medium/slow/superslow 空 hook | 仅用于理解旧扩展点；正式功能优先放入职责明确、可测试的组件 |
| 传统用户变量 | [`UserVariables.h`](../ArduSub/UserVariables.h) | 在 `USERHOOK_VARIABLES` 下保留用户变量示例 | 不是公共库或新功能的默认存放点，示例变量也不是当前产品能力 |

### 5.3 第二级 B：输入、GCS 与外部命令

| 子层 | 文件 | 具体职责和功能 | 向控制主线的交接 |
|---|---|---|---|
| RC 输入输出初始化 | [`radio.cpp`](../ArduSub/radio.cpp) | 绑定 roll/pitch/throttle/yaw/forward/lateral 通道，设置范围和死区；读取 RC、检测超时；在 `init_rc_out()` 中按 `FRAME_CONFIG` 初始化 motors 和 SRV 输出 | 向模式提供规范化通道状态；向 `AP_Motors6DOF` 交付当前 frame 配置 |
| RC 车辆适配声明 | [`RC_Channel_Sub.h`](../ArduSub/RC_Channel_Sub.h) | 声明 `RC_Channel_Sub` 与 `RC_Channels_Sub` 的车辆派生接口 | 把通用 RC_Channel 接到 Sub 的模式、arming 与 failsafe 语义 |
| RC 车辆适配实现 | [`RC_Channel_Sub.cpp`](../ArduSub/RC_Channel_Sub.cpp) | 定义 RC 参数组、模式开关回调、有效输入判断、RC failsafe 查询和 throttle arming 检查 | 模式开关通过 `Sub::set_mode()`，不会直接替换模式指针 |
| joystick 映射 | [`joystick.cpp`](../ArduSub/joystick.cpp) | 将 MAVLink manual control 与按钮转换为 RC override，处理按钮按下/释放、模式/arming/灯光/增益/继电器/servo 功能、输入保持和中性控制 | 形成驾驶员输入和离散命令；修改时同时审计 GCS failsafe、按钮保持和接管路径 |
| script 按钮状态 | [`script_button.h`](../ArduSub/script_button.h)、[`script_button.cpp`](../ArduSub/script_button.cpp) | 在 scripting 启用时保存按钮 pressed 状态和带饱和的按压计数，提供读取与清零接口 | 只保存脚本按钮事件，不负责执行推进器或模式逻辑 |
| GCS 车辆前端声明 | [`GCS_Sub.h`](../ArduSub/GCS_Sub.h) | 声明 `GCS_Sub` 和 Sub 专用 MAVLink channel 类型 | 把通用 GCS 框架绑定到车辆对象 |
| GCS 状态实现 | [`GCS_Sub.cpp`](../ArduSub/GCS_Sub.cpp) | 根据当前模式和传感器状态填充 MAV_SYS_STATUS controller、压力和 rangefinder 等 present/enabled/health 位 | 对外报告真实能力和健康，不生成控制目标 |
| MAVLink 车辆适配声明 | [`GCS_MAVLink_Sub.h`](../ArduSub/GCS_MAVLink_Sub.h) | 声明 Sub 专用 MAVLink 发送、接收、命令和能力接口 | 规定哪些通用 GCS_MAVLink 行为由 Sub 覆盖 |
| MAVLink 车辆适配实现 | [`GCS_MAVLink_Sub.cpp`](../ArduSub/GCS_MAVLink_Sub.cpp) | 报告 frame、base/custom mode、状态、深度、PID 和 available modes；处理 manual control、模式/任务/Guided/Motor Test 等消息和命令 | 外部位置/速度/姿态目标进入 Guided/mission；驾驶输入进入 joystick/RC；电机测试进入 `motors.cpp` |
| Home 管理 | [`commands.cpp`](../ArduSub/commands.cpp) | 从 EKF 更新 home，设置当前或指定 home，并处理 home lock | 为 mission、相对位置和导航提供原点，不执行航点轨迹 |
| Mission 命令逻辑 | [`commands_logic.cpp`](../ArduSub/commands_logic.cpp) | 启动和验证 mission command，处理 waypoint、surface、RTL、loiter、circle、nav-guided、delay、yaw、speed、ROI 和 mount 等命令 | 把任务命令转成 Auto/Guided/WPNav 目标，并报告完成或失败 |

### 5.4 第二级 C：传感器、状态、安全与日志

| 子层 | 文件 | 具体职责和功能 | 阅读重点 |
|---|---|---|---|
| 深度与测距 | [`sensors.cpp`](../ArduSub/sensors.cpp) | 读取 barometer，初始化和读取 downward rangefinder，维护 rangefinder 健康计数、信号质量、倾斜修正和超时状态 | 区分压力深度、离底距离和 EKF 垂直状态；它们不是同一个量 |
| 惯导状态同步 | [`inertia.cpp`](../ArduSub/inertia.cpp) | `read_inertia()` 从 AHRS/EKF 获取 NE 位置并更新 surface/bottom detector | 这是车辆层消费估计结果的入口之一，不是 IMU 驱动实现 |
| 水面/触底检测 | [`surface_bottom_detector.cpp`](../ArduSub/surface_bottom_detector.cpp) | 综合垂直运动、目标、motor limit 和计时判断 surfaced/bottomed，并更新相应状态 | AltHold/Surface 等模式会消费这些状态；修改阈值要验证误触发和解除条件 |
| 地形状态 | [`terrain.cpp`](../ArduSub/terrain.cpp) | 定期更新 AP_Terrain，并记录 terrain/rangefinder 状态 | 当前车辆使用范围有限，仍与 terrain failsafe 和日志相连 |
| 围栏检查 | [`fence.cpp`](../ArduSub/fence.cpp) | 异步执行 AC_Fence 检查并在新 breach 时记录日志 | 只在相应 feature 启用时有效，不能绕过 guard |
| 转圈计数 | [`turn_counter.cpp`](../ArduSub/turn_counter.cpp) | 根据 yaw 变化累计机体旋转圈数 | 供状态/日志使用，不是 yaw controller |
| 解锁声明 | [`AP_Arming_Sub.h`](../ArduSub/AP_Arming_Sub.h) | 声明 Sub 对通用 `AP_Arming` 的检查、arm/disarm 覆盖 | 理解推进器获得许可前必须经过的车辆级入口 |
| 解锁实现 | [`AP_Arming_Sub.cpp`](../ArduSub/AP_Arming_Sub.cpp) | 检查 RC 校准、可用 disarm 功能、系统/AHRS/INS/throttle 条件；arm 时设置 soft-armed、启用输出并置 motors armed，disarm 时停止 motors、重置 mission 和输入保持 | arming、HAL soft armed、motor armed 和 spool state 是多层约束，不能压缩成一个布尔值 |
| 失效保护 | [`failsafe.cpp`](../ArduSub/failsafe.cpp) | 处理 mainloop、sensors、EKF、battery、pilot input、内部压力/温度、leak、GCS、crash、terrain 和 radio failsafe 及动作优先级 | 修改任一模式或传感器时必须检查它在这里的触发、模式回退和冲突关系 |
| 车辆日志 | [`Log.cpp`](../ArduSub/Log.cpp) | 定义 control tuning、attitude、通用 data、Guided target、startup 等车辆日志结构和写入函数 | 设计测试前先确认需要的目标、估计、输出和 failsafe 是否可观测 |

### 5.5 第二级 D：模式基类与所有具体模式

| 层次 | 文件 | 具体职责和功能 | 相对上一模式新增的能力 |
|---|---|---|---|
| 模式接口 | [`mode.h`](../ArduSub/mode.h) | 定义 `Mode::Number`、Mode 基类契约、Guided/Auto 子状态以及所有具体模式类；声明 requires position/altitude、arming、name 和 run/init 接口 | 建立运行时多态边界；模式号是外部兼容接口 |
| 模式生命周期 | [`mode.cpp`](../ArduSub/mode.cpp) | 构造 Mode 对公共对象的引用，映射模式号，执行 `set_mode()` 检查/init/退出/日志/notify，并由 `update_flight_mode()` 调用当前 `run()` | 统一所有模式的进入、退出和运行分派 |
| 直接六自由度 | [`mode_manual.cpp`](../ArduSub/mode_manual.cpp) | 解锁后把 roll/pitch/yaw/throttle/forward/lateral 驾驶输入基本直接写入 motors | 基线：没有常规姿态 rate controller |
| 角速度控制 | [`mode_acro.cpp`](../ArduSub/mode_acro.cpp) | 将驾驶输入转换为机体系角速度请求，同时保留手动平移/升沉 | 相对 Manual 增加角速度闭环 |
| 姿态稳定 | [`mode_stabilize.cpp`](../ArduSub/mode_stabilize.cpp) | roll/pitch 生成目标姿态，yaw 有杆量时给 rate、松杆后保持航向，forward/lateral 仍由驾驶员控制 | 相对 Manual 增加姿态外环和角速度内环 |
| 深度保持 | [`mode_althold.cpp`](../ArduSub/mode_althold.cpp) | 通过 `run_pre() -> control_depth() -> run_post()` 组合姿态、垂直速度/位置控制及水平手动输入，并应用 surface/bottom 约束 | 在 Stabilize 能力上增加垂直位置/速度/加速度闭环 |
| 离底跟踪 | [`mode_surftrak.cpp`](../ArduSub/mode_surftrak.cpp) | 在 AltHold 基础上利用 downward rangefinder 维持离底目标，处理健康、目标重置和 surface offset | 将垂直目标从单纯深度保持扩展为离底距离跟踪 |
| 水平定点 | [`mode_poshold.cpp`](../ArduSub/mode_poshold.cpp) | 在 AltHold 基础上把 body forward/lateral 转为 NE 速度目标，运行水平位置控制，再把输出转回 body 力 | 增加水平 NE 位置/速度闭环 |
| 外部引导 | [`mode_guided.cpp`](../ArduSub/mode_guided.cpp) | 接受 position、velocity、position+velocity、angle/climb-rate 目标，选择对应控制器，处理 yaw 策略、3 秒目标超时和 Guided limits | 将目标来源从驾驶杆扩展为 GCS/外部命令 |
| 自动任务 | [`mode_auto.cpp`](../ArduSub/mode_auto.cpp) | 根据 Auto 子状态运行 waypoint、circle、nav-guided、loiter 和 terrain recovery，设置自动 yaw/ROI/rate | 在 Guided/WPNav 能力上增加 mission 状态与命令推进 |
| 圆周运动 | [`mode_circle.cpp`](../ArduSub/mode_circle.cpp) | 初始化并运行 AC_Circle，组合深度、位置和朝向控制 | 提供专用圆形轨迹策略 |
| 自动上浮 | [`mode_surface.cpp`](../ArduSub/mode_surface.cpp) | 要求有效高度，使用垂直控制上浮，到达水面后切回 AltHold | 提供确定的上浮状态与完成切换 |
| 电机方向检测 | [`mode_motordetect.cpp`](../ArduSub/mode_motordetect.cpp) | 按受控时序驱动单个 motor 并根据机体角运动推断方向，保存 reverse 配置 | 专用维护模式；不运行常规 rate controller，实机必须拆桨/隔离 |

推荐仍按 `Manual -> Stabilize -> AltHold -> PosHold -> Guided` 阅读。Acro、SurfTrak、Circle、Surface、Motor Detect 和 Auto 在主线掌握后按需求插入。

### 5.6 第二级 E：车辆层控制换算、推进器和辅助执行器

| 子层 | 文件 | 具体职责和功能 | 与共享库的交接 |
|---|---|---|---|
| 驾驶员目标换算 | [`Attitude.cpp`](../ArduSub/Attitude.cpp) | 将驾驶员 roll/pitch、yaw、throttle 和水平通道换算为目标倾角、角速度、升沉/水平速度；处理 EKF yaw reset、ROI/look-ahead yaw 和 body/NE 旋转 | 向 `AC_AttitudeControl`、`AC_PosControl` 或模式提供带单位和坐标语义的目标 |
| 推进器车辆适配 | [`motors.cpp`](../ArduSub/motors.cpp) | 启用 motor output，在每周期调用 `motors.output()` 前处理 spoolup block；实现 motor test；把 WPNav/Circle/PosControl 的 roll/pitch 输出换算成 body lateral/forward | 把车辆控制器结果交给 `AP_Motors6DOF`，但最终 PWM 仍在共享库/SRV/HAL |
| 辅助执行器声明 | [`actuators.h`](../ArduSub/actuators.h) | 定义六个通用 actuator 的参数、当前值和增减/居中接口 | 与推进器 motor 通道分开管理 |
| 辅助执行器实现 | [`actuators.cpp`](../ArduSub/actuators.cpp) | 通过 `k_actuator1...` 查找 SRV 通道，将归一化值映射到 min/trim/max PWM，并响应 joystick 增减/切换 | 直接使用 `SRV_Channels` 输出辅助设备，不参与 6DOF 推进器混控 |

### 5.7 第三级：控制主线的共享库逐文件入口

以下文件不属于 `ArduSub/`，但车辆层会直接或间接调用。阅读时只追当前 Sub 使用的前端、backend 和动态类型，不需要横向读完 Heli、Plane 或 Rover 实现。

#### 5.7.1 传感器与状态估计

| 组件 | 关键文件 | 具体职责和功能 | ArduSub 使用方式 |
|---|---|---|---|
| IMU 前端 | [`AP_InertialSensor.h`](../libraries/AP_InertialSensor/AP_InertialSensor.h)、[`AP_InertialSensor.cpp`](../libraries/AP_InertialSensor/AP_InertialSensor.cpp) | 管理 gyro/accelerometer 实例、采样、校准、滤波和健康状态 | fast task 首先更新，为 rate controller 和 EKF 提供惯性观测 |
| 罗盘 | [`AP_Compass.h`](../libraries/AP_Compass/AP_Compass.h)、[`AP_Compass.cpp`](../libraries/AP_Compass/AP_Compass.cpp) | 管理磁场观测、校准、方向和健康状态 | 为 AHRS/EKF 的航向约束提供观测；推进器干扰补偿使用 throttle 信息 |
| 压力传感器 | [`AP_Baro.h`](../libraries/AP_Baro/AP_Baro.h)、[`AP_Baro.cpp`](../libraries/AP_Baro/AP_Baro.cpp) | 管理 barometer backend、压力/温度和健康状态 | 水压数据形成深度/高度来源，并被 arming、AltHold 和 failsafe 消费 |
| AHRS 前端 | [`AP_AHRS.h`](../libraries/AP_AHRS/AP_AHRS.h)、[`AP_AHRS.cpp`](../libraries/AP_AHRS/AP_AHRS.cpp) | 向车辆提供统一姿态、位置、速度、origin/home、坐标变换和 estimator 状态接口 | `Sub::read_AHRS()`、模式和控制器通过它消费估计结果 |
| EKF3 前端与核心 | [`AP_NavEKF3.h`](../libraries/AP_NavEKF3/AP_NavEKF3.h)、[`AP_NavEKF3.cpp`](../libraries/AP_NavEKF3/AP_NavEKF3.cpp)、[`AP_NavEKF3_core.h`](../libraries/AP_NavEKF3/AP_NavEKF3_core.h) | 选择/管理 EKF core，融合 IMU、磁罗盘、压力、GPS/ExternalNav 等观测，输出状态和创新健康 | 决定 position/depth/yaw 是否可供 PosHold、Guided、Auto 和 failsafe 使用 |
| 惯导适配 | [`AP_InertialNav.h`](../libraries/AP_InertialNav/AP_InertialNav.h)、[`AP_InertialNav.cpp`](../libraries/AP_InertialNav/AP_InertialNav.cpp) | 向控制器提供经过 AHRS/EKF 统一的 NEU 位置与速度接口 | 被 Mode 和位置控制器用作导航状态入口 |
| ExternalNav | [`AP_VisualOdom.h`](../libraries/AP_VisualOdom/AP_VisualOdom.h)、[`AP_VisualOdom.cpp`](../libraries/AP_VisualOdom/AP_VisualOdom.cpp)、[`AP_VisualOdom_MAV.cpp`](../libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp) | 接收视觉/外部 odometry，处理 backend、延迟、质量、超时和坐标数据 | 外部融合结果可经 MAVLink ODOMETRY 进入 EKF；模式只消费经过验证的估计状态 |
| Rangefinder | [`AP_RangeFinder.h`](../libraries/AP_RangeFinder/AP_RangeFinder.h)、[`AP_RangeFinder.cpp`](../libraries/AP_RangeFinder/AP_RangeFinder.cpp) | 管理测距 backend、方向、质量、状态和距离数据 | 为 SurfTrak、terrain 和离底状态提供原始测距入口 |

#### 5.7.2 姿态、位置与轨迹控制

| 组件 | 关键文件 | 具体职责和功能 | 输出 |
|---|---|---|---|
| 姿态控制基类 | [`AC_AttitudeControl.h`](../libraries/AC_AttitudeControl/AC_AttitudeControl.h)、[`AC_AttitudeControl.cpp`](../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp) | 管理姿态目标、姿态误差到角速度目标的外环、输入整形和通用限制 | 目标角速度和控制状态 |
| Sub 姿态/角速度实现 | [`AC_AttitudeControl_Sub.h`](../libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h)、[`AC_AttitudeControl_Sub.cpp`](../libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp) | 使用 gyro 反馈运行 roll/pitch/yaw rate PID，并把结果写入 Sub motors 接口 | 归一化 roll/pitch/yaw 和 throttle 控制量 |
| 位置控制 | [`AC_PosControl.h`](../libraries/AC_AttitudeControl/AC_PosControl.h)、[`AC_PosControl.cpp`](../libraries/AC_AttitudeControl/AC_PosControl.cpp) | 管理 NE/U 位置、速度、加速度目标，执行 input shaping、限速/限加速度/jerk 和级联闭环 | 导航 roll/pitch、垂直 throttle 及误差/limit 状态 |
| 航点导航 | [`AC_WPNav.h`](../libraries/AC_WPNav/AC_WPNav.h)、[`AC_WPNav.cpp`](../libraries/AC_WPNav/AC_WPNav.cpp) | 生成 waypoint/spline 路径目标，管理速度、加速度、jerk、到达判定和 crosstrack | 向 AC_PosControl 交付连续目标 |
| 圆周导航 | [`AC_Circle.h`](../libraries/AC_WPNav/AC_Circle.h)、[`AC_Circle.cpp`](../libraries/AC_WPNav/AC_Circle.cpp) | 生成圆心、半径、角速度和朝向相关目标 | 供 ModeCircle/Auto Circle 使用 |
| Mission | [`AP_Mission.h`](../libraries/AP_Mission/AP_Mission.h)、[`AP_Mission.cpp`](../libraries/AP_Mission/AP_Mission.cpp) | 存储、推进和回调 mission command，维护任务状态 | 调用 Sub 的 start/verify command，模式层再选择导航行为 |

#### 5.7.3 推进器、通道与硬件输出

| 组件 | 关键文件 | 具体职责和功能 | 阅读边界 |
|---|---|---|---|
| 6DOF 混控 | [`AP_Motors6DOF.h`](../libraries/AP_Motors/AP_Motors6DOF.h)、[`AP_Motors6DOF.cpp`](../libraries/AP_Motors/AP_Motors6DOF.cpp) | 根据 `FRAME_CONFIG` 建立每个 motor 的 roll/pitch/yaw/throttle/forward/lateral 系数，执行 Sub 专用混控、方向修正和双向 PWM 转换 | ArduSub 的实际 motors 动态类型，修改 frame 首先看这里 |
| Multicopter 状态机 | [`AP_MotorsMulticopter.h`](../libraries/AP_Motors/AP_MotorsMulticopter.h)、[`AP_MotorsMulticopter.cpp`](../libraries/AP_Motors/AP_MotorsMulticopter.cpp) | 处理 desired/actual spool state、armed/interlock 约束、输出逻辑、限幅和电池相关状态 | `set_desired_spool_state()` 只提出请求，许可与过渡在这里 |
| Matrix 基础 | [`AP_MotorsMatrix.h`](../libraries/AP_Motors/AP_MotorsMatrix.h)、[`AP_MotorsMatrix.cpp`](../libraries/AP_Motors/AP_MotorsMatrix.cpp) | 管理 motor enable、angular factor、testing order 和 frame 初始化的矩阵基础设施 | 只追 `AP_Motors6DOF` 调用的父类部分，不读无关机型表 |
| Motors 基类 | [`AP_Motors_Class.h`](../libraries/AP_Motors/AP_Motors_Class.h)、[`AP_Motors_Class.cpp`](../libraries/AP_Motors/AP_Motors_Class.cpp) | 定义通用 motor 输入、armed/interlock、frame、output、limit 和 SRV 写出接口 | 所有安全状态与输出 API 的公共契约 |
| Servo function | [`SRV_Channel.h`](../libraries/SRV_Channel/SRV_Channel.h)、[`SRV_Channel.cpp`](../libraries/SRV_Channel/SRV_Channel.cpp)、[`SRV_Channels.cpp`](../libraries/SRV_Channel/SRV_Channels.cpp) | 把 motor/actuator 等功能映射到物理 channel，管理 trim/min/max、PWM 和协议输出 | 混控结果到 HAL 之前的最后一层功能映射 |
| HAL 接口 | [`AP_HAL`](../libraries/AP_HAL/) | 定义 scheduler、RCOutput、GPIO、I2C/SPI/UART、storage 等跨平台接口 | 控制层依赖接口，不写 ChibiOS 寄存器特例 |
| ChibiOS 实现 | [`AP_HAL_ChibiOS`](../libraries/AP_HAL_ChibiOS/) | 实现 STM32/ChibiOS 驱动、板级启动、定时器/DMA/总线和 hwdef 资源映射 | 新 Pixhawk 类板卡通过 hwdef/HAL 适配，不修改模式来适配 GPIO |

#### 5.7.4 通信、记录与直接车辆库

| 组件 | 关键位置 | 具体职责和功能 | 与 ArduSub 的关系 |
|---|---|---|---|
| 通用 MAVLink | [`GCS_MAVLink`](../libraries/GCS_MAVLink/) | 实现消息路由、stream、通用命令、ODOMETRY 和参数/任务协议 | `GCS_MAVLink_Sub` 只覆盖车辆差异，其余行为继承这里 |
| 日志 | [`AP_Logger`](../libraries/AP_Logger/) | 管理日志 backend、消息格式、启动/armed 状态和写入队列 | `Log.cpp` 定义 Sub 专用记录并调用通用 logger |
| 通用解锁 | [`AP_Arming`](../libraries/AP_Arming/) | 提供参数、通用 pre-arm 检查和 arm/disarm 框架 | `AP_Arming_Sub` 叠加潜艇专用的 disarm-button、throttle 和 AHRS 要求 |
| 泄漏检测 | [`AP_LeakDetector`](../libraries/AP_LeakDetector/) | 管理 leak sensor 实例、pin/logic 和检测状态 | 由 Sub failsafe 周期检查并执行告警或上浮 |
| 相机 | [`AP_Camera`](../libraries/AP_Camera/) | 管理相机触发和相关 MAVLink/mission 行为 | 是 `ArduSub/wscript` 直接库，供任务和 joystick 功能调用 |
| Joystick 按钮库 | [`AP_JSButton`](../libraries/AP_JSButton/) | 定义按钮参数、功能枚举、shift/held 行为 | `joystick.cpp` 消费其配置并执行 Sub 动作 |
| 温度传感器 | [`AP_TemperatureSensor`](../libraries/AP_TemperatureSensor/) | 管理外部温度 backend 和读数 | Sub 可通过 MAVLink scaled pressure 兼容路径发送温度 |

这组表格的使用方法不是一次读完全部文件，而是先在 `ArduSub/` 表中找到车辆策略，再沿“向下一层交付什么”进入对应共享库。到达实际动态类型、健康检查、限幅和 SRV/HAL 输出后即可停止横向扩散。

## 6. 用模式递进理解控制体系

初学者不要先读 AUTO。推荐按 `Manual -> Stabilize -> AltHold -> PosHold -> Guided`，每次只观察新增的一层能力。

| 模式 | 驾驶输入/外部目标 | 新增闭环 | 主要输出路径 | 核心风险 |
|---|---|---|---|---|
| Manual | 六个驾驶通道 | 无常规姿态 rate controller | 归一化输入直接写入 motors | 通道、方向、混控、arming |
| Stabilize | roll/pitch 姿态、yaw rate/hold、手动平移 | 姿态外环 + 角速度内环 | `AC_AttitudeControl_Sub -> AP_Motors6DOF` | 角度/角速度单位、松杆航向、饱和 |
| AltHold | 姿态、升沉速度、手动水平 | 增加垂直位置/速度/加速度闭环 | `AC_PosControl U -> throttle` | Up/Down 符号、深度健康、surface/bottom |
| PosHold | body forward/lateral 速度意图 | 增加水平 NE 位置/速度闭环 | body -> NE -> PosControl -> body 力 | yaw 坐标转换、位置失效和目标重置 |
| Guided | GCS 给定位置/速度/姿态目标 | 按目标类型选择控制器 | MAVLink -> Guided submode -> WPNav/PosControl | frame、时间戳、3 秒超时、限幅 |

### 6.1 Manual：最短输入链

[`ModeManual::run()`](../ArduSub/mode_manual.cpp) 是检查 RC/joystick、归一化和混控方向的最短路径：

```text
roll/pitch/yaw/throttle/forward/lateral
    -> motors.set_*
    -> AP_Motors6DOF
```

Manual 适合定位输入和 frame 问题，但不能代表稳定模式的闭环行为。

### 6.2 Stabilize：姿态目标和 rate controller

roll/pitch 被换算成目标倾角；yaw 有杆量时给角速度，松杆后经过短暂减速再保持航向。fast loop 的 `rate_controller_run()` 使用 gyro 反馈生成 roll/pitch/yaw 控制量。调手感时应区分输入映射、姿态外环、rate PID、滤波和 motor limit。

### 6.3 AltHold：垂直级联控制

[`ModeAlthold::run()`](../ArduSub/mode_althold.cpp) 当前分为：

```text
run_pre() -> control_depth() -> run_post()
```

驾驶员 throttle 被解释为升沉速度，控制器再维护垂直速度和位置目标。看到 `D_*` 命名和 `get_position_z_up_cm()` 时必须逐个核对轴和正方向，不能根据字母猜符号。

### 6.4 PosHold：body 输入与 NE 闭环

```text
pilot forward/lateral
    -> body velocity
    -> ahrs.body_to_earth2D()
    -> North/East target
    -> AC_PosControl
    -> translate_pos_control_rp()
    -> body lateral/forward
```

同一个 North 误差在不同 yaw 下会映射成不同的艇体前后/左右推力。理解 PosHold 的关键不是先看 PID，而是先把两次坐标转换画清楚。

### 6.5 Guided：四种目标接口

| Guided 子模式 | 目标 | 主要消费者 |
|---|---|---|
| `Guided_WP` | 位置/航点 | `AC_WPNav` |
| `Guided_Velocity` | 速度 | `AC_PosControl` velocity input |
| `Guided_PosVel` | 位置 + 速度 | `AC_PosControl` position/velocity input |
| `Guided_Angle` | 姿态 + 升沉速度 | 姿态与垂直控制器 |

位置、速度和姿态请求都有各自的更新时刻与超时处理。修改消息行为时要从消息 frame、mask 和单位一直追到目标整形与最终 motors 输出。

## 7. 推进器状态机、6DOF 混控与输出

### 7.1 模式只提出 desired spool state

模式调用 `set_desired_spool_state()` 时提出：

- `SHUT_DOWN`：请求停止。
- `GROUND_IDLE`：请求低能量待命。
- `THROTTLE_UNLIMITED`：请求进入正常推力范围。

这不是最终许可。`AP_MotorsMulticopter` 会用 armed、interlock、safety、safe-time 和 block 条件推进 actual spool state：

```text
SHUT_DOWN
    <-> GROUND_IDLE
    <-> SPOOLING_UP / SPOOLING_DOWN
    <-> THROTTLE_UNLIMITED
```

### 7.2 当前车辆输出链

[`Sub::motors_output()`](../ArduSub/motors.cpp) 处理 motor-test/normal path、interlock 和 SRV cork/push，并调用 `motors.output()`。当前 4.7.0 还会在调用前清除 ArduSub spool-up block，这是基线逻辑，不能脱离父类状态机单独删除。

`motors.output()` 继续执行：

1. throttle filter 和电池/推力状态更新。
2. `output_logic()` 推进 spool 状态。
3. `output_armed_stabilizing()` 计算受允许的控制推力。
4. frame 补偿、6DOF 混控和饱和限制。
5. `output_to_motors()` 转换为通道输出。
6. SRV/HAL 写出。

### 7.3 六自由度系数

`AP_Motors6DOF::add_motor_raw_6dof()` 为每个推进器记录：

```text
roll, pitch, yaw, throttle(vertical), forward, lateral
```

检查推进器方向时要同时核对 `FRAME_CONFIG`、motor number、testing order、六列 factor、`MOT_n_DIRECTION`、`SERVOx_FUNCTION`、接线和 ESC 中值/范围。

### 7.4 输出故障诊断顺序

```text
输入
 -> 模式目标
 -> 状态估计
 -> 控制器目标/输出
 -> desired/actual spool state
 -> 6DOF mixer/limit
 -> SRV function/PWM
 -> HAL timer/DMA/pin
 -> ESC/接线/推进器
```

不能通过绕过 arming、interlock、failsafe 或 safety 来验证最后一级。

## 8. 轨迹、目标整形与 Guided

模式文件主要选择目标和控制器。速度、加速度、jerk 与 S 曲线/输入整形主要位于 `AC_WPNav`、`AC_PosControl` 和公共 shaping 函数，不在 `mode_auto.cpp` 中集中实现。

### 8.1 先确定目标类型

| 目标 | 常见入口 | 必查约束 |
|---|---|---|
| 位置 | Guided WP、mission waypoint | 最大速度、加速度、jerk、停止点和到达判定 |
| 速度 | Guided Velocity、PosHold 输入 | 加速度、jerk、命令超时和位置稳定状态 |
| 位置 + 速度 | Guided PosVel | 两种目标一致性、积分和超时后的速度归零 |
| 加速度 | `AC_PosControl::input_*accel*` | jerk、姿态能力和 motor saturation |
| 姿态 + 升沉 | Guided Angle | 倾角、升沉速度、深度健康和消息超时 |

“向前移动”可以是 body-forward 速度、NE 速度、NE 位置或 waypoint；它们的坐标和停止行为不同。

### 8.2 记录轨迹行为的证据

- 外部命令时间戳和频率。
- 原始及整形后的 position/velocity/acceleration。
- 实际位置、速度、姿态和 yaw。
- 速度、加速度、jerk、snap 与倾角限制。
- controller limit、integrator 和 motor limit flags。
- 命令超时、position health 变化和目标重置。

SITL 可以证明软件路径与约束，不能替代真实水动力、推进器死区和机体耦合验证。

## 9. 公共库管理与代码归属

### 9.1 前缀是职责线索，不是依赖边界

- `AP_*`：跨车辆基础设施、传感器、估计、任务和 HAL 上层。
- `AC_*`：主要源于 Copter 的姿态、位置、航点等控制库，也被 Sub 复用。
- `AR_*`：主要面向 Rover，不应为了命名统一引入 ArduSub。

最终依赖证据来自 `wscript`、include、feature guard、Waf 任务图和链接结果。

### 9.2 需求应该放在哪一层

| 需求 | 首选位置 | 原因 |
|---|---|---|
| 单一 Sub 模式的进入、目标和降级 | `ArduSub/mode_*.cpp` | 属于车辆策略 |
| 多个模式共享的车辆坐标换算 | `ArduSub/Attitude.cpp` 或明确车辆辅助接口 | 仍依赖 Sub 语义 |
| 通用姿态、位置或轨迹算法 | 现有 `AC_*` | 算法不应知道具体模式 |
| 通用传感器设备 | 对应 `AP_*` frontend/backend | 数据语义与车辆无关 |
| MCU 引脚、总线、timer 和设备实例 | `AP_HAL_ChibiOS/hwdef/<board>/` | 属于硬件描述 |
| 通用 MAVLink 行为 | `GCS_MAVLink` 或现有消息入口 | 协议跨车辆 |
| 伴随计算机程序 | 独立仓库 | 不属于实时飞控固件 |

### 9.3 最小复用顺序

```text
配置已有能力
    -> ArduSub 薄适配
    -> 小幅扩展现有公共 API
    -> 为现有 frontend 增加 backend
    -> 最后才新建公共库
```

新 sensor backend 应保持 frontend 的坐标、单位、时间、质量和健康语义；消费者依赖 frontend，不依赖具体设备型号。

### 9.4 公共库安全规则

- 不改变已有 `AP_GROUPINFO` 索引。
- guard 关闭时声明、实现、日志和调用点仍可编译。
- 核心组件不能依赖可选组件。
- 新依赖不能反向 include `ArduSub/Sub.h`。
- 删除依赖必须有 Waf dependency closure、固件尺寸、SITL 和目标板证据。
- parser、frontend/backend、consumer integration、SITL、Pixhawk4 和硬件测试分别覆盖不同风险。

## 10. WSL 构建、测试与发布

### 10.1 构建系统

- 根 `waf`：启动器。
- `modules/waf`：通用 Waf 引擎。
- 根 `wscript`：板卡、工具链和全局配置。
- `Tools/ardupilotwaf`：车辆、库、板卡和固件规则。
- `ArduSub/wscript`：Sub 静态库、程序和直接依赖。

禁止使用 `sudo ./waf`。`build/<board>/` 是可再生产物，不是精简源码。

### 10.2 基线命令

```bash
./waf configure --board sitl
./waf sub -j"$(nproc)"

./waf configure --board Pixhawk4
./waf sub -j"$(nproc)"

Tools/autotest/autotest.py build.Sub test.Sub
```

输出分别位于 `build/sitl/bin/ardusub` 和 `build/Pixhawk4/bin/ardusub.apj`。

### 10.3 默认验证矩阵

| 变更 | 最低验证 |
|---|---|
| Markdown | 链接、表格/围栏、`git diff --check` |
| 参数/非控制逻辑 | SITL build、相关测试、Pixhawk4 build、参数兼容检查 |
| 模式/控制算法 | SITL build、针对性 autotest、日志、Pixhawk4 build |
| AP_Motors/failsafe | 上述全部，加拆桨/隔离台架和回退固件 |
| 传感器 backend | parser/unit、frontend health/timeout、consumer integration、目标板 build |
| hwdef/新板卡 | bootloader、board build、接口电气验证和逐级上电 |

每次记录精确 SHA、环境、命令、退出状态、固件路径、尺寸、已执行/未执行测试、硬件条件和回退方式。

## 11. 安全变更工作流

### 11.1 开始前冻结事实

```bash
git status --short --branch
git rev-parse HEAD
git submodule status
```

区分任务修改、用户已有内容、未跟踪资料和构建输出。不得覆盖或清理不属于当前任务的内容。

### 11.2 先画真实数据链

```text
input/message/sensor
 -> mode selection/init
 -> target generation
 -> coordinate/unit conversion
 -> attitude/position controller
 -> desired/actual spool state
 -> 6DOF mixer/limits
 -> SRV/HAL
 -> log/test evidence
```

只修改最小必要节点，不为一个小需求重构整条控制体系。

### 11.3 变更规则

- 参数：新增使用未占用索引，完整记录范围、单位、默认值和持久化兼容。
- Feature：保留 guard，验证 enabled/disabled，核心不依赖可选组件。
- HAL：板级特性进入 hwdef/HAL，不在模式里判断板卡名或写 GPIO。
- 子模块：不直接修改 `modules/`。
- 提交：一项逻辑变更一个提交，不混入全局格式化或无关移动。
- 测试：未执行的项目明确写“未执行”，不能推测通过。

### 11.4 推进器实机红线

- 拆桨、拆除负载或物理隔离推进器。
- 准备 hardware safety、disarm、通信中断和独立断电。
- 一次只验证一个通道，记录功能、方向、中性、最小和最大值。
- 完成空载后再进入低功率系留水池。

### 11.5 回退

回退至少包含前一固件 SHA、参数备份、触发条件、负责人，以及通道方向/failsafe 的复核步骤。不要用 `git reset --hard`、`git checkout --`、`git clean` 或强制推送代替产品回退方案。

## 12. Pixhawk 类 STM32/ChibiOS 板卡移植

本仓库只把 Pixhawk 类实时飞控作为固件目标。Linux SBC 默认是伴随计算机，不在这里扩展成飞控主控。

### 12.1 设计输入

- MCU、封装、flash/RAM、时钟和启动方式。
- 电源、brownout、watchdog、reset、hardware safety 和 SWD 恢复。
- IMU、罗盘、压力/深度、存储和总线拓扑。
- UART/I2C/SPI/CAN/USB 引脚与 DMA 冲突。
- PWM/DSHOT timer、channel、DMA、IO 电平和 ESC 接口。

### 12.2 Board definition

```text
libraries/AP_HAL_ChibiOS/hwdef/<board>/
    hwdef.dat
    hwdef-bl.dat
    defaults.parm       # 仅在确有板级默认需求时
    README.md
```

选择同 MCU 和相似传感器拓扑的参考板，但必须逐项对照原理图。

### 12.3 构建顺序

```bash
./waf configure --board <board> --bootloader
./waf bootloader

./waf configure --board <board>
./waf sub -j"$(nproc)"
```

### 12.4 分阶段验证

1. SWD、reset、bootloader 和 console。
2. 时钟、USB、flash、参数存储和 SD。
3. I2C/SPI/UART/CAN 电平、时钟和设备枚举。
4. IMU 方向、采样率、DRDY、温度和振动。
5. 压力/深度、罗盘和其他传感器。
6. RC/MAVLink、hardware safety、arming 和 failsafe。
7. 示波器验证输出，不接推进器负载。
8. 拆桨逐通道测试。
9. 低功率系留水池回归。

HAL/hwdef 应解决绝大多数硬件适配。只有现有抽象确实无法表达硬件能力时，才评估可复用的 HAL 扩展。

## 13. 阅读记录与掌握标准

研究任何行为时，使用同一张记录表：

| 项目 | 必须记录 |
|---|---|
| 输入 | 来源、范围、频率、超时 |
| 状态 | 估计量、健康条件、reset |
| 坐标 | body/NE/NED/Up/Down、正方向 |
| 单位 | cm、cm/s、rad、centi-degree、归一化力等 |
| 模式 | 进入条件、`init()`、`run()`、退出和降级 |
| 目标 | position/velocity/acceleration/attitude/rate/force |
| 控制器 | 限速、滤波、integrator、saturation |
| Motors | desired/actual spool、armed/interlock、mix factors |
| 输出 | SRV function、PWM/协议、物理通道 |
| 证据 | 源码函数、日志、SITL、板卡和实机结果 |

开始具体案例前，应能独立回答：

- `flightmode->run()` 如何分派到具体模式？
- rate controller 为什么在调度顺序中先于本周期 mode update？
- PosHold 为什么既使用 NE，又输出 body forward/lateral？
- desired spool state 为什么不等于立即允许 PWM？
- `FRAME_CONFIG` 如何选择 6DOF 系数表？
- 新模式、传感器 backend、控制算法、参数和板卡定义分别应放在哪一层？

达到这些标准后，不再继续横向阅读目录，而应从[案例候选表](10_case_catalog.md)选择一个小需求完成端到端实践。
