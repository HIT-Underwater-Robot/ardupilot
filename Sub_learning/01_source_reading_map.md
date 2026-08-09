# 源码阅读地图：从入口到推进器输出

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
| ExternalNav | [`AP_VisualOdom.h`](../libraries/AP_VisualOdom/AP_VisualOdom.h)、[`AP_VisualOdom.cpp`](../libraries/AP_VisualOdom/AP_VisualOdom.cpp)、[`AP_VisualOdom_MAV.cpp`](../libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp) | 接收视觉/外部 odometry，处理 backend、延迟、质量、超时和坐标数据 | DVL 等外部融合结果可经 MAVLink ODOMETRY 进入 EKF，而不是在模式里临时积分 |
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

## 6. 库名前缀只表示职责线索

- `AP_*`：跨车辆基础设施、传感器、状态估计、任务、HAL 上层和通用库。
- `AC_*`：主要源于 Copter 的姿态、位置、航点和避障控制，也被 ArduSub 复用。
- `AR_*`：主要面向 Rover，不应为了命名统一而引入 ArduSub。

前缀不是严格依赖边界。[ArduSub/wscript](../ArduSub/wscript) 明确列出 `AC_AttitudeControl`、`AC_WPNav`、`AP_InertialNav`、`AP_Motors` 等直接库，并叠加 `ap_common_vehicle_libraries()` 的公共依赖。判断依赖必须看 `wscript`、include、feature guard 和链接结果。

## 7. 每次阅读使用同一张记录表

研究一个行为时，至少记录：

1. 输入来源和有效范围。
2. 当前模式与进入条件。
3. 坐标系、正方向和单位。
4. 目标类型及其限制器。
5. 使用的状态估计量和有效性检查。
6. 控制器输出怎样转换到 roll/pitch/yaw/throttle/forward/lateral。
7. armed、interlock、failsafe、safety switch 和 spool 状态。
8. 最终 SRV 功能与物理推进器通道。
9. 能证明结论的源码函数、SITL 测试或日志字段。
