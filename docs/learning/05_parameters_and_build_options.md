# 第 5 章：参数系统与编译开关

这一章解决两个经常混淆的问题：

1. 为什么修改 QGroundControl 中的参数，不需要重新编译固件？
2. 为什么关闭某些 AP_xxx_ENABLED 功能，必须重新编译？

~~~text
运行参数
    保存在持久化存储中
    启动时加载
    同一份固件可以取不同值

编译开关
    在预处理阶段决定代码是否进入固件
    改变后必须重新编译
~~~

## 1. 本章文件地图

1. **ArduSub/Parameters.h**：车辆参数成员和稳定 key；
2. **ArduSub/Parameters.cpp**：元数据表、默认值和迁移；
3. **libraries/AP_Param/AP_Param.h**：参数类型和登记宏；
4. **libraries/AP_Param/AP_Param.cpp**：查找、加载、保存和遍历；
5. **libraries/AP_Vehicle/AP_Vehicle.cpp**：启动加载位置；
6. **Tools/scripts/build_options.py**：编译期开关。

第一遍不要通读 AP_Param.cpp，只围绕本章列出的函数查找。

## 2. 为什么不用普通 int 和 float

ArduPilot 常见参数成员是 AP_Int8、AP_Int16、AP_Int32、AP_Float 和 AP_Vector3f。

这些包装类型一方面保存当前值，另一方面提供 get、set、save、set_and_save、set_default 和 set_and_default 等参数语义。

例如 AP_Float failsafe_pilot_input_timeout 仍可参与数值计算，但比裸 float 多了默认值和持久化管理能力。

## 3. C++ 成员不等于已经成为参数

在类里增加 AP_Float 成员，只是增加一个 C++ 对象。要成为可查找、保存并通过 MAVLink 枚举的参数，还要登记到参数表。

ArduSub 顶层表是 Sub::var_info，子对象自己的表通常是 SomeClass::var_info。

~~~text
Sub::var_info
    ├─ 单个车辆参数
    ├─ Parameters g
    ├─ ParametersG2 g2
    ├─ AP_SerialManager
    ├─ AP_Scheduler
    └─ 其他公共对象
            ↓
        各自 GroupInfo
            ↓
        具体 AP_Int/AP_Float 成员
~~~

## 4. 参数宏记录什么

概念形式：

~~~cpp
AP_GROUPINFO("NAME", index, ClassName, member, default_value)
~~~

它提供对外名称片段、组内稳定索引、成员在对象中的位置、成员类型和默认值。宏最终生成静态结构体数据，AP_Param 在运行时遍历它们。

ArduSub 的 GSCALAR 等辅助宏把 Sub、g 成员、Parameters 枚举 key 和默认值连接起来。

AP_SUBGROUPINFO 则把一个子对象接入参数树并添加前缀。例如 servo_channels 加 SERVO 前缀后形成 SERVOx_xxx 参数。

## 5. 默认值什么时候进入对象

许多类在构造函数中调用 AP_Param::setup_object_defaults(this, var_info)，依据 GroupInfo 设置编译时默认值。

~~~text
C++ 对象构造
    ↓
登记表默认值进入成员
    ↓
AP_Vehicle::setup()
    ↓
AP_Param::check_var_info()
    ↓
Sub::load_parameters()
    ↓
已保存值覆盖默认值
~~~

所以改源码默认值不会自动覆盖用户已经保存的旧参数。

## 6. 默认值、当前值和保存值

| 层次 | 含义 |
|---|---|
| default | 固件提供的缺省配置 |
| current | RAM 中当前使用的值 |
| saved | 持久化存储中跨重启保留的值 |

概念区别：

- set：改变当前值；
- save：把当前值写入持久化存储；
- set_and_save：改变并保存；
- set_default：调整默认值语义；
- set_and_default：同时调整当前值和默认值语义。

在安全逻辑中使用前仍要查看具体方法实现和调用上下文。

## 7. 为什么参数索引不能重新编号

参数对人显示名称，但持久化定位还依赖顶层 key、组索引、group element 和类型。这些数字构成固件与旧配置之间的存储契约。

危险操作包括：

- 为列表整齐而重排枚举值；
- 重用标有 was 的旧索引；
- 改变已有条目类型；
- 移动参数却不写迁移；
- 删除旧条目后立即复用编号。

源码中的 “1 was AP_Stats” 不是垃圾，而是历史兼容提示。

## 8. format version 不是万能迁移

ArduSub 的 load_parameters() 会执行 convert_old_parameters、convert_class、convert_g2_objects、convert_toplevel_objects，以及控制器自己的 convert_parameters。

真实用户需要保留标定值、串口配置、PID 和安全配置，因此成熟工程不能只靠清空全部参数升级。

新增或移动参数时应明确旧 key、旧组索引、旧类型、新名称、新位置和转换时机。

## 9. g 与 g2

g 和 g2 不是第一版与第二版配置文件，而是两个参数容器。g 保存较早建立的车辆参数，g2 为后来扩展提供另一组空间和子对象入口。

选择位置必须遵循现有兼容布局，不应按个人偏好重排。

## 10. QGroundControl 如何看到参数

飞控通过 MAVLink 参数消息列举、读取和设置参数值，GCS_MAVLink 使用 AP_Param 遍历参数树。

~~~text
QGC 请求参数
    ↓ MAVLink
GCS_MAVLink 参数处理
    ↓
AP_Param 查找或遍历
    ↓
读取当前值
    ↓ MAVLink
QGC 显示
~~~

而 Parameters.cpp 中的 @Param、@DisplayName、@Description、@Units、@Range 等注释会被工具提取为参数元数据，供文档或地面站显示友好说明。

MCU 不会在运行时解析这些 C++ 注释。参数数值协议与说明元数据是两条相关但不同的链。

## 11. 参数名和元数据约束

当前贡献规则要求参数全名不超过 16 个字符，前缀也计入完整名称。

新增前检查：

- 完整名称和冲突；
- 单位、范围、增量；
- 保守默认值；
- 是否需要重启；
- Standard 或 Advanced；
- 是否只适用于 ArduSub。

## 12. 运行参数与编译开关

运行参数的代码已经存在于固件，值可修改；关闭功能未必释放 Flash。

编译开关使用条件预处理：

~~~cpp
#if AP_FEATURE_ENABLED
    ...
#endif
~~~

它决定类成员、任务、驱动和参数是否参与编译，修改后必须重新 configure/build，并可能改变 Flash、RAM 和依赖。

不能用运行参数代替所有编译开关，也不能为省空间移除条件编译保护。

## 13. build_options.py

Tools/scripts/build_options.py 描述自定义构建可控制的功能选项和依赖关系，包括类别、名称、define、默认值和依赖。

核心功能不能反向依赖一个可能关闭的可选功能。关闭功能后要重新编译目标并执行相关测试。

## 14. 安全新增参数的步骤

1. 明确所属类；
2. 加入正确 AP_Param 包装成员；
3. 选择未用且兼容的索引；
4. 在 Info 或 GroupInfo 登记；
5. 写完整元数据；
6. 给保守默认值；
7. 在初始化或周期逻辑读取；
8. 若移动旧参数则写转换；
9. 编译 SITL 和目标板；
10. 经 MAVLink 列举、设置、保存并重启读取；
11. 测试边界和非法值；
12. 检查参数文档生成。

## 15. 不改源码的跟踪练习

选择 FS_PILOT_TIMEOUT：

1. 在 Parameters.cpp 找 GSCALAR；
2. 记录成员名、对外名和默认值；
3. 在 Parameters.h 找成员类型；
4. 在 failsafe.cpp 找读取位置；
5. 确认单位；
6. 找到它如何与最后输入时间比较；
7. 分清检测条件与动作参数。

目标是建立：

~~~text
参数声明
→ 默认值
→ 启动加载
→ 运行读取
→ 行为结果
~~~

## 16. 常见误区

- 改默认值后，旧设备不一定采用新值；
- 名称不变也不能随意改内部索引；
- 参数注释还会用于文档和地面站元数据；
- ENABLE 设为零不等于代码没有编进固件；
- 旧参数占位不能因看似无用而直接复用。

## 17. 本章验收问题

1. AP_Float 相比 float 多了什么？
2. 参数成员为什么还要登记到 var_info？
3. Info 和 GroupInfo 怎样理解？
4. 三种参数值有什么区别？
5. 为什么改默认值不一定影响旧设备？
6. 为什么不能重排索引？
7. g 和 g2 是什么关系？
8. QGC 获取数值与说明有什么区别？
9. 运行参数和编译开关何时生效？
10. 新增参数要验证哪些环节？
