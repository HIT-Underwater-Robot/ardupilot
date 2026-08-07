# 第 7 章：MAVLink 接收、处理与发送

前面已经知道 MAVLink XML 如何生成协议代码。本章只研究生成代码进入车辆后怎样运行。

先看接收主线：

~~~text
TELEM2 电平和字节
    ↓
USART3 / HAL UARTDriver
    ↓
GCS::update_receive
    ↓
逐字节 mavlink_frame_char_buffer
    ↓
校验、路由、来源过滤
    ↓
GCS_MAVLINK_Sub::handle_message
    ├─ ArduSub 专属处理
    └─ GCS_MAVLINK::handle_message 公共处理
~~~

## 1. 文件地图

1. **wscript**：mavgen 构建任务；
2. **modules/mavlink/message_definitions/v1.0/all.xml**：本仓库生成入口；
3. **Tools/ardupilotwaf/mavgen.py**：调用生成器；
4. **libraries/GCS_MAVLink/GCS_Common.cpp**：字节解析、路由和公共消息；
5. **ArduSub/GCS_MAVLink_Sub.cpp**：Sub 专属消息；
6. **ArduSub/GCS_Sub.cpp**：车辆 GCS 对象和流配置；
7. **build/Pixhawk1/libraries/GCS_MAVLink/include/**：生成头文件。

不要编辑 build 下的生成头文件，也不要直接修改 modules 子模块来保存本仓库的临时教学笔记。

## 2. XML 不会在 Pixhawk 上解析

当前根 wscript 把 all.xml 交给 mavgen，生成 MAVLink 2.0 C 头文件。

~~~text
all.xml
    ↓ include 其他 dialect
mavgen
    ↓
消息结构体、ID、CRC 信息、encode/decode/send 函数
    ↓
C++ 编译器
    ↓
ardusub 固件
~~~

Pixhawk 运行的是生成后的 C/C++ 代码，而不是 XML 解释器。

all.xml 是聚合入口，ardupilotmega.xml 又 include common.xml 和若干厂商 dialect。被生成不代表每种消息都被 ArduSub 业务逻辑主动使用；生成协议定义与实际处理函数是两回事。

## 3. 一个消息有三种身份

以 ODOMETRY 为例：

- XML 定义：字段、类型、单位、枚举和 CRC；
- 线上的 frame：消息头、payload、校验和、可选签名；
- C 结构体：mavlink_odometry_t。

decode 函数只完成 payload 到结构体的解包，不会自动把数据送入 EKF。

~~~cpp
mavlink_odometry_t m;
mavlink_msg_odometry_decode(&msg, &m);
~~~

解包之后仍需要业务代码检查坐标系、质量和配置，再调用 AP_VisualOdom。

## 4. 字节如何组成消息

GCS_MAVLINK::update_receive()：

1. 查询 UART 当前 available 字节数；
2. 逐字节 read；
3. 把字节交给 mavlink_frame_char_buffer；
4. incomplete 时继续收；
5. 得到完整或坏帧时进入 raw_packetReceived；
6. 避免在一次调度中占用过长时间。

串口 read 返回一个字节，并不等于收到一条消息。解析器要跨多次循环保存状态，直到头、payload 和校验完整。

## 5. 完整帧先经过哪些门

raw_packetReceived 的次序很重要：

~~~text
帧完整性结果
    ↓
packetReceived 更新链路状态/MAVLink 版本
    ↓
MAVLink routing 检查与必要转发
    ↓
脚本或 Follow 等旁路观察者
    ↓
accept_packet 来源过滤
    ↓
handle_message 本机处理
~~~

### 5.1 CRC

坏 CRC 表示帧内容不可信，通常不能进入常规业务处理。CRC 只能检测传输错误，不能证明发送者可信。

### 5.2 routing

Pixhawk 可能连接多个 MAVLink 组件和多个链路。路由表学习 system ID、component ID 和 channel，并决定：

- 本机是否应处理；
- 是否应转发给其他通道；
- 消息目标是否是另一个组件。

所以“Pixhawk 收到”不等于“ArduSub 一定消费”。

### 5.3 accept_packet

它还可以根据 system ID 等规则拒绝不应接受的数据。控制消息通常还会在车辆处理器中做模式、来源和字段检查。

## 6. 公共处理与 Sub 专属处理

GCS_MAVLINK_Sub 继承 GCS_MAVLINK，并 override handle_message。

~~~text
GCS_MAVLINK_Sub::handle_message
    ↓ switch(msg.msgid)
    ├─ RC_CHANNELS_OVERRIDE
    ├─ SET_ATTITUDE_TARGET
    ├─ SET_POSITION_TARGET_LOCAL_NED
    ├─ SET_POSITION_TARGET_GLOBAL_INT
    ├─ SYS_STATUS 远程漏水状态
    └─ default
           ↓
       GCS_MAVLINK::handle_message
~~~

公共层处理 ODOMETRY、参数、任务协议、COMMAND_LONG/INT、心跳、日志、视觉里程计等跨车辆功能。

车辆层只处理需要 ArduSub 模式和控制器知识的消息。这个边界避免在公共库中硬编码 sub.mode_guided。

## 7. 位置目标消息不是里程计

必须区分：

| 消息 | 含义 | 数据方向 |
|---|---|---|
| ODOMETRY | 我估计车辆现在在哪里、怎样运动 | 估计器 → Pixhawk |
| SET_POSITION_TARGET_LOCAL_NED | 我希望车辆去哪里或以多快运动 | 规划器 → 控制模式 |
| LOCAL_POSITION_NED | Pixhawk 报告当前局部位置 | Pixhawk → 外部系统 |

把 ODOMETRY 当作目标点，会污染状态估计；把 SET_POSITION_TARGET 当作测量，控制器不会获得真实反馈。

## 8. LOCAL_NED 目标怎样进入 Guided

Sub 专属处理器先检查：

- 当前是否 Guided，或 Auto 中的 NavGuided；
- coordinate_frame 是否在支持列表；
- type_mask 中位置、速度、加速度、yaw、yaw rate 哪些有效。

之后完成：

- 米转换为厘米；
- NED 的 Down 转为内部位置控制常用的 Up；
- body frame 的 XY 旋转到 North/East；
- offset frame 加上当前位置；
- 弧度转换为 centidegree。

最后只接受当前代码明确支持的组合：

- 位置 + 速度；
- 仅速度；
- 仅位置；
- 加速度目标在这里并未形成相应处理分支。

type_mask 不是随便填零。发送端必须明确每个字段是否有效。

## 9. SET_ATTITUDE_TARGET 的检查

该消息包含四元数、角速度、推力和 type mask。Sub 当前处理逻辑会根据 mask 判断姿态和推力字段是否应使用，并把 0～1 的 thrust 以 0.5 为中点转换为上升/下降速度目标。

所以消息字段的物理含义还会受车辆实现解释，不能只看 XML 字段名。

## 10. COMMAND 与普通消息

COMMAND_LONG 或 COMMAND_INT 表达一次有确认语义的命令，通常返回 COMMAND_ACK。

SETPOINT 通常是连续更新的控制目标，超时与模式规则由车辆处理。

状态消息一般用于周期报告，不等同于带 ACK 的命令。

选择机制时问：

- 是否需要明确接受/拒绝结果？
- 是一次动作还是连续流？
- 丢一帧时能否由下一帧覆盖？
- 是否需要目标 system/component？

## 11. 发送链路

~~~text
模块请求发送某类 ap_message
    ↓
GCS_MAVLink 记录消息间隔或队列
    ↓
GCS::update_send 周期运行
    ↓
检查到期、时间预算和 UART 空间
    ↓
try_send_message
    ↓
mavlink_msg_xxx_send
    ↓
UART TX buffer
~~~

try_send_message 返回 false 往往表示当前没有足够发送空间，框架会根据消息类型和调度状态延后重试。

不能在 400 Hz 控制任务里无条件发送大量消息；串口带宽、TX buffer 和调度时间都是有限资源。

## 12. stream 和消息间隔

stream 是一组消息的传统配置方式，具体消息最终仍有发送间隔。地面站也可通过 MAV_CMD_SET_MESSAGE_INTERVAL 请求某个 message ID 的周期。

发送频率的结果可能同时受：

- 默认 stream 配置；
- SRx 参数；
- 单消息间隔请求；
- 链路带宽；
- 调度预算；
- 消息是否当前可生成。

“设为 50 Hz”是请求目标，不代表拥塞链路必定无丢帧地达到 50 Hz。

## 13. 树莓派/香橙派到 QGC

若链路是：

~~~text
传感器 → Pixhawk TELEM2
Pixhawk → 香橙派
香橙派 → QGC
~~~

需要明确每段职责：

- 传感器若说的不是 MAVLink，Pixhawk 必须有对应串口驱动；
- 若传感器本身发送 MAVLink，路由规则决定转发和本地处理；
- 香橙派可以路由 MAVLink，但不要重复伪造同一 system/component；
- QGC 能显示消息不代表 Pixhawk 已把它用于控制或 EKF。

最可靠的验证要同时看：链路字节、MAVLink Inspector、Pixhawk 日志和车辆行为。

## 14. 一条消息的排查模板

1. XML 中找到 message ID、字段、单位和 frame；
2. 在生成头文件确认 decode 函数存在；
3. 搜索 MAVLINK_MSG_ID_xxx；
4. 找到公共或 Sub handle_message 分支；
5. 记录每个拒绝条件；
6. 跟到最终对象方法；
7. 找健康状态和日志；
8. 检查发送 system/component、时间戳和频率。

## 15. 验收问题

1. XML 为什么不会被 Pixhawk 运行时解析？
2. 生成了某消息为何不代表 ArduSub 使用它？
3. 字节何时才成为 mavlink_message_t？
4. 路由和本机处理有什么区别？
5. Sub 不认识的消息去哪一层？
6. ODOMETRY 与 SET_POSITION_TARGET 有何根本区别？
7. type_mask 为什么必须认真填写？
8. try_send_message 为什么可能延后？
9. QGC 看到消息为何不能证明 EKF 已融合？
10. 如何从 TELEM2 一帧追踪到 Guided？
