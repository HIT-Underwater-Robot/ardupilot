# 第 13 章：Guided、Auto 与任务系统

Guided 和 Auto 都能自动运动，但“计划保存在谁手里”不同：

~~~text
Guided
    香橙派保存计划
    持续发送当前 setpoint

Auto
    Pixhawk 保存 Mission
    自己启动、验证并推进任务项
~~~

## 1. 文件地图

1. **ArduSub/mode_guided.cpp**：即时目标；
2. **ArduSub/mode_auto.cpp**：Auto 子状态；
3. **ArduSub/commands.cpp**：Home；
4. **ArduSub/commands_logic.cpp**：任务项启动与完成判断；
5. **libraries/AP_Mission/AP_Mission.h/.cpp**：任务存储和状态机；
6. **libraries/GCS_MAVLink/**：Mission Protocol。

## 2. Mission 如何进入 Pixhawk

QGC 不会把一份 JSON 或规划程序直接放进 ModeAuto。它通过 MAVLink Mission Protocol 协商并上传 MISSION_ITEM_INT 等任务项。

~~~text
QGC 任务编辑器
    ↓ Mission Protocol
GCS_MAVLink
    ↓ 校验和转换
AP_Mission::Mission_Command
    ↓
持久化 mission storage
~~~

任务项保存后，即使伴随计算机暂时断开，Pixhawk 仍可按当前 Auto 和 failsafe 配置继续处理。

## 3. Mission_Command 是内部表示

MAVLink mission item 包含 command、frame、参数和位置。AP_Mission 会：

- 检查 item 索引和类型；
- 把 MAVLink 字段转换为紧凑内部结构；
- 写入任务存储；
- 读取时再转换回 MAVLink。

上传成功不等于该车辆一定能执行每一种 MAV_CMD。车辆 start_command 仍会判断是否支持。

## 4. 进入 Auto 的条件

ModeAuto::init 当前检查：

- position_ok；
- mission.present。

然后：

1. 初始 auto_mode 设为 Loiter；
2. 清理上一模式 ROI/yaw 状态；
3. 初始化 WPNav；
4. 清 Guided limits；
5. mission.start_or_resume。

MIS_RESTART 决定重新从头开始还是尝试恢复。

## 5. AP_Mission 的状态

概念上包括：

- stopped；
- running；
- complete。

start 会 reset 到开头；stop 后 update 不再推进；resume 从保存位置继续；complete 会调用车辆提供的 mission complete 回调。

具体行为还受 mission 修改、跳转计数、resume repeat distance 和参数影响。

## 6. mission.update 做什么

ModeAuto::run 每个主循环调用 sub.mission.update()。

~~~text
mission.update
    ↓
当前是否 running 且有任务？
    ↓
若没有 active NAV，寻找并启动下一项
    ↓
verify 当前 NAV
    ├─ 未完成：保留
    └─ 完成：推进下一 NAV
    ↓
处理与 NAV 关联的 DO/CONDITION
    ↓
到末尾：complete
~~~

AP_Mission 管理通用队列；实际水下动作由车辆 callback 完成。

## 7. start 与 verify 是一对

Sub::start_command 根据 cmd.id 调用：

- do_nav_wp；
- do_surface；
- do_RTL；
- do_loiter；
- do_circle；
- do_nav_delay；
- do_change_speed；
- do_set_home；
- do_roi 等。

Sub::verify_command 再调用对应 verify 函数。

~~~text
start_command
    建立目标、计时器或子状态

每轮 verify_command
    检查目标是否完成
~~~

DO 类一次性命令通常执行后即可视作完成；NAV 命令需要持续导航并等待到达。

## 8. 高度 frame 的水下含义

start_command 会检查 altitude frame 和符号：

- ABOVE_HOME 的目标高度在水下通常应为非正；
- ABOVE_TERRAIN 代表距地形高度并要求相应正值；
- 其他不支持 frame 会拒绝。

同一个 alt 数字若 frame 不同，物理意义完全不同。

## 9. Auto 子状态

ModeAuto 自己还有：

- Auto_WP；
- Auto_CircleMoveToEdge；
- Auto_Circle；
- Auto_NavGuided；
- Auto_Loiter；
- Auto_TerrainRecover。

Mission 选择当前命令，ModeAuto 再运行对应控制器。这是“任务状态机”和“运动控制子状态”两层。

## 10. Mission 完成后

Sub::exit_mission：

1. 触发 mission_complete Notify；
2. 尝试进入 Auto Loiter；
3. 若失败则切换 AltHold。

它不会默认自动 disarm。水下任务完成行为必须结合实际安全需求验证。

## 11. 离开 Auto

模式切换的退出逻辑会在离开 AUTO 时停止正在运行的 mission，并恢复相机云台等状态。

停止不等于清空任务。再次进入时 start/resume 行为取决于任务状态和 MIS_RESTART。

## 12. Guided 的实时契约

Guided 不保存完整路径，它保存当前子模式和目标：

- destination；
- velocity；
- pos+vel；
- angle。

外部规划器应负责：

- 定期刷新目标；
- 限制跳变；
- 监控飞控是否仍在 Guided；
- 监控位置估计；
- 处理 ACK/状态与链路中断；
- 不用 setpoint 消息代替心跳策略。

## 13. 牛耕式巡检放在哪里

两种都合理：

### 香橙派规划

动态避障、多传感器地图和实时重规划放在香橙派，持续给 Guided 局部目标。

### QGC/Auto

水面 GPS 航线或无需动态规划的任务可上传 Mission，由 Pixhawk 执行。

保留 Auto 提供独立任务能力；常规水下智能规划仍可使用 Guided。

## 14. Auto_NavGuided

任务命令可暂时允许外部导航计算机接管即时目标，并有 Guided limits。它说明 Auto 与 Guided 并非只能二选一，但这种混合模式必须理解：

- 谁开始和结束接管；
- timeout 和水平/深度边界；
- Mission 何时恢复；
- 失联走哪条 failsafe。

## 15. 选择矩阵

| 问题 | Guided | Auto |
|---|---|---|
| 完整计划存放 | 伴随计算机 | Pixhawk |
| 动态重规划 | 强 | 较弱/按任务项 |
| 外部链路依赖 | 高 | 上传后较低 |
| 水面固定航线 | 可用 | 很适合 |
| 水下复杂感知规划 | 很适合 | 可作备用 |
| 失联后继续计划 | 通常不能 | 取决于任务/failsafe |

## 16. 验收问题

1. Mission Protocol 和普通 setpoint 有何区别？
2. 上传成功为何不代表车辆支持命令？
3. start_command 与 verify_command 如何配合？
4. NAV 与 DO 命令为什么完成语义不同？
5. ModeAuto 与 AP_Mission 各负责哪层？
6. MIS_RESTART 影响什么？
7. 任务完成后当前代码做什么？
8. 离开 Auto 是否会清空任务？
9. 牛耕式巡检何时选 Guided，何时选 Auto？
10. Auto_NavGuided 需要哪些边界？
