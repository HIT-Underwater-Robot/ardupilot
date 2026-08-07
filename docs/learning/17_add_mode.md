# 第 17 章：新增控制模式

新增模式的第一原则是复用已有控制器和安全框架，而不是在 Mode::run 中自己写 PWM。

## 1. 先写模式合同

在编码前回答：

- 输入来自驾驶、Mission、MAVLink 还是脚本？
- 控制目标是姿态、速度、位置还是距离？
- 需要水平位置吗？
- 需要深度吗？
- 哪些传感器失效后不能继续？
- setpoint 多久超时？
- 未解锁时做什么？
- 进入时怎样消除目标跳变？
- 离开后哪些状态要恢复？
- failsafe 应切到哪里？

## 2. 选择最接近的模式

| 新需求 | 优先参考 |
|---|---|
| 直接六轴驾驶 | Manual |
| 姿态稳定 | Stabilize |
| 角速度控制 | Acro |
| 深度保持 | AltHold |
| 离底高度 | SurfTrak |
| 局部位置保持 | PosHold |
| 伴随计算机目标 | Guided |
| 任务项 | Auto |

继承或组合时应只复用语义一致的部分。

## 3. 编号

Mode::Number 是协议和日志接口。新增前：

- 查当前 ArduPilot 模式编号；
- 查保留编号；
- 查 QGC 是否识别；
- 考虑与上游合并还是仅私有使用；
- 不改已有编号。

私有实验编号也要在项目文档中登记，避免团队冲突。

## 4. 文件改动地图

典型需要：

1. mode.h：Number 和类；
2. 新的 mode_xxx.cpp；
3. Sub.h：模式对象成员；
4. mode.cpp：mode_from_mode_num；
5. Parameters.cpp：可选参数和 FLTMODE 元数据；
6. GCS/Notify：名称和 capability；
7. Log：必要目标；
8. autotest：切换和行为。

ArduSub 的车辆静态库会收集车辆源文件，但仍应通过构建输出确认新 cpp 实际编译。

## 5. 类的最小接口

模式至少实现：

~~~cpp
bool init(bool ignore_checks) override;
void run() override;
bool requires_GPS() const override;
bool requires_altitude() const override;
bool allows_arming(bool from_gcs) const override;
const char *name() const override;
const char *name4() const override;
Mode::Number number() const override;
~~~

不要把 requires_GPS 为 false 当成绕过 position health；如果 run 使用位置，就必须诚实返回依赖。

## 6. init 应做什么

- 验证当前估计和传感器；
- 把目标初始化为当前状态；
- 初始化相应控制器；
- 清旧积分或调用合适 relax/init；
- 记录更新时间；
- 拒绝无法安全进入的条件。

平滑进入的常用思路：

~~~text
target_position = current_position
target_velocity = 0
target_yaw = current_yaw
~~~

但必须按具体模式需求选择，不能机械复制。

## 7. run 的标准骨架

~~~text
若未 armed
    请求安全 spool
    relax/init 控制器
    return

检查输入 timeout 和传感器健康
    ↓
限制/整形目标
    ↓
设置位置或姿态控制目标
    ↓
运行需要的控制器
    ↓
写 Motors 六轴输入
~~~

run 不应阻塞等待 UART，不应使用长 delay，不应动态分配大块内存。

## 8. 当前退出机制

当前 Mode 没有虚拟 exit。若新模式需要离开清理，应在 Sub::exit_mode 的旧/新模式判断中加入最小逻辑，或先设计清晰的通用退出扩展并评审。

不要以为 mode_xxx.cpp 中写一个同名 exit 就会自动调用。

## 9. 输入超时

外部控制模式要记录最后目标时间。超时动作应明确：

- 目标速度归零；
- 保持当前位置；
- 切 AltHold/PosHold；
- Surface；
- Disarm。

动作要结合是否仍有健康位置/深度。不能在位置已经失效时选择 PosHold。

## 10. 不直接操作 PWM

正确：

~~~text
Mode
→ Pos/Attitude Controller
→ Motors axis input
→ mixer
→ SRV/HAL
~~~

错误：

~~~text
Mode::run
→ hal.rcout->write
~~~

后者绕过 armed、spool、混控、端点和 failsafe。

## 11. 参数

新增参数只应表达需要现场调节的策略：

- 最大速度；
- timeout；
- 距离；
- 允许误差。

不应把每个内部临时变量做成参数。默认值要使首次启用保持保守。

## 12. QGC/MAVLink 切换

验证：

- SET_MODE 或 MAV_CMD_DO_SET_MODE；
- custom_mode 编号；
- HEARTBEAT 回报；
- mode name；
- 拒绝时有清晰消息；
- 旧 QGC 不识别名称时仍能显示编号/不影响协议。

## 13. SITL 测试案例

1. 状态健康时进入成功；
2. 缺位置/深度时拒绝；
3. 未解锁运行无输出；
4. 解锁后跟踪目标；
5. setpoint 超时；
6. 传感器中断；
7. 切入时输出连续；
8. 切出后状态清理；
9. failsafe 动作；
10. reboot 后参数保持。

## 14. 教学模式推荐

第一种练习模式不应发明新控制算法。可从 AltHold 或 Guided 复制最小结构，改变一个安全且可观察的目标限制，例如在 SITL 中限制水平速度。

完成后必须能解释每一处差异，不保留大段无关复制。

## 15. Code review 清单

- 编号不冲突；
- 依赖声明真实；
- init 可失败；
- run 无阻塞；
- disarmed 安全；
- timeout；
- 单位和 frame；
- 控制器初始化；
- 不直写硬件；
- 日志；
- tests；
- 文档；
- Pixhawk1 容量。
