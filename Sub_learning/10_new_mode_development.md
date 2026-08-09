# 实践一：开发一个新的 ArduSub 飞行模式

## 0. 本章要完成什么

本章用“DVL 辅助定点”做学习案例，演示怎样从需求走到可审查的新模式设计。当前 `master` 并没有实现这个模式，本章中的类名和代码是设计骨架，不能复制后直接用于实艇。

学习成果应是一份完整的开发包，而不只是一个 `mode_dvlhold.cpp`：

- 一页模式需求和数据契约。
- 一张输入到推进器的调用链。
- 一个最小文件改动清单。
- 一组进入、运行、失效和恢复测试。
- 一套拆桨台架、水池测试和回退步骤。

## 1. 先证明“真的需要新模式”

示例需求是：

> 外部融合器根据 DVL 等来源发送可靠的 Local FRD/Body FRD odometry；飞控在水平位置和深度有效时保持位置。ExternalNav 数据失效后，退出依赖水平位置的行为并让驾驶员可预测地接管。

第一步不是新建类，而是用现有链路做零代码验证：

```text
DVL / 其他传感器
    -> 外部融合器
    -> MAVLink ODOMETRY
    -> GCS_MAVLink::handle_odometry()
    -> AP_VisualOdom
    -> EKF3 ExternalNav source
    -> ModePoshold
    -> AC_PosControl / AC_AttitudeControl
    -> AP_Motors6DOF
```

如果普通 PosHold 已能满足产品行为，就不增加模式。只有当“进入前必须检查指定数据契约、运行期必须执行专用降级、GCS/日志必须区分依赖来源”等差异确实存在时，才增加一个薄模式。

[DVL 案例附录](09_dvl_feature_case_study.md)详细说明了为什么 DVL 速度不等于位置、ExternalNav 契约应包含什么，以及健康检查为什么要分设备、传输和 EKF 三层。

## 2. 先读懂现有模式的完整生命周期

新模式不是只实现 `run()`。当前 4.7.0 的模式切换链是：

```text
RC / joystick / GCS 请求模式号
    -> Sub::set_mode()
    -> mode_from_mode_num()
    -> requires_GPS() 对应的位置有效性检查
    -> requires_altitude() 对应的高度检查
    -> new_flightmode->init(false)
    -> exit_mode()
    -> 更新 flightmode 指针和 control_mode
    -> 写模式日志、heartbeat 和 notify

后续 fast loop
    -> Sub::update_flight_mode()
    -> flightmode->run()
```

`Mode::requires_GPS()` 是沿用的接口名；[`Sub::set_mode()`](../ArduSub/mode.cpp) 对它执行的是 `position_ok()` 检查，不代表必须安装物理 GPS。使用 ExternalNav 的水下模式仍要审计这个接口，但不能只按函数名猜测行为。

模式对象由 [`Sub.h`](../ArduSub/Sub.h) 静态持有，不应在切换时动态分配。模式号由 [`Mode::Number`](../ArduSub/mode.h) 定义，[`mode_from_mode_num()`](../ArduSub/mode.cpp) 完成运行时映射。

## 3. 选择最接近的父模式

先回答“新模式相对哪个已有模式只多了一条规则”：

| 新需求 | 优先复用 | 需要新增的差异 |
|---|---|---|
| 姿态稳定但深度手动 | `ModeStabilize` | 特定输入或限制策略 |
| 保深并允许水平手动 | `ModeAlthold` | 垂直目标以外的专用契约 |
| 保深、保持水平位置 | `ModePoshold` | 位置来源检查、专用降级或界面语义 |
| GCS 连续发送位置/速度目标 | `ModeGuided` | 新目标类型、超时或权限策略 |

DVL Hold 的控制能力与 PosHold 相同，因此学习设计应复用它：

```cpp
class ModeDVLHold : public ModePoshold
{
public:
    bool init(bool ignore_checks) override;
    void run() override;

protected:
    const char *name() const override { return "DVL Hold"; }
    const char *name4() const override { return "DVLH"; }
    Mode::Number number() const override;

private:
    bool source_contract_ok() const;
};
```

健康时 `run()` 只委托 `ModePoshold::run()`。不要复制 `control_horizontal()`，不要新写一套位置 PID，也不要直接设置 PWM。

## 4. 在编码前写清模式契约

### 4.1 进入条件

至少明确：

- 高度/深度估计是否有效。
- 水平位置和速度估计是否有效。
- ExternalNav backend 是否已启用并收到新鲜数据。
- 质量是否高于配置门槛。
- source set 是否真的选择 ExternalNav。
- 数据刚 reset 后是否允许立刻进入。
- `ignore_checks` 在什么内部场景允许跳过哪些检查；不能借它绕过真实安全前提。

### 4.2 运行期条件

定义每种故障的检测者、时间和动作：

| 故障 | 直接证据 | 示例动作 |
|---|---|---|
| 消息停止 | backend 更新时间超过门槛 | 退出水平定点 |
| 消息仍到达但质量差 | quality/bottom lock | 退出水平定点 |
| EKF 拒绝观测 | EKF/navigation health | 走 EKF 失效策略 |
| 深度同时失效 | altitude/depth health | 不再假定 AltHold 可用 |
| 坐标 reset | reset counter/EKF reset | 丢弃旧目标，禁止自动恢复 |

模式健康不能由一个模糊的 `healthy()` 包办。设备健康、传输健康和 EKF 接受状态属于不同层。

### 4.3 降级和恢复

示范策略可以是：

```text
水平来源失效
    -> 请求 AltHold，停止依赖水平位置
    -> AltHold 进入失败时请求 Manual
    -> 数据恢复后不自动回到旧 DVL Hold 目标
```

这只是需要评审的产品策略，不是通用答案。还要检查 leak、battery、pilot input、GCS 和 EKF failsafe 的优先级，避免新模式与既有 failsafe 互相抢占。

## 5. 最小实现顺序

正式开发必须在独立 feature 分支进行，并将以下步骤拆成小、可审查的改动。

### 步骤一：定义 feature guard

如果新模式依赖 PosHold 和 VisualOdom，guard 应表达真实依赖，例如概念上的：

```cpp
#define DVL_HOLD_ENABLED (POSHOLD_ENABLED && HAL_VISUALODOM_ENABLED)
```

正式写法必须以当前预处理环境和 build option 体系为准，并验证 guard 打开与关闭的构建。核心 PosHold 不能反向依赖这个可选模式。

### 步骤二：声明模式号和类

在 [`mode.h`](../ArduSub/mode.h) 中：

- 审计 `Mode::Number`、MAVLink custom mode、参数元数据、日志和地面站兼容性。
- 只增加一个未冲突的新值，不重排或复用已有模式号。
- 声明薄派生类，并准确实现 `requires_GPS()`、`requires_altitude()`、`allows_arming()` 和 `is_autopilot()`。

实验讨论过的某个空闲数字不等于永久保留号。正式开发时必须重新全局搜索。

### 步骤三：添加静态对象和运行时映射

在 [`Sub.h`](../ArduSub/Sub.h) 中增加受 guard 保护的模式对象；在 [`mode.cpp`](../ArduSub/mode.cpp) 的 `mode_from_mode_num()` 中增加同样 guard 下的映射。

这一步完成后，模式号才真正能指向对象。不能绕过 `set_mode()` 直接改 `flightmode` 或 `control_mode`。

### 步骤四：实现 `init()` 和 `run()`

建议将职责严格限制为：

```text
init()
    -> 检查新模式额外的数据契约
    -> 调用 ModePoshold::init()
    -> 任一步失败都拒绝进入

run()
    -> 检查运行期数据契约
    -> 健康：ModePoshold::run()
    -> 失效：请求经过评审的降级模式并发送一次性状态信息
```

注意避免：

- 每个循环重复发送 GCS 文本。
- 数据抖动时在两个模式间来回切换。
- 数据恢复后继续使用失效前的水平目标。
- 直接清除 spool block 或绕过 armed/interlock。
- 在模式中硬编码 EKF/VisualOdom 部署参数。

### 步骤五：同步用户可见接口

至少审计：

| 文件 | 核对内容 |
|---|---|
| [`Parameters.cpp`](../ArduSub/Parameters.cpp) | `FLTMODE1 @Values` 是否加入新模式；不改变任何 `GSCALAR/AP_GROUPINFO` 既有索引 |
| [`GCS_MAVLink_Sub.cpp`](../ArduSub/GCS_MAVLink_Sub.cpp) | base mode/能力报告是否应把它视为位置控制模式 |
| [`GCS_Sub.cpp`](../ArduSub/GCS_Sub.cpp) | XY/Z controller 的 enabled/health 报告是否准确 |
| joystick/RC 入口 | 是否真的需要专用按钮；不要无需求扩展 |
| 日志与 statustext | 能否辨认进入拒绝、运行失效和降级原因 |

新模式名字显示正常，不代表 GCS 的能力位和传感器健康报告已经正确。

## 6. 测试先覆盖生命周期，不先调 PID

新模式复用 PosHold 时，首要风险是模式契约和失效路径，而不是控制器参数。建议先写测试矩阵：

| 场景 | 初始条件 | 操作 | 期望结果 |
|---|---|---|---|
| 正常进入 | 深度、位置、ExternalNav 都有效 | 请求新模式 | `init()` 成功，模式号和名称正确 |
| 无数据进入 | 无 ExternalNav | 请求新模式 | 明确拒绝，不改变当前模式 |
| 低质量进入 | 消息存在但质量不足 | 请求新模式 | 明确拒绝 |
| 运行期超时 | 正常运行后停止消息 | 等待超时 | 一次告警，按策略降级 |
| 运行期低质量 | 保持频率但降低质量 | 继续运行 | 不能被“消息仍在到达”掩盖 |
| EKF 未接受 | backend 新鲜但导航无效 | 请求/保持模式 | 不把 backend health 冒充位置有效 |
| reset | 改变 reset counter 或注入位置跳变 | 继续运行 | 旧目标不被静默复用 |
| 降级也失败 | 水平和深度同时失效 | 触发失效 | 进入最终接管模式 |
| 恢复 | 数据重新健康 | 不发模式请求 | 不自动恢复旧目标 |

自动化用例应进入 [`Tools/autotest/ardusub.py`](../Tools/autotest/ardusub.py) 或对应库测试，而不是只在文档中描述。

## 7. 验证阶梯

1. 静态审查：模式号、guard、坐标、单位、超时、状态报告和所有回退分支。
2. 构建矩阵：SITL、Pixhawk4、feature enabled 和 disabled；记录真实 flash/RAM 增量。
3. SITL 正常链：无 GPS、ExternalNav、PosHold 复用、不同 yaw 下方向正确。
4. SITL 故障链：停止消息、低质量、reset、EKF 拒绝、深度同时失效。
5. 不接推进器的实物链：核对消息时间戳、frame、协方差、EKF source 和日志。
6. 拆桨台架：检查降级时各通道没有突跳，safety/arming/failsafe 仍有效。
7. 低功率系留水池：验证悬停、数据中断和人工接管；最后才考虑自由航行。

每一级都保留参数、命令、日志、结果和未覆盖风险。不得把“编译通过”写成“功能验证通过”。

## 8. 初学者最终应提交的设计说明

在真正写模式代码前，先写出以下十项：

1. 一句话用户行为。
2. 与最接近现有模式的唯一差异。
3. 输入来源、坐标系、单位、频率和延迟。
4. 进入条件。
5. 运行期健康条件。
6. 降级与恢复策略。
7. 复用的控制器和明确不新增的组件。
8. 修改文件及每个文件的职责。
9. 自动测试矩阵和硬件安全步骤。
10. 参数/模式号兼容性与回退方式。

能把这十项说清楚，新模式才进入编码阶段；否则应继续做源码追踪或零代码实验。
