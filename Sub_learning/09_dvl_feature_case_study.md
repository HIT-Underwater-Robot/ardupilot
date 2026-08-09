# DVL 功能案例附录：从 ExternalNav 到专用模式

## 0. 案例状态

本章是[新模式开发主线](10_new_mode_development.md)的传感器与状态估计附录，来自一个独立工作树中的 DVL Hold 实验草案，只用于说明二次开发方法。当前 `master` 没有 DVL Hold 模式、没有 `AP_DVL` 串口驱动，也没有伴随计算机发送程序。不要把本章当作已经通过实机验证的产品功能。

案例的价值不在于复制代码，而在于展示怎样回答以下问题：

1. 新传感器提供的物理量是否足以支持目标功能？
2. 现有公共库能否承载数据，是否真的需要新增 `lib`？
3. 新功能应位于传感器、EKF、模式还是控制器层？
4. 数据失效后，飞控应怎样确定性降级？
5. 怎样用 SITL 和硬件测试证明整个链路，而不是只证明编译通过？

## 1. 先定义需求，不先定义类名

示例需求是：

> 当 DVL 提供可靠的海底参考运动估计时，潜艇保持水平位置和深度；DVL 数据超时或质量不足时，立即退出水平悬停，并保留驾驶员可接管的安全模式。

开发前仍要补齐：

- DVL 型号、安装姿态和输出协议。
- bottom track 与 water track 的选择规则。
- 输出是速度、位置，还是已经融合的 odometry。
- 坐标系、正方向、单位、时间戳、延迟和频率。
- 质量、波束、bottom lock、重置和协方差定义。
- 水平来源失效时保深、全手动或解除解锁的产品策略。

如果这些问题没有答案，就不能开始写“通用 DVL 驱动”。

## 2. DVL 速度不等于悬停位置

DVL 常见输出是相对海底或水体的三轴速度。位置悬停需要：

```text
position error = target position - estimated position
```

单独把速度控制到零，只能抑制当前运动，不能知道累计漂移后离原目标多远。water track 的零速度还是相对水体；存在水流时，它不等于相对海底位置不变。

因此，第一版必须明确由谁完成位置积分或多传感器融合：

- 伴随计算机输出连续的 Local NED/FRD odometry；或
- 飞控内部新增经过验证的 DVL frontend/backend，并把数据送入 EKF。

不能在模式中临时积分 DVL 速度，再把这个积分值冒充完整状态估计。

## 3. 第一选择：复用 ExternalNav 数据链

如果外部融合器已经能给出位置、速度、姿态/航向和质量，最小路径是：

```text
DVL + IMU/USBL/其他来源
    -> 外部融合器
    -> MAVLink ODOMETRY
    -> GCS_MAVLink::handle_odometry()
    -> AP_VisualOdom
    -> EKF3 ExternalNav source
    -> ModePoshold
    -> AC_PosControl / AC_AttitudeControl
    -> AP_Motors6DOF
```

当前 [GCS_Common.cpp](../libraries/GCS_MAVLink/GCS_Common.cpp) 的 `handle_odometry()` 接受 `MAV_FRAME_LOCAL_FRD` 与 `MAV_FRAME_BODY_FRD` 组合；[AP_VisualOdom](../libraries/AP_VisualOdom/) 已提供 MAVLink backend、健康超时、质量门槛、位置偏移、延迟和噪声参数。

外部融合程序属于独立伴随计算机工程，不放回本固件仓库。飞控仓库只维护数据契约、消费路径、模式安全策略和测试。

## 4. 先证明 PosHold 能工作，再考虑新模式

第一阶段甚至不需要增加模式。通过参数把 EKF3 的水平位置和速度来源配置为 ExternalNav，然后使用现有 [ModePoshold](../ArduSub/mode_poshold.cpp) 验证：

- 无 GPS 时 `position_ok()` 是否成立。
- body forward/lateral 是否正确转换到 NE 速度目标。
- 松杆后位置目标是否稳定。
- 深度仍由压力传感器路径控制。
- yaw 变化后水平输出旋转是否正确。
- ExternalNav 超时、质量下降或 reset 后 EKF 和模式怎样反应。

只有这条现有链路已经被日志和 SITL 证明，才有理由增加 DVL 专用用户体验。

## 5. 什么时候需要专门的 DVL Hold 模式

一个薄的专用模式只有在以下需求成立时才有价值：

- 进入模式前必须额外检查 ExternalNav 流本身，而不只检查 EKF 当前仍有位置。
- 运行期质量下降时要采用明确的产品降级策略。
- 地面站和日志需要区分普通 PosHold 与“依赖 DVL 契约的 PosHold”。
- 不允许数据恢复后自动回到旧水平目标。

示范设计复用 PosHold，而不是复制控制器：

```cpp
class ModeDVLHold : public ModePoshold
{
public:
    bool init(bool ignore_checks) override;
    void run() override;

private:
    bool odometry_healthy() const;
};
```

健康时只调用 `ModePoshold::run()`。新模式只增加来源契约、进入检查和丢失策略，不新增 PID、不直接计算推进器 PWM。

## 6. 一个新模式会触及哪些文件

| 位置 | 示例职责 | 不应该做什么 |
|---|---|---|
| `ArduSub/config.h` | 定义依赖 PosHold/VisualOdom 的 feature guard | 强行打开所有板卡功能 |
| `ArduSub/mode.h` | 分配未占用模式号并声明薄模式 | 重排已有模式号 |
| 新 `mode_*.cpp` | 健康检查、init、run 和降级 | 复制 PosHold 控制器 |
| `ArduSub/Sub.h` | 持有模式对象 | 动态分配长期对象 |
| `ArduSub/mode.cpp` | 模式号映射 | 绕过 `set_mode()` |
| `ArduSub/Parameters.cpp` | 更新 FLTMODE 元数据 | 改变既有参数索引 |
| `GCS_MAVLink_Sub.cpp` | 报告模式能力和可用名称 | 修改 MAVLink 协议定义 |
| `GCS_Sub.cpp` | 报告控制器状态 | 伪报传感器健康 |
| `Tools/autotest/ardusub.py` | 验证进入、健康丢失和降级 | 只检查模式号能切换 |

实验草案曾选择空闲的模式号 22。它只是当前源码中的候选，不是永久保留号；正式实现前必须重新审计枚举、参数、日志和地面站兼容性。

## 7. 健康契约必须分层

示范中的最小流健康检查包含：

```text
AP_VisualOdom object exists
    AND backend enabled
    AND last accepted update is fresh
    AND quality >= configured minimum
```

当前 `AP_VISUALODOM_TIMEOUT_MS` 是 300 ms。这只能证明 ExternalNav backend 最近收到满足接口的数据，不能证明：

- 物理来源一定是 DVL。
- EKF 当前 source set 确实选择了 ExternalNav。
- bottom lock 有效。
- 坐标、时间延迟和协方差正确。
- EKF 已经接受该观测而非持续拒绝创新。

所以健康契约至少分三层记录：

1. 设备/融合器健康：bottom lock、波束、时间戳、质量。
2. 传输/backend 健康：消息频率、frame、超时和解析结果。
3. EKF/导航健康：来源选择、创新、位置有效性和 reset。

专用模式不能用一个 `healthy()` 替代这三层事实。

## 8. ODOMETRY 数据契约

外部融合路径至少定义以下字段：

| 字段 | 坐标与单位 | 审查重点 |
|---|---|---|
| time | 单调时钟，µs | 采样时刻，不是发送时刻；处理回绕和重启 |
| position | Local FRD/NED，m | 原点、Down 正方向和 reset 行为 |
| quaternion | child 到 parent，`[w,x,y,z]` | 归一化、旋转方向和 yaw 来源 |
| velocity | Body FRD，m/s | 前、右、下符号及安装旋转 |
| angular rate | Body FRD，rad/s | 是否真实提供，不能填伪零 |
| pose covariance | 方差 | 标准差必须平方，未知量按协议表达 |
| velocity covariance | 方差 | 与实际噪声匹配，不为追求平滑而虚报 |
| reset counter | 0–255 | 每次坐标系重置递增 |
| quality | -1–100 | 明确 bottom/water track 和失效映射 |

消息仍在到达但质量变差，必须被视为独立故障场景。只检查频率会让持续发送的坏数据继续控制真实载具。

## 9. 降级策略

示范策略是：

```text
DVL/ExternalNav unhealthy
    -> request AltHold with EKF failsafe reason
    -> if AltHold init fails, request Manual
```

选择 AltHold 的意图是停止依赖水平位置，同时继续压力深度和姿态控制；Manual 是深度估计也不可用时的最终驾驶员接管路径。

正式产品还必须评审：

- 是否需要先把水平目标清零或限制水平输出。
- 消息抖动时是否需要滞回和 debounce。
- GCS 文本是否会重复发送。
- failsafe 优先级是否与 pilot input、leak、battery 和 EKF failsafe 冲突。
- 恢复后是否允许重新进入；默认不自动恢复可避免目标跳变。
- 操作者怎样知道当前依赖和接管责任。

不要在数据丢失时直接绕过 spool state，也不要把“退出模式”误写成“解除硬件 safety”。

## 10. feature guard 与参数

专用模式的 guard 可以表达真实依赖：

```cpp
DVL_HOLD_ENABLED = POSHOLD_ENABLED && HAL_VISUALODOM_ENABLED
```

但这只是示范。正式实现要验证 guard 在预处理阶段可见，并分别构建 enabled/disabled 配置。核心 PosHold 不能反向依赖 DVL 模式。

EKF 与 VisualOdom 参数属于部署配置，不应硬编码在模式里。至少审计：

- EKF3 水平位置/速度是否选 ExternalNav。
- 垂直位置是否继续使用压力传感器。
- VisualOdom backend 类型、质量门槛、延迟、位置偏移和噪声。
- yaw 是否真的有可靠外部来源。

DVL 通常不能单独提供长期绝对 yaw。没有磁罗盘、双天线或其他航向约束时，陀螺零偏会逐渐旋转水平坐标系。

## 11. 自动测试应该证明什么

一个有价值的 SITL 用例可使用 Vicon/ODOMETRY 模拟器：

1. 禁用 GPS，选择 ExternalNav 水平位置和速度。
2. 保留压力深度来源。
3. 以高于门槛的质量发送 ODOMETRY。
4. 等待可解锁并进入候选模式。
5. 确认正常运行时仍复用 PosHold 控制链。
6. 保持消息发送，但把质量降到门槛以下。
7. 等待明确的 statustext 和 AltHold 降级。
8. 解除解锁并恢复测试环境。

还应增加：

- 完全停止消息的 300 ms 超时场景。
- 进入模式前无数据、低质量和 EKF 未就绪场景。
- reset counter 改变和位置跳变。
- 不同 yaw 下速度方向检查。
- NaN/Inf、错误 frame、异常 covariance 和时间戳。
- AltHold 也不可进入时的 Manual 回退。

测试只写在实验文档里不算完成；正式功能必须把可重复用例放入 `Tools/autotest/ardusub.py` 或对应库测试。

## 12. 推荐开发阶段

### 阶段 A：零代码证明

- 用已知良好的 ODOMETRY 源配置 EKF3。
- 在普通 PosHold 验证坐标、方向、频率、延迟和悬停逻辑。
- 保存日志和参数，不接推进器或保持物理隔离。

### 阶段 B：薄模式实验

- 建立独立 feature 分支。
- 只增加来源检查、模式注册和降级。
- 不复制 PosHold，不新增公共 PID，不加入伴随计算机工具。
- 先写 SITL 失效测试，再评审代码。

### 阶段 C：固件验证

- 构建 SITL 与 Pixhawk4。
- 记录 flash/RAM 增量。
- 检查 feature guard 关闭的构建。
- 审查参数、AVAILABLE_MODES、日志和地面站显示。

### 阶段 D：硬件验证

- 不接推进器验证消息和 EKF。
- 手动移动/旋转机体核对所有轴。
- 停发和降质量分别验证降级。
- 拆桨台架检查输出方向。
- 低功率系留水池测试后才做自由悬停。

### 阶段 E：是否需要直连驱动

只有获得厂商协议、真实录包和明确的通用 API 后，才单独立项 `AP_DVL`。直连驱动与 DVL Hold 模式是两个不同变更，应分支、提交和测试分离。

## 13. 这个案例最重要的结论

- 新传感器功能先证明数据语义，再设计类。
- 能通过现有公共库表达的数据，不新增库。
- 能继承成熟模式的安全行为，不复制控制器。
- source health、transport health 和 EKF health 不能混为一个布尔值。
- failsafe 与恢复策略是功能主体，不是最后补的异常分支。
- 伴随计算机程序保持在独立工程，固件仓库只维护飞控职责。
- 编译通过只是起点；SITL、Pixhawk4、拆桨台架和水池证据缺一不可。
