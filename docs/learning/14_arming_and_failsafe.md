# 第 14 章：解锁、健康状态与失控保护

安全系统不是一句 if(error) disarm。它至少有四层：

~~~text
启动和参数有效
    ↓
Pre-arm 检查
    ↓
Arm 当下检查与状态切换
    ↓
运行时健康监控和 failsafe
    ↓
Safety / Motors / RCOutput 最终门控
~~~

## 1. 文件地图

1. **ArduSub/AP_Arming_Sub.h/.cpp**；
2. **libraries/AP_Arming/**；
3. **ArduSub/failsafe.cpp**；
4. **ArduSub/Sub.cpp** 中 50 Hz 和 3 Hz 任务；
5. **ArduSub/system.cpp** 中传感器健康和定时回调；
6. **libraries/AP_BattMonitor/**；
7. **libraries/AP_HAL_ChibiOS/RCOutput.cpp**。

## 2. Pre-arm 与 Arm checks

pre_arm_checks 可在未准备解锁时周期运行，为 QGC 提供准备状态。

arm_checks 在真正请求解锁时执行，某些检查依赖当前输入和动作来源。

mandatory_checks 是即使用户请求跳过常规检查也不能绕过的底线。

不要把所有返回 false 都当成同一错误；检查失败消息会指出子系统和原因。

## 3. ArduSub 专属 Pre-arm

AP_Arming_Sub::pre_arm_checks 先检查：

- 若已经 armed 则直接成功；
- HAL system 是否初始化完成；
- 是否配置了可用的 disarm/arm-toggle 按钮或 AUX 功能；
- 再进入 AP_Arming 公共检查。

要求明确的解除武装手段，是水下车辆安全策略的一部分。

## 4. 公共检查

AP_Arming 公共层覆盖的内容随编译开关、ARMING_CHECK 和硬件配置变化，典型包括：

- 参数和存储；
- INS/AHRS；
- barometer；
- compass；
- battery；
- RC；
- logging；
- GPS/位置；
- VisualOdom；
- board voltage；
- Safety；
- mission/fence 等。

ArduSub 还在 INS 检查中调用 AHRS pre_arm_check。

不能只在 AP_Arming_Sub.cpp 没看到某项就断定没有检查；大量逻辑在父类。

## 5. 解锁顺序

AP_Arming_Sub::arm 的关键过程：

1. 防止重入；
2. 必要时检查 throttle 位于 trim；
3. 调用 AP_Arming::arm；
4. 通知 Logger vehicle armed；
5. 暂停 mainloop failsafe；
6. 更新 Notify 和文本；
7. 记录初始 heading、处理 Home；
8. hal.util soft_armed = true；
9. enable_motor_output；
10. motors.armed(true)；
11. 写模式日志；
12. 重新启用 mainloop failsafe。

armed 状态与硬件 Safety 状态仍然不同。

## 6. 解除武装

disarm 会：

- 调用公共 disarm；
- 通知用户；
- 保存允许学习的罗盘偏置；
- motors.armed(false)；
- reset mission；
- 更新 Logger；
- soft_armed = false；
- 清输入 hold。

disarm method 会写入日志，便于区分人工、GCS、漏水或 CPU 等原因。

## 7. Failsafe 的三段结构

每条 failsafe 都应分开看：

~~~text
Detection
    何种数据、频率、阈值和持续时间
        ↓
Action
    警告、置中、切模式、上浮或 disarm
        ↓
Recovery
    状态何时清除，是否自动恢复旧模式
~~~

恢复健康一般不会自动切回原模式，因为突然恢复自主控制可能比保持当前安全模式更危险。

## 8. 检查频率

### 50 Hz 汇总任务

- pilot input；
- crash；
- EKF；
- depth sensor。

### 约 3 Hz 任务

- leak；
- internal pressure；
- internal temperature；
- GCS heartbeat；
- terrain。

### 1 kHz timer interrupt

- main loop lockup。

每项的触发持续时间还在自身代码中定义，不等于任务调度周期。

## 9. 主循环卡死保护

mainloop_failsafe_check 从核心定时中断以 1 kHz 调用，通过 scheduler tick 是否变化判断主循环是否还活着。

若约 2 秒无主循环：

1. 先输出最小值以保留记录机会；
2. 记录 CPU failsafe；
3. 之后周期性强制 motors disarmed/output。

它必须位于独立定时上下文，否则主循环卡死时同一循环里的检查也不会运行。

## 10. 深度计故障

若存在 depth sensor 但不健康：

- 设置 sensor_health failsafe；
- 发 Critical 和日志；
- 在 AltHold、Surface 或需要位置的模式下尝试切 Manual；
- 理论上切换失败则 disarm。

选择 Manual 是因为失去深度反馈后继续自动垂向闭环可能危险。

## 11. EKF failsafe

当前代码观察速度、位置、高度和罗盘相关 variance，按阈值和持续时间判断。坏状态连续约 2 秒才触发，避免单帧毛刺。

动作由 FS_EKF_ACTION 决定，当前可包含禁用或 disarm 等配置。它还更新 Notify 和周期警告。

对 ExternalNav 项目必须测试断流、跳变和质量下降如何反映到 EKF variance，而不是只测串口拔掉。

## 12. GCS 与驾驶输入不是同一 failsafe

GCS failsafe 根据匹配 MAV_GCS_SYSID 的 heartbeat 最后时间判断，可配置 warn、disarm、hold 或 surface。

pilot input failsafe 根据驾驶控制更新判断，可置中并按配置 disarm。

收到任意 MAVLink 数据不一定刷新两者；来源 system ID 和具体消息处理很重要。

## 13. 漏水与舱内状态

漏水：

- 周期警告；
- 记录状态；
- 可在已解锁时切 Surface。

舱内压力/温度：

- 阈值持续约 2 秒；
- 当前相应配置主要提供周期 warning。

“检测到”与“执行上浮/解除武装”要分别看参数和代码。

## 14. Battery failsafe

AP_BattMonitor 负责检测电压、容量等状态，并调用车辆 handler。Sub 根据 action 可：

- Surface；
- Disarm；
- Warn；
- None。

电池驱动、阈值检测和车辆动作分离，便于多个 battery backend 共用车辆策略。

## 15. 新传感器健康设计

新增 DVL/USBL 不能只提供 bool valid。至少定义：

- 最近接收时间；
- 帧 CRC；
- 设备自报状态；
- quality；
- 数值范围；
- 时间戳连续性；
- covariance；
- reset；
- 未启用、未发现、暂时退化和永久错误的区别。

然后决定：

- 是否阻止解锁；
- 哪些模式依赖；
- 运行中失效切什么模式；
- 恢复后是否只清 warning；
- 日志记录哪些证据。

## 16. 禁止的调试方式

- 删除 pre-arm 检查以让电机转；
- 把所有 failsafe action 改成 None；
- 直接 force safety off；
- 在桌面带桨/无约束做 MotorDetect；
- 把传感器 healthy 永久写 true。

安全验证要解决根因，不是让警告消失。

## 17. 验收问题

1. Pre-arm、arm checks 和 runtime failsafe 有何区别？
2. 强制解锁是否能绕过 mandatory checks？
3. ArduSub 为什么要求可用 disarm 功能？
4. soft_armed 与 hardware Safety 有何不同？
5. failsafe 为什么要拆 Detection/Action/Recovery？
6. 主循环卡死检查为何在定时中断？
7. 深度计故障为何切 Manual？
8. GCS heartbeat 和 pilot input 为什么分开？
9. 健康恢复后是否必然恢复旧模式？
10. 新传感器要设计哪些失败状态？
