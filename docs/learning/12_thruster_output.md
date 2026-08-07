# 第 12 章：六自由度混控到 PWM/DShot

这一章详细展开：

~~~text
各轴归一化控制量
    ↓
AP_Motors6DOF
    ↓
SRV_Channels
    ↓
AP_HAL::RCOutput
    ↓
PWM / DShot / IO MCU
~~~

## 1. 文件地图

1. **ArduSub/motors.cpp**：车辆输出入口；
2. **libraries/AP_Motors/AP_Motors6DOF.h/.cpp**：布局矩阵和混控；
3. **libraries/AP_Motors/AP_MotorsMulticopter.cpp**：spool 和输出框架；
4. **libraries/AP_Motors/AP_Motors_Class.cpp**：Motor 到 SRV function；
5. **libraries/SRV_Channel/**：功能映射、端点和通道；
6. **libraries/AP_HAL_ChibiOS/RCOutput.cpp**：硬件波形；
7. **fmuv3.inc**：Pixhawk1 输出引脚和 IO MCU。

## 2. 六个输入轴

AP_Motors6DOF 接收：

- roll；
- pitch；
- yaw；
- throttle，代表垂向；
- forward；
- lateral。

它们通常归一化到约 -1～+1，但 throttle 在不同上游接口还可能以 0～1 表达，再由 Motors 转成双向语义。

正负方向必须结合坐标约定、推进器安装和 motor direction 参数验证。

## 3. 每个推进器是一行系数

add_motor_raw_6dof 为每个电机登记：

~~~text
roll factor
pitch factor
yaw factor
throttle factor
forward factor
lateral factor
testing order
~~~

通用混合公式：

~~~text
rpy_i =
    roll × roll_factor_i
  + pitch × pitch_factor_i
  + yaw × yaw_factor_i

linear_i =
    throttle × throttle_factor_i
  + forward × forward_factor_i
  + lateral × lateral_factor_i

motor_i =
    reverse_i × constrain(rpy_i + linear_i, -1, +1)
~~~

部分 Vectored 布局有专门的缩放和解耦实现，不能假定所有 frame 永远只执行这一段通用公式。

## 4. Vectored 布局手算

取 SUB_FRAME_VECTORED 的水平四推进器，只给：

~~~text
forward = +0.4
yaw     = +0.2
lateral = 0
~~~

按当前系数：

| Motor | yaw factor | forward factor | 结果 |
|---:|---:|---:|---:|
| 1 | +1 | -1 | +0.2 - 0.4 = -0.2 |
| 2 | -1 | -1 | -0.2 - 0.4 = -0.6 |
| 3 | -1 | +1 | -0.2 + 0.4 = +0.2 |
| 4 | +1 | +1 | +0.2 + 0.4 = +0.6 |

Motor 5、6 只参与 roll/throttle，本例为零。

负数不表示“坏电机”，而表示双向推进器反向推力。实际水动力方向还受布线、ESC 和 MOT_x_DIRECTION 影响。

## 5. 为什么要 constrain

多个轴相加可能超过范围：

~~~text
forward 0.8 + yaw 0.6 = 1.4
~~~

输出能力只有 1.0，就必须缩放或裁剪，并设置 limit 信息。否则转换到 PWM 会超出配置端点。

裁剪意味着所请求的六自由度合力无法完全实现。上层控制器需要通过 limit/anti-windup 避免继续积累。

## 6. motor reverse

MOT_1_DIRECTION 等参数为 +1 或 -1，在混控结果上乘方向。

它适合纠正单个推进器的正反定义，不应该用来掩盖整个坐标系或布局矩阵错误。

方向校验必须卸桨或采取水下推进器的安全台架措施，逐个低输出验证。

## 7. spool 状态机

armed 不是电机立即获得任意推力。AP_MotorsMulticopter 还管理：

- SHUT_DOWN；
- GROUND_IDLE；
- SPOOLING_UP；
- THROTTLE_UNLIMITED；
- SPOOLING_DOWN。

模式在未解锁时通常请求 GROUND_IDLE 并 relax 控制器；正常控制时请求 THROTTLE_UNLIMITED。

AP_Motors::output() 先运行 output_logic，再混控，再转换并输出。

## 8. 双向 PWM 映射

AP_Motors6DOF::calc_thrust_to_pwm 以 1500 为中点：

~~~text
thrust = 0    → 1500
thrust > 0    → 1500 到 output max
thrust < 0    → output min 到 1500
~~~

正负两侧范围可不同，因此分别使用：

- range_up = max - 1500；
- range_down = 1500 - min。

实际端点还要匹配 ESC 校准和 SRV/MOT 参数。

## 9. 从 Motor 编号到 SRV Function

AP_Motors::rc_write 不直接使用物理通道，而是：

~~~text
motor index 0
    ↓
SRV_Channel::k_motor1
    ↓
查找配置了 Motor1 function 的 SERVOx
    ↓
该物理 output channel
~~~

因此 Motor1 是“功能”，不必永远等于物理输出 1。SERVOx_FUNCTION 决定功能放在哪个口。

setup_motors 通过 add_motor_num 建立合理默认映射，但用户参数仍可能改变它。

## 10. SRV_Channels 做什么

SRV 层负责：

- function 到物理 channel；
- MIN/MAX/TRIM；
- reverse；
- scaled value 到 PWM；
- slew limit；
- emergency stop；
- override；
- failsafe PWM；
- 输出频率和协议设置；
- 最终调用 hal.rcout。

AP_Motors 关心 Motor1 推力；SRV 关心 Motor1 被分配到哪一路以及端点如何换算。

## 11. cork 与 push

Sub::motors_output 使用：

~~~text
srv.cork()
    ↓
多路输出写入临时缓冲
    ↓
motors.output()
    ↓
srv.push()
~~~

这样多通道尽量在同一批次提交，避免更新八个推进器时前几路已经改变、后几路仍是旧值。

在标准 PWM 路径中，rc_write 通过 SRV function 找到通道，set_output_pwm 会把值写向已 cork 的 RCOutput 缓冲，push 再统一提交。

## 12. RCOutput 才接触硬件

AP_HAL_ChibiOS::RCOutput 负责：

- 定时器 channel group；
- 输出频率；
- PWM/OneShot/DShot 模式；
- DMA；
- 同组通道约束；
- Safety 状态；
- IO MCU 转发；
- cork/push；
- 真正触发波形。

车辆层不应知道 TIM1_CH4、DMA stream 或 IO MCU 串口细节。

## 13. PWM 与 DShot

### PWM

用脉宽表达指令，双向 ESC 常以约 1500 微秒为中点。

### DShot

用数字帧表达 throttle code，可配合校验和与某些遥测能力。它对 DMA、时序和 channel group 有硬件要求。

选择 DShot 不是把 PWM 数值直接原样发出去；RCOutput 会按 output mode 编码。

Pixhawk1 的 IO MCU DShot 支持还依赖嵌入的 IO firmware 和硬件能力。

## 14. Safety、armed、interlock、spool

四者不是同一个布尔量：

| 层次 | 作用 |
|---|---|
| Safety Switch | 硬件输出安全门 |
| armed | 飞控逻辑是否解锁 |
| interlock | 是否允许 Motors 输出链工作 |
| spool state | 输出从关闭到全范围的状态过程 |

任一层禁止时，都可能使最终硬件保持安全值。调试“为什么电机不转”要从上到下逐层检查，不能直接绕过。

## 15. MotorDetect 与 Motor Test

MotorDetect 是飞行模式，用于自动识别推进器方向。

Motor Test 由 MAV_CMD_DO_MOTOR_TEST 等命令触发，包含：

- Safety 检查；
- armed 检查；
- 请求间隔超时；
- 失败冷却；
- 测试顺序和 throttle 类型检查。

两者都具有真实输出风险，不是无害的软件单元测试。

## 16. 故障排查顺序

某个推进器方向不对：

1. 检查 frame class；
2. 检查该 motor 六轴 factors；
3. 检查 MOT_x_DIRECTION；
4. 检查 SERVOx_FUNCTION；
5. 检查 MIN/MAX/TRIM；
6. 检查 PWM/DShot output mode；
7. 检查 Safety/armed/interlock/spool；
8. 最后检查线序和机械安装。

先查逻辑层级比盲目交换电机线更容易保留可解释性。

## 17. 验收问题

1. 六个混控输入是什么？
2. factor 正负表达什么？
3. 前进与偏航为什么会让四个推进器输出不同？
4. 输出超过 1 后发生什么？
5. Motor1 与物理 Output1 是否必然相同？
6. AP_Motors 与 SRV_Channels 各负责什么？
7. cork/push 解决什么问题？
8. 1500 在双向 PWM 中代表什么？
9. 四层安全状态有什么区别？
10. PWM 与 DShot 的差异在哪一层处理？
