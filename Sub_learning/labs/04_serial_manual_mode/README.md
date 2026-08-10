# 实验 04：把串口命令接入一个独立模式

> 高级拆桨实验。必须先完成实验 02 和 03，并能解释串口状态机、direct-thrust 模式契约、spool 和现有 failsafe。不得把本实验固件直接用于下水航行。

## 为什么必须新增模式

通信库的职责是“解析并判断数据新鲜度”，不应偷偷调用 motors。是否允许串口数据影响载具，是模式策略和安全准入问题。因此本实验新增 `SerialManual`，而不是在 `AP_LearningSerial::update()` 中直接输出推进器。

```text
PC 二进制帧
   ↓ CRC/范围/时间戳
AP_LearningSerial
   ↓ 最近有效 Command
ModeSerialManual
   ↓ enable、超时锁存、25% 上限、armed/spool
AP_Motors6DOF
```

## 安全状态机

```text
进入模式前：链路健康 + disabled 帧
        ↓
模式进入，输出保持 GROUND_IDLE
        ↓ enabled 新鲜帧
最多 25% 归一化请求
        ↓ 超过 250 ms 没有有效帧
link_lost 锁存 → GROUND_IDLE
        ↓
即使通信恢复也不能自动恢复输出
必须切出模式、检查原因、重新发送 disabled 帧后再进入
```

`allows_arming=false`：不能在 SerialManual 内解锁。实验者必须在 Manual 内解锁并保持输入中立，再切换到该模式。

自定义串口不更新 `failsafe.last_pilot_input_ms`，也不伪造 GCS heartbeat。正常 RC/MAVLink 驾驶输入和 GCS 链路必须保持连接；本实验不能绕过现有 failsafe。

## 1. 前置代码

先按实验 02 接入：

- `libraries/AP_LearningSerial/`
- `ArduSub/wscript` 中的库白名单
- `Sub.h` 中的 `learning_serial` 成员
- `system.cpp` 中的 `learning_serial.init()`
- `Sub.cpp` 中 100 Hz `learning_serial.update()` 任务

再按实验 03完成：

- `Mode::uses_direct_thrust()` 契约
- Manual 的 `uses_direct_thrust=true`
- `run_rate_controller()` 按能力判断
- crash check 按能力判断

实验 04 可以与 Precision Manual 共存；建议保留它作为随时可切换的低权重 RC 回退模式。

## 2. 复制模式文件

```bash
cp \
  Sub_learning/labs/04_serial_manual_mode/files/ArduSub/mode_serial_manual.cpp \
  ArduSub/
```

## 3. 注册模式类

在 `ArduSub/mode.h` 的枚举中追加：

```cpp
SERIAL_MANUAL = 31,
```

在文件末尾增加：

```cpp
class ModeSerialManual : public Mode
{
public:
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return false; }
    bool uses_direct_thrust() const override { return true; }

protected:
    const char *name() const override { return "SerialManual"; }
    const char *name4() const override { return "SMAN"; }
    Mode::Number number() const override { return Mode::Number::SERIAL_MANUAL; }

private:
    bool _link_lost = true;
};
```

`_link_lost` 属于模式对象，不是函数局部变量；这样断线锁存能跨越每次 `run()` 调用保存。

## 4. 让 `Sub` 持有并授权对象

在 `ArduSub/Sub.h` 加入：

```cpp
friend class ModeSerialManual;
```

以及：

```cpp
ModeSerialManual mode_serial_manual;
```

friend 只授予这个模式访问 `Sub` 私有的 `learning_serial` 和控制辅助函数，不应把整个库改为 public。

## 5. 分派、GCS 与参数元数据

在 `ArduSub/mode.cpp` 的 switch 加入：

```cpp
case Mode::Number::SERIAL_MANUAL:
    return &mode_serial_manual;
```

在 `ArduSub/GCS_MAVLink_Sub.cpp` 的 `modes[]` 加入：

```cpp
&sub.mode_serial_manual,
```

在 `ArduSub/Parameters.cpp` 更新：

```text
@Values: 0:Stabilize,19:Manual,30:PrecisionManual,31:SerialManual
```

最后给一个模式槽位赋值，例如：

```text
FLTMODE5 = 19
FLTMODE6 = 31
```

确保相邻档位保留 Manual 作为快速回退。

## 6. 构建

```bash
grep -Rns "SERIAL_MANUAL\|uses_direct_thrust\|learning_serial" ArduSub
git diff --check
./waf configure --board Pixhawk1 \
    --out build_student_serial_manual \
    --no-submodule-update
./waf sub -j4
```

逐条解释 grep 命中：库对象、初始化、调度、模式声明、分派、对外公布、控制器契约和 failsafe 契约缺一不可。

## 7. 第一次验证：不解锁

保持推进器断电，运行 PC 工具，在前 5 秒发送 disabled 帧，之后自动置 enable：

```powershell
py Sub_learning\labs\02_framed_serial_library\tools\send_serial_command.py `
  COM6 --count 400 --rate 20 --enable-after 5 `
  --axes 0 0 0 0 100 0
```

在前 5 秒内切换到 `FLTMODE6=31`。模式应能进入；超过 5 秒后，帧虽然 enable，但载具未解锁，所以 spool 仍不能输出。

停止工具超过 250 ms 后再次发送，模式应保持锁存，不能因重连自动恢复。切到 Manual，再发送 disabled 帧并重新进入，才能清除锁存。

## 8. 第二次验证：拆桨台架

只有完成以下检查后才进入本阶段：

- 推进器已拆桨或物理断开，只用示波器/逻辑分析仪观察输出。
- GCS 与正常驾驶输入链路持续在线。
- Manual 档位可立即回退，驾驶输入全部中立。
- 独立断电开关可触达。

流程：

1. PC 发送 5 秒 disabled 帧。
2. 在 Manual 解锁。
3. 保持输入中立，切到 SerialManual。
4. PC 自动开始 enabled 帧，每次只给一个轴 100，即最大输入的 10%。
5. 观察 mixer 前请求理论上为 0.025；实际 PWM 还受 mixer、spool、safety 和输出范围影响。
6. 拔掉 USB-TTL 或停止脚本，250 ms 后应进入 GROUND_IDLE 并锁存。
7. 切回 Manual 后立即 disarm。

不要一次发送多轴大输入，不要接桨验证，不要在断线后依赖自动重连。

## 9. 代码审计重点

| 代码 | 必须能解释的理由 |
|---|---|
| 进入模式要求 disabled 新鲜帧 | 防止带着旧的运动命令切入 |
| `allows_arming=false` | 解锁必须发生在已知基线模式 |
| 250 ms timeout | 20 Hz 链路允许少量抖动，但不能长期沿用旧命令 |
| timeout 后锁存 | 重连不会自动重新启动推进器 |
| 25% 固定上限 | 台架实验限制最大请求，不宣称产品参数 |
| UART frontend 不调用 motors | 通信、策略、执行三层职责分离 |
| 不刷新现有 failsafe 时间戳 | 自定义链路不能冒充 GCS/驾驶链路健康 |

## 10. 为什么仍不能进入 `master`

本实验没有解决：认证与来源、序号回放、正式参数接口、日志消息、完整 pre-arm check、与 GCS failsafe 的产品策略、协议版本迁移、硬件 EMI/断线统计、SITL/自动回归和水下验证。因此它只适合作为架构学习样例。

## 11. 复盘问题

1. 为什么超时处理在模式层，而 CRC 在串口库？
2. 为什么 disabled 帧不触发断线锁存，而 stale 帧会触发？
3. 为什么链路恢复后不自动清除 `_link_lost`？
4. 如果要产品化，哪些状态必须记录进 DataFlash log？
5. 如何把 25% 上限做成新参数，同时不改变已有 `AP_GROUPINFO` 索引？
