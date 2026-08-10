# 实验 03：新增 Precision Manual 飞行模式

## 需求边界

新增一个低权重人工直控模式：输入仍来自现有 RC/MAVLink Manual Control，输出仍走现有 `AP_Motors6DOF`，只把角向请求限制为 35%，平移/升沉请求限制为 40%。

本实验明确不增加传感器、位置控制器、姿态闭环、新 mixer 或 PWM 特例。这样可以专注理解一个模式怎样真正进入 ArduSub，而不是把多个架构边界揉在一起。

## 数据流

```text
现有驾驶输入 [-1, 1]
        ↓
ModePrecisionManual::run()
        ↓ 比例/限幅，不改变坐标含义
AP_Motors6DOF 请求
        ↓
spool → SRV → IOMCU/PWM
```

这是直接推力模式，不是姿态稳定模式。仅复制一个 `mode_precision_manual.cpp` 会产生错误：rate controller 和 crash check 仍会把“所有非 Manual 模式”当作姿态闭环。因此必须同时扩展模式契约。

## 1. 复制模式实现

```bash
cp \
  Sub_learning/labs/03_precision_manual_mode/files/ArduSub/mode_precision_manual.cpp \
  ArduSub/
```

`ArduSub/` 的 `.cpp` 由车辆构建规则自动发现，不需要在 `wscript` 中写文件名。

## 2. 定义模式号与模式契约

在 `ArduSub/mode.h` 的 `Mode::Number` 中追加，不改变已有值：

```cpp
enum class Number : uint8_t {
    STABILIZE = 0,
    MANUAL = 19,
    PRECISION_MANUAL = 30,
};
```

`30` 是本教学分支的本地实验号。不能重排 `0` 和 `19`，也不能在不了解上游分配的情况下把实验号当成正式协议兼容承诺。

在 `Mode` 公共虚函数区新增：

```cpp
virtual bool uses_direct_thrust() const { return false; }
```

在 `ModeManual` 中覆盖：

```cpp
bool uses_direct_thrust() const override { return true; }
```

然后在 `mode.h` 末尾加入完整模式声明：

```cpp
class ModePrecisionManual : public Mode
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
    const char *name() const override { return "PrecisionManual"; }
    const char *name4() const override { return "PMAN"; }
    Mode::Number number() const override { return Mode::Number::PRECISION_MANUAL; }
};
```

`allows_arming=false` 是教学安全限制：不能直接在新模式内解锁；必须先在 Manual 中解锁、确认输入中立，再切入实验模式。

## 3. 让 `Sub` 拥有模式对象

在 `ArduSub/Sub.h` 的 friend 区加入：

```cpp
friend class ModePrecisionManual;
```

在现有两个模式对象旁加入：

```cpp
ModePrecisionManual mode_precision_manual;
```

模式不是每次切换时动态创建。`Sub` 持有常驻对象，`flightmode` 只指向当前对象。

## 4. 接入运行时分派

在 `ArduSub/mode.cpp` 的 `mode_from_mode_num()` switch 中加入：

```cpp
case Mode::Number::PRECISION_MANUAL:
    return &mode_precision_manual;
```

如果漏掉这一处，参数可以保存 30，但 `set_mode()` 会得到 `nullptr` 并报告不存在的模式。

## 5. 修正控制器契约

把 `ArduSub/Sub.cpp` 中：

```cpp
if (control_mode != Mode::Number::MANUAL) {
```

替换为：

```cpp
if (!flightmode->uses_direct_thrust()) {
```

这表示只有需要闭环角速度输出的模式才运行 rate controller。Manual 和 Precision Manual 都直接写 roll/pitch/yaw thrust 请求，不应被“非 Manual”这一旧的二值判断误分类。

同样，把 `ArduSub/failsafe.cpp` 的 crash-check 跳过条件：

```cpp
if (control_mode == Mode::Number::MANUAL) {
```

替换为：

```cpp
if (flightmode->uses_direct_thrust()) {
```

crash check 使用姿态控制误差；直接推力模式没有有效的姿态目标，不能拿该误差判定碰撞。这里不是关闭全局 failsafe，电池、漏水、GCS 和驾驶输入 failsafe 仍保持原逻辑。

## 6. 对外公布模式

在 `ArduSub/GCS_MAVLink_Sub.cpp` 的 `modes[]` 中加入：

```cpp
&sub.mode_precision_manual,
```

在 `ArduSub/Parameters.cpp` 的 `FLTMODE1` 元数据中把：

```text
@Values: 0:Stabilize,19:Manual
```

改为：

```text
@Values: 0:Stabilize,19:Manual,30:PrecisionManual
```

模式枚举决定内部身份，`mode_from_mode_num()` 决定能否切换，GCS 数组决定 AVAILABLE_MODES 公布，参数元数据决定地面站怎样解释 `FLTMODEn` 数值；四者职责不同。

## 7. 构建与静态检查

```bash
grep -Rns "PRECISION_MANUAL" ArduSub
git diff --check
./waf configure --board Pixhawk1 \
    --out build_student_precision_manual \
    --no-submodule-update
./waf sub -j4
```

检查所有命中是否都能解释。不要为了让编译通过而随意添加类型转换或扩大 friend 范围。

## 8. 分阶段验证

### 阶段 A：只验证模式注册

不连接推进器电源，设置一个飞行模式槽位：

```text
FLTMODE6 = 30
```

切换到第六档，确认 heartbeat/custom mode 变为 30，并能切回 Manual/Stabilize。

### 阶段 B：验证禁止在实验模式解锁

保持在 Precision Manual，尝试解锁应被拒绝。该结果证明 `allows_arming=false` 生效。

### 阶段 C：拆桨台架

1. 物理拆桨或断开推进器，只保留可观测的 PWM/示波器链路。
2. 在 Manual 解锁，所有输入保持中立。
3. 切入 Precision Manual。
4. 每次只改变一个轴，比较 Manual 与 Precision Manual 的归一化输出范围。
5. 断开 GCS/驾驶输入，确认原有 failsafe 仍按配置执行。

不要在水中第一次验证新模式。

## 9. 预期输出关系

| 输入轴 | Manual 满量程 | Precision Manual 满量程 |
|---|---:|---:|
| roll/pitch/yaw | 约 ±1.0 | 约 ±0.35 |
| forward/lateral | ±1.0 | ±0.40 |
| heave 对 throttle 的偏移 | 0–1，0.5 中立 | 约 0.30–0.70，0.5 中立 |

这里说的是进入 mixer 前的归一化请求，不是某个固定 PWM。最终通道值还取决于 frame mixer、servo function、限幅、spool 和 safety。

## 10. 复盘问题

1. 为什么 `run_rate_controller()` 不能继续使用 `control_mode != MANUAL`？
2. 为什么 crash check 需要模式能力，而不是模式号列表？
3. `flightmode`、`mode_precision_manual` 和 `Mode::Number` 分别是什么？
4. 为什么 0.5 是 heave 中立点，而 forward/lateral 的中立点是 0？
5. 哪些证据仍然不足以让这个模式进入 `master`？
