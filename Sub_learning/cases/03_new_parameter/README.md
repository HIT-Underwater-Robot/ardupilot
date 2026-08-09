# 案例 03：新增航向保持过渡时间参数

> 设计练习：不要重排或复用已有参数索引；当前极简分支未实现本文参数。

> 状态：教学设计，当前源码仍使用固定 `250 ms`。本案例演示新增一个车辆参数时怎样保持 EEPROM 身份、默认行为和多模式一致性。

## 1. 需求定义

Stabilize、AltHold 和 PosHold 在驾驶员松开 yaw 杆后，不会立刻锁住航向：它们先用零 yaw-rate 请求让载具减速，并持续更新待保持航向；固定约 250 ms 后才进入绝对航向保持。

需求是把这段时间变为高级参数：

| 项目 | 设计 |
|---|---|
| 候选参数名 | `PILOT_YAW_DLY` |
| 类型 | `AP_Int16`，单位 ms |
| 默认值 | `250`，严格保持当前正常行为 |
| 候选范围 | `0–2000 ms` |
| 候选增量 | `10 ms` |
| 使用者 | Stabilize、AltHold、PosHold |
| 是否重启 | 不需要；每个 loop 读取并限制参数值 |

`0` 的明确语义是：yaw 输入回到 deadzone 后，下一次控制更新即可进入 heading hold。较大值会延迟航向锁定，可能增加漂移，不能宣传为“更稳定”。

## 2. 为什么不能只改 Stabilize

当前分支中相同的固定值出现在：

- [`ArduSub/mode_stabilize.cpp`](../../../ArduSub/mode_stabilize.cpp)
- [`ArduSub/mode_althold.cpp`](../../../ArduSub/mode_althold.cpp)
- [`ArduSub/mode_poshold.cpp`](../../../ArduSub/mode_poshold.cpp)

三者都使用 `last_pilot_yaw_input_ms` 和 `last_pilot_heading_rad` 完成“rate yaw → heading hold”过渡。如果参数名采用 vehicle-wide 的 `PILOT_*` 语义，却只让 Stabilize 消费它，那么同一驾驶员在切换 AltHold/PosHold 后会得到不同响应。这属于接口和实现不一致。

因此，本案例把三个模式视为同一个最小行为闭包。若产品只想改变某一个模式，应使用模式专属参数名，并在需求中明确这种差异。

## 3. 当前参数系统证据

| 源码 | 当前事实 |
|---|---|
| [`ArduSub/Parameters.h`](../../../ArduSub/Parameters.h) | enum 数字是 EEPROM 中的稳定身份；注释明确警告不得重排或重叠 |
| [`ArduSub/Parameters.cpp`](../../../ArduSub/Parameters.cpp) | `GSCALAR` 把成员、名称、默认值和参数 enum identity 关联起来 |
| [`ArduSub/Sub.h`](../../../ArduSub/Sub.h) | 已保存 `last_pilot_heading_rad`、`last_pilot_yaw_input_ms` |
| [`ArduSub/system.cpp`](../../../ArduSub/system.cpp) | 启动时初始化 pilot heading |
| 三个 `mode_*.cpp` | 当前比较 `tnow`、最后 yaw 输入时间和固定 `250` |

当前 `Parameters.h` 在 `k_param_failsafe_throttle_value` 之后、`k_param_vehicle = 257` 之前留有未使用编号。针对这一个固定基线，`245` 可作为候选新 identity；正式实现时仍必须重新检查整份 enum 和当前工作分支，不能仅凭本教程复制。

参数 identity 与参数显示名是两回事：

```text
k_param_pilot_yaw_delay = 245  # EEPROM identity
PILOT_YAW_DLY                  # GCS/API visible name
```

任何一个都不能与现有参数冲突。

## 4. 最小修改设计

### 4.1 `Parameters.h`

伪代码：

```cpp
enum {
    // ... existing identities remain unchanged ...
    k_param_failsafe_throttle_value,
    k_param_pilot_yaw_delay = 245,
    k_param_vehicle = 257,
    k_param__gcs = 258,
};

AP_Int16 pilot_yaw_delay_ms;
```

规则：

- 明确赋值，不让未来插入项意外改变编号。
- 不使用已经标为 deprecated/unused 的旧 identity 来“节省编号”；旧固件或参数文件仍可能依赖它。
- 不提高 `k_format_version`，因为这是兼容的追加参数，不是重新解释全部 EEPROM layout。
- 成员类型能覆盖元数据范围；若将来范围扩大，要重新审计类型。

### 4.2 `Parameters.cpp`

```cpp
// @Param: PILOT_YAW_DLY
// @DisplayName: Pilot yaw hold transition delay
// @Description: Time after pilot yaw input returns to the deadzone before Stabilize, AltHold and PosHold capture and hold heading
// @Units: ms
// @Range: 0 2000
// @Increment: 10
// @User: Advanced
GSCALAR(pilot_yaw_delay_ms, "PILOT_YAW_DLY", 250),
```

参数名需要满足 ArduPilot 的长度限制；描述必须列出三个受影响模式，不能让用户误以为它改变 Manual/Acro yaw。

### 4.3 模式消费

当前代码使用“当前时间小于最后时间加 250”的形式。参数化时建议改成 elapsed-time 比较，能自然处理 `uint32_t` 毫秒计数回绕：

```cpp
const uint32_t yaw_hold_delay_ms =
    constrain_int32(g.pilot_yaw_delay_ms.get(), 0, 2000);

if ((tnow - sub.last_pilot_yaw_input_ms) < yaw_hold_delay_ms) {
    // 继续给零 yaw-rate，并更新待保持 heading
} else {
    // 进入 absolute heading hold
}
```

相同语义应应用在 Stabilize、AltHold、PosHold。第一版可以保留三处清晰的小改动，避免为了一个参数顺带做模式基类重构；如果以后抽 helper，应在独立提交中做行为等价证明。

`@Range` 主要服务参数工具和用户界面，不能假定所有写入路径都会自动强制范围，所以消费点仍需 `constrain`。

## 5. 修改文件清单

| 文件 | 修改 |
|---|---|
| `ArduSub/Parameters.h` | 追加唯一 enum identity 和 `AP_Int16` 成员 |
| `ArduSub/Parameters.cpp` | 增加完整元数据和默认值 |
| `ArduSub/mode_stabilize.cpp` | 用受限参数替换固定 250 ms |
| `ArduSub/mode_althold.cpp` | 同上 |
| `ArduSub/mode_poshold.cpp` | 同上 |
| 参数/模式相关测试 | 默认值、边界、存储和三模式行为 |

不需要修改 `AP_Param` library，也不需要建立新的 `ParametersG2` 组。只有在车辆参数编号空间或职责确有证据不适合时才考虑其他位置。

## 6. 行为状态表

| yaw 输入 | 距最后非零 yaw 输入时间 | 参数 | 预期 |
|---|---:|---:|---|
| 非零 | 任意 | 任意 | 使用 yaw-rate；更新 heading 和 timestamp |
| 零 | 小于 delay | `>0` | yaw-rate 设零减速；持续更新 heading |
| 零 | 等于/大于 delay | `>0` | 锁住最后更新的绝对 heading |
| 零 | 任意有效 loop | `0` | 直接进入 heading hold 分支 |
| 非法负值 | 任意 | `<0` | 运行时限制为 `0` |
| 过大值 | 任意 | `>2000` | 运行时限制为 `2000` |

切换模式时，各模式的 `init()` 和 disarmed 分支仍应按当前逻辑重置待保持航向，不能让参数改动改变进入安全性。

## 7. 测试设计

### 7.1 参数 identity 和存储

- 生成/检查参数元数据，确认名称、单位、范围、增量和默认值。
- 从旧 4.7.0 参数文件启动，新参数应得到默认 `250`，其他参数值保持不变。
- 写入 `0`、`250`、`2000`，重启后值能正确保存。
- 写入范围外值时，运行行为按消费点限制；地面站提示与真实行为一致。
- 恢复默认后得到 `250`。

### 7.2 SITL 时序

对 Stabilize、AltHold、PosHold 分别执行同一组步骤：

1. 建立可用的模式进入条件。
2. 给定一段非零 yaw 输入并记录最后输入时间。
3. yaw 归中。
4. 从日志观察 target yaw rate、target heading 和 actual yaw。
5. 检查 heading capture 相对于 timestamp 的时间。

| 参数 | 通过标准 |
|---:|---|
| 0 ms | 不保留额外减速窗口 |
| 250 ms | 与修改前基线行为一致 |
| 1000 ms | 约 1 秒内持续更新 capture heading，之后保持 |
| 2000 ms | 无溢出、NaN 或模式失效 |

还应覆盖毫秒计数回绕附近的单元测试或可控时间测试，证明 elapsed subtraction 正确。

### 7.3 回归

- Manual 和 Acro 不受此参数影响。
- roll、pitch、throttle、forward、lateral 请求不受影响。
- 三个模式的进入拒绝、disarmed、failsafe 和切换行为不变。
- SITL 和实际 Pixhawk 类目标板构建。
- 控制相关实机验证必须拆桨或隔离推进器，先小输入对比 250 ms 默认值。

## 8. 验收标准

- 参数 enum identity 唯一，所有已有 identity 原值不变。
- 默认值 `250` 对三个模式均保持基线时序。
- `0–2000` 边界语义清晰并有测试。
- 三个模式的行为一致，Manual/Acro 不受影响。
- 参数可保存、重启、恢复默认，地面站元数据正确。
- 没有借此重构 yaw controller、改变 PID 或绕过 failsafe。

## 9. 风险与回退

风险包括：只修改一个模式、误用旧参数编号、以为 `@Range` 自动保证运行值、时间加法在计数回绕时异常，以及大 delay 造成操纵手感和航向漂移。

运行时回退只需把 `PILOT_YAW_DLY` 设回 `250`。完整回退则刷回增加参数前的固件；由于是兼容追加，旧固件会忽略未知的新 identity，其他参数不应被迁移或清空。实施前仍要备份参数并保存基线日志。
