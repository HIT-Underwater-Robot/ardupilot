# 案例 02：为 MCP9808 增加温度传感器 backend

> 状态：教学设计，当前仓库尚未包含 MCP9808 backend。本案例选择明确型号和官方协议，目的是演示怎样把真实器件接入公共 library，而不是在 ArduSub 模式中临时读取 I²C。

## 1. 需求定义与硬件依据

需求：让 ArduSub 通过现有 `TEMPn_*` 参数配置并读取 Microchip MCP9808 I²C 温度传感器，输出摄氏度、健康状态和 `TEMP` 日志；第一版不让温度值参与控制或 failsafe。

实现前必须保存并评审以下官方资料：

- [MCP9808 产品页](https://www.microchip.com/en-us/product/mcp9808)
- [MCP9808 官方数据手册 DS20005095B](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP9808-0.5C-Maximum-Accuracy-Digital-Temperature-Sensor-Data-Sheet-DS20005095B.pdf)
- [Microchip 在线寄存器说明](https://onlinedocs.microchip.com/oxy/GUID-E22726BA-F321-4F22-9A99-2F3CADEC9AAB-en-US-1/GUID-E3B6C1D7-1A2E-4A30-80EB-5C4FDA227C4F.html)

datasheet 中与第一版 driver 直接相关的事实包括：

| 项目 | 设计使用 |
|---|---|
| 总线 | I²C/SMBus |
| 默认地址 | A2/A1/A0 均为低时使用 `0x18`；其他接法必须按原理图和 datasheet 配置 |
| 环境温度寄存器 | 指针 `0x05`，16 bit，大端传输 |
| Manufacturer ID | 寄存器 `0x06`，期望 `0x0054` |
| Device ID/Revision | 寄存器 `0x07`，Device ID 字段应匹配 MCP9808 |
| Resolution | 寄存器 `0x08`；第一版可保留上电默认，或明确配置后验证回读 |
| 温度基本分辨率 | 温度数据的有效低位对应 `0.0625 °C`；符号位和告警标志必须先按手册解析 |

不要把相近型号的寄存器、ID 或换算公式凭经验套入。driver 注释应注明使用的数据手册版本。

## 2. 当前公共库结构

| 源码 | 当前职责 | 本案例的接入点 |
|---|---|---|
| [`AP_TemperatureSensor.h`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor.h) | frontend、实例 state、backend 指针 | 增加 class 声明/friend（若当前风格需要） |
| [`AP_TemperatureSensor.cpp`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp) | 根据 `TYPE` 创建 backend，统一 update/log | include 新 backend，并在 switch 末尾追加类型 |
| [`AP_TemperatureSensor_Params.h`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor_Params.h) | `Type` 枚举和 BUS/ADDR/SRC 参数 | 在现有 `TMP119=10` 后追加 MCP9808 值，不重排 |
| [`AP_TemperatureSensor_Params.cpp`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor_Params.cpp) | `TEMPn_TYPE` 元数据 | 追加新显示值，保留全部旧值 |
| [`AP_TemperatureSensor_Backend.h`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor_Backend.h) | backend 抽象和 `set_temperature()` | 新 driver 继承它，不另造 frontend |
| [`AP_TemperatureSensor_Backend.cpp`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor_Backend.cpp) | 更新时间戳、公共健康度、`TEMP` 日志和外部映射 | 成功样本必须经 `set_temperature()` 提交 |
| [`AP_TemperatureSensor_config.h`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor_config.h) | feature guards 和实例数量 | 增加 MCP9808 guard |
| [`AP_TemperatureSensor_TMP119.cpp`](../../../libraries/AP_TemperatureSensor/AP_TemperatureSensor_TMP119.cpp) | 当前 I²C 16-bit 温度 backend 范例 | 参考 device、semaphore、retry、callback 风格，不复制协议常量 |

当前 frontend 的公共契约是：

- 温度 state 使用摄氏度 `float`。
- backend 每次成功调用 `set_temperature()` 时更新 `last_time_ms`。
- 公共 `healthy()` 要求至少有过一次样本，且最近样本距当前时间小于 5 秒。
- `AP_TemperatureSensor::update()` 统一调用 backend 并按配置写 `TEMP` 日志。
- `TEMPn_SRC` 可把温度映射给 ESC、motor、battery 等其他组件；第一版建议 `SRC=None`，只读取和记录。

因此，不能在 `mode_*.cpp`、failsafe 或 scheduler 中直接操作 MCP9808 I²C。

## 3. 最小数据流

```text
TEMPn_TYPE/BUS/ADDR
 -> AP_TemperatureSensor::init()
 -> AP_TemperatureSensor_MCP9808::init()
 -> HAL I2C device + ID verification
 -> periodic callback reads ambient-temperature register
 -> raw flags/sign/value decode
 -> set_temperature(degC)
 -> frontend health / TEMP log / optional SRC mapping
```

## 4. 预计修改文件

| 文件 | 修改内容 |
|---|---|
| `libraries/AP_TemperatureSensor/AP_TemperatureSensor_MCP9808.h` | backend 类、默认地址、私有寄存器读函数和 timer 声明 |
| `libraries/AP_TemperatureSensor/AP_TemperatureSensor_MCP9808.cpp` | probe、ID 校验、周期读取和温度换算 |
| `libraries/AP_TemperatureSensor/AP_TemperatureSensor_Params.h/.cpp` | 在末尾追加 Type 和元数据 |
| `libraries/AP_TemperatureSensor/AP_TemperatureSensor.h/.cpp` | 声明、include 和 factory switch |
| `libraries/AP_TemperatureSensor/AP_TemperatureSensor_config.h` | `AP_TEMPERATURE_SENSOR_MCP9808_ENABLED` guard |
| `Tools/scripts/build_options.py` | 只有当前构建选项体系要求单独裁剪该 backend 时才增加；不能无证据修改 |
| 对应测试目录 | raw conversion、ID、总线失败和 freshness 测试 |

公共库代码不能依赖 ArduSub 的 `Sub` 对象。其他车辆即使没有启用该 backend，核心库也必须能在 feature guard 关闭时编译。

## 5. Driver 设计

### 5.1 `init()`

建议顺序：

1. 用 `set_default()` 设置候选默认地址 `0x18`，不覆盖用户已保存的 `TEMPn_ADDR`。
2. 通过 `hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address)` 获取设备。
3. 设备为空时直接返回，保持不健康；不得 panic 或阻塞启动。
4. 持有设备 semaphore，初始化阶段使用较高重试次数。
5. 读取 Manufacturer ID 和 Device ID/Revision；二者必须按 datasheet 检查。
6. 若第一版写 Resolution/Config，必须检查写入成功，并优先回读验证。
7. 降低运行期 retry，注册适合传感器转换速率的 periodic callback。

probe 失败不能伪造 `0 °C` 样本，因为那会让 frontend 误判为健康。

### 5.2 温度解码

伪代码只表达位处理顺序：

```cpp
uint16_t raw;
if (!read_u16_be(MCP9808_REG_AMBIENT_TEMP, raw)) {
    return; // 不更新时间戳，公共 freshness 最终转为 unhealthy
}

// 先去掉告警状态位，再按 datasheet 解释 sign 和 12-bit magnitude。
const bool negative = (raw & MCP9808_TEMP_SIGN_BIT) != 0;
const uint16_t magnitude = raw & MCP9808_TEMP_VALUE_MASK;
float temperature_c = magnitude * 0.0625f;
if (negative) {
    temperature_c -= 256.0f;
}

if (isfinite(temperature_c) && within_datasheet_range(temperature_c)) {
    set_temperature(temperature_c);
}
```

`MCP9808_TEMP_SIGN_BIT`、`MCP9808_TEMP_VALUE_MASK` 和合法量程必须从指定 datasheet 写成有名称的常量，不能在代码里散落 magic number。

### 5.3 周期与线程安全

- periodic callback 运行在 HAL device 的周期任务上下文，不在回调里发送 GCS 文本或执行长时间工作。
- I²C transaction 失败时快速返回，让下一周期重试。
- `set_temperature()` 已通过 backend semaphore 更新 frontend state；不要再建立一套无证据的重复锁。
- 采样周期必须低于传感器实际转换能力，并留出总线余量。
- `update()` 可以保持空实现，由 callback 更新样本，沿用 TMP119 等现有 backend 模式。

## 6. 参数接口

候选配置示例：

```text
TEMP1_TYPE = 11        # 仅当正式追加枚举值 11 后成立
TEMP1_BUS  = <实际 I2C bus>
TEMP1_ADDR = 24        # 0x18 的十进制显示；以地面站格式为准
TEMP1_SRC  = 0         # None，仅记录
```

兼容规则：

- 只能在 `Type` 末尾追加 `MCP9808=11`，不能改变 0–10。
- `TYPE` 的 `@Values` 必须同步。
- `BUS`、`ADDR`、`SRC`、`SRC_ID` 已能表达第一版需求，不新增重复参数。
- `AP_GROUPINFO` 的现有索引 1–5 不改变。
- 如果后端未来需要专属参数，要先审计 backend 参数索引占用表；当前公共注释已标记 Analog、DroneCAN 和 MAX31865 的占用范围。

## 7. 软件测试

### 7.1 纯换算测试

建议把 raw-to-degC 转换提取为无 I/O 的小函数，至少覆盖：

| 输入情形 | 期望 |
|---|---|
| `0 °C` 编码 | 精确得到 `0.0` |
| `25 °C` 编码 | 得到 `25.0` |
| 负温编码 | 正确处理 sign，不把标志位当数值 |
| 正负边界 | 与 datasheet 公式一致 |
| 三个告警 flag 组合 | 温度值不受 flag 污染 |

测试向量应在测试注释中标注数据手册章节，避免后续“修测试迎合错误实现”。

### 7.2 Backend/Frontend 行为

| 场景 | 通过标准 |
|---|---|
| Manufacturer ID 不匹配 | 不注册有效样本，保持 unhealthy |
| Device ID 不匹配 | 不把总线上其他设备误识别为 MCP9808 |
| 单次 I²C 失败 | 不更新 last-time，不发布伪温度 |
| 持续断线超过 5 秒 | 公共 frontend `healthy()` 变为 false |
| 恢复连接 | 获得新有效样本后恢复健康，不需重启则需明确证明 |
| 多实例 | 每个实例的 BUS/ADDR/state 不串线 |
| guard 关闭 | 相关源码被裁剪，其他构建不引用未定义类 |

### 7.3 构建与集成

- WSL SITL configure/build。
- Pixhawk4 目标板 build，并记录 flash 变化。
- 参数元数据检查，确认 `TEMP1_TYPE` 显示 MCP9808。
- 启动后验证 `TEMP` 日志的 instance、单位和更新频率。
- 保持 `TEMPn_SRC=None` 时，不应改变 battery、ESC 或控制行为。

## 8. 实物验证

1. 对照原理图核对供电电压、I²C 电平、上拉、A2/A1/A0 和实际地址。
2. 用逻辑分析仪或示波器确认地址、寄存器指针、ACK 和采样周期。
3. 室温下与可信参考温度计对比，记录稳态误差。
4. 做至少两个温度点的缓慢变化，检查符号、比例、响应和噪声。
5. 运行中断开 SDA/SCL 或传感器电源，确认 5 秒 freshness 超时和恢复行为。
6. 同总线挂接其他设备，检查 bus 错误不会阻塞主循环。

本案例不涉及推进器输出，但若将来把温度映射到 failsafe，必须另立需求，验证阈值、传感器错误、断线和误触发；不能顺带接入。

## 9. 验收与回退

验收条件：

- ID probe、寄存器字节序、符号和单位全部有 datasheet 证据。
- 纯换算、总线失败、freshness、多实例和 guard 均通过相应测试。
- SITL 与实际 Pixhawk 类板卡真实构建通过。
- 实物测温和断线测试有日志/总线记录。
- 现有 `TEMPn_TYPE` 0–10 的数值和行为不变。

回退优先使用配置：把对应 `TEMPn_TYPE` 设为 `0` 并重启。若 backend 影响启动或总线，再刷回加入 MCP9808 前的已验证固件。不要删除整个 `AP_TemperatureSensor` library，也不要为了一个设备在 ArduSub 中写板级 I²C 特例。
