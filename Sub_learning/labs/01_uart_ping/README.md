# 实验 01：UART Ping——第一次让 PC 与 Pixhawk1 对话

## 实验结论先行

本实验完成后，PC 发送一行 `PING\n`，Pixhawk1 返回 `PONG\n`。代码不读取姿态、不选择模式、不调用 motors，因此即使串口输入异常也没有推进器控制权限。

## 你将亲手接通的链路

```text
Windows Python + USB 转 3.3 V TTL
        ↓ 115200 bit/s
Pixhawk1 TELEM2 / SERIAL2
        ↓
AP_SerialManager 查找端口
        ↓
LearningUARTPing::update() @ 50 Hz
        ↓
逐字节收行 → PING/PONG
```

## 1. 硬件准备

- Pixhawk1，一开始不要连接推进器电源。
- 支持 3.3 V UART 逻辑的 USB 转 TTL。
- 三根信号线：适配器 TX → Pixhawk RX、适配器 RX ← Pixhawk TX、GND ↔ GND。

不要连接适配器的 5 V/VCC。不同 Pixhawk1 兼容板的连接器方向可能不同，必须按本板原理图或端口丝印确认 TX、RX、GND，不能按插头外观猜测。

TELEM2 在本分支映射为 `SERIAL2`。实验期间让 Mission Planner/QGroundControl 继续使用 USB 或 TELEM1，避免两个协议抢占同一 UART。

## 2. 配置串口参数

通过 USB 连接 GCS，设置：

```text
SERIAL2_PROTOCOL = 28
SERIAL2_BAUD     = 115
SERIAL2_OPTIONS  = 0
```

保存后重启。`SERIAL2_BAUD=115` 在参数中表示 115200 bit/s。

协议值 28 在 ArduPilot 中名为 Scripting。极简 Pixhawk1 分支已经关闭 Lua，本实验借用这个现成协议标识让 `AP_SerialManager` 安全地找到一个专用端口，不修改公共协议枚举。正式产品驱动不应长期借用无关协议号，应单独设计 frontend/backend 和协议所有权。

## 3. 复制参考源码

从仓库根目录执行：

```bash
cp Sub_learning/labs/01_uart_ping/files/ArduSub/learning_uart_ping.h ArduSub/
cp Sub_learning/labs/01_uart_ping/files/ArduSub/learning_uart_ping.cpp ArduSub/
```

先运行 `git status --short`，确认只出现这两个新文件。

## 4. 把对象接入 `Sub`

在 `ArduSub/Sub.h` 的其他本地 include 附近加入：

```cpp
#include "learning_uart_ping.h"
```

在 `Sub` 的私有成员区加入：

```cpp
LearningUARTPing learning_uart_ping;
```

为什么不能只放两个源码文件？因为解析器需要保存 UART 指针、半行文本和当前长度；这些状态必须属于一个生命周期与 `Sub` 一致的对象。

## 5. 接入初始化生命周期

`AP_Vehicle::setup()` 已在调用 `Sub::init_ardupilot()` 前完成 `serial_manager.init()`。因此在 `ArduSub/system.cpp` 的 `Sub::init_ardupilot()` 末尾、`ap.initialised = true;` 之前加入：

```cpp
learning_uart_ping.init();
```

如果在 SerialManager 之前查找端口，会得到空指针或与其他协议争用；这就是初始化顺序必须从真实源码核对的原因。

## 6. 接入调度器

在 `ArduSub/Sub.cpp` 的 `scheduler_tasks[]` 中，放在两个 GCS 任务之后、logging 任务之前：

```cpp
SCHED_TASK_CLASS(LearningUARTPing, &sub.learning_uart_ping, update, 50, 100, 45),
```

字段依次表示对象类型、对象地址、成员函数、50 Hz、预计最多 100 μs、优先级 45。任务表要求按优先级从小到大排列。

`ArduSub/` 顶层 `.cpp` 会由车辆静态库自动发现，所以本实验不修改 `ArduSub/wscript`。这不是“没有编译链”，而是文件发现规则与库白名单规则不同。

## 7. 构建

```bash
git diff --check
./waf configure --board Pixhawk1 \
    --out build_student_uart_ping \
    --no-submodule-update
./waf sub -j4
```

构建成功只能证明接口和链接正确，不能证明接线或 UART 行为正确。

## 8. 烧录与 PC 验证

先在 Windows PowerShell 安装 pyserial：

```powershell
py -m pip install pyserial
```

烧录实验固件并确认 TELEM2 接线后运行：

```powershell
py Sub_learning\labs\01_uart_ping\tools\ping_uart.py COM6
```

把 `COM6` 替换为设备管理器中的实际端口。WSL 默认不能直接访问 Windows COM 口，因此构建可在 WSL 中完成，串口工具推荐在 Windows PowerShell 中运行。

预期输出：

```text
reply='PONG'
```

## 9. 故障定位

| 现象 | 优先检查 |
|---|---|
| 打不开 COM 口 | 端口号错误，或被串口助手占用 |
| 一直超时 | TX/RX 未交叉、没有共地、固件未重启、协议号不是 28 |
| 收到乱码 | 两端波特率不同，或适配器不是 3.3 V UART |
| 只能收到 `ERR` | PC 发送了额外字符或行结尾不是 `\n` |
| GCS 从 TELEM2 断开 | 正常；该端口已从 MAVLink 改作实验 UART，改用 USB/TELEM1 |

## 10. 复盘问题

1. 为什么使用 `AP_SerialManager::find_serial()`，而不是在公共库里写死 `hal.serial(2)`？
2. 为什么 `update()` 每次最多读取 32 字节？
3. 如果只复制 `.cpp/.h`，但不增加成员、初始化或 scheduler，分别会发生什么？
4. 为什么这个实验不需要修改 `wscript`？
