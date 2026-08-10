# 实验 02：把串口解析器做成一个显式 Waf 库

## 为什么不继续使用字符串

`PING\n` 能验证接线，却不能代表可靠传感器/命令协议。真实串口会遇到半包、粘包、错位、噪声和断线。本实验实现一个固定 21 字节帧：有限长度、版本号、消息号、序号、范围检查和 CRC16。

解析器只保存最近一帧并回复 ACK，**不连接 motors**。先把“数据有没有正确进入飞控”与“数据能不能控制载具”分开，是安全二次开发的基本方法。

本实验应从 `learning/minimal-pixhawk1` 新建独立学生分支，不要与实验 01 同时运行；两个实验都占用协议 28 的第一个串口实例，同时读取会产生竞争。

## 1. 帧格式

所有多字节整数采用 little-endian：

| 偏移 | 长度 | 字段 | 约束 |
|---:|---:|---|---|
| 0 | 1 | Header 1 | `0xAA` |
| 1 | 1 | Header 2 | `0x55` |
| 2 | 1 | Version | `1` |
| 3 | 1 | Message ID | `0x31` |
| 4 | 1 | Sequence | 0–255 循环 |
| 5 | 1 | Payload length | `13` |
| 6 | 1 | Flags | bit0 为 enable/deadman；本实验只记录，不执行 |
| 7 | 12 | 六个 `int16` | roll、pitch、yaw、heave、forward、lateral，各为 `[-1000,1000]` |
| 19 | 2 | CRC16-CCITT | 初值 `0xFFFF`，覆盖偏移 2–18 |

有效帧返回三字节 ACK：`0xAC, sequence, 0x00`。CRC、版本、长度或范围错误的帧不会更新最近样本，也不会 ACK。

## 2. 复制库

```bash
cp -r \
  Sub_learning/labs/02_framed_serial_library/files/libraries/AP_LearningSerial \
  libraries/
```

新库只依赖极简分支已经保留的 `AP_HAL` 与 `AP_SerialManager`。

## 3. 修改 Waf 白名单

在 `ArduSub/wscript` 的 `ap_libraries=[...]` 中按字母顺序加入：

```python
'AP_LearningSerial',
```

这是本课程第一次显式修改构建依赖。忘记它时，`Sub.h` 可能报头文件找不到，或链接器找不到 `AP_LearningSerial::init/update`。不要为了省事恢复 `ap_common_vehicle_libraries()`；那会把被裁掉的通用能力重新拉回来，破坏极简分支的依赖边界。

## 4. 创建对象

在 `ArduSub/Sub.h` 的库 include 区加入：

```cpp
#include <AP_LearningSerial/AP_LearningSerial.h>
```

在 `Sub` 私有成员区加入：

```cpp
AP_LearningSerial learning_serial;
```

对象保存接收状态机、最近命令、时间戳和好/坏帧计数。它必须长期存在，不能在 scheduler 回调中每次临时构造。

## 5. 初始化与调度

在 `ArduSub/system.cpp` 的 `Sub::init_ardupilot()` 末尾、`ap.initialised = true;` 之前加入：

```cpp
learning_serial.init();
```

在 `ArduSub/Sub.cpp` 的 GCS 任务之后、logging 任务之前加入：

```cpp
SCHED_TASK_CLASS(AP_LearningSerial, &sub.learning_serial, update, 100, 120, 45),
```

这里选择 100 Hz，使 115200 bit/s 下的短帧能及时消费；单次仍最多读取 64 字节，避免串口洪泛长期占用主循环。

## 6. 串口设置与构建

仍然使用：

```text
SERIAL2_PROTOCOL = 28
SERIAL2_BAUD     = 115
SERIAL2_OPTIONS  = 0
```

构建：

```bash
git diff --check
./waf configure --board Pixhawk1 \
    --out build_student_serial_library \
    --no-submodule-update
./waf sub -j4
```

构建前检查 `git diff -- ArduSub/wscript`，确认只增加一个库名，没有恢复公共库全集。

## 7. PC 验证

在 Windows PowerShell 安装 pyserial 后发送 40 个全零、未使能帧：

```powershell
py Sub_learning\labs\02_framed_serial_library\tools\send_serial_command.py COM6
```

预期每个序号都出现 `ack=ok`，最后：

```text
sent=40 missed=0
```

即使使用 `--enable --axes 100 0 0 0 0 0`，本实验也不会产生推进器请求，因为没有任何模式读取 `learning_serial.get_command()`。这个现象应作为架构隔离的验证，而不是缺陷。

## 8. 主动制造错误

完成正常测试后，学习者应临时修改 PC 工具做三次故障注入：

1. CRC 最后异或 `0x0001`：应超时，无 ACK。
2. 任一轴设为 1001：应超时，无 ACK。
3. 删掉一个字节后继续发送正确帧：解析器应重新寻找 `0xAA 0x55` 并恢复。

不要同时修改飞控解析器和 PC 生成器来“让错误通过”；协议两端必须由同一份格式表独立核对。

## 9. 为什么这些细节重要

| 设计 | 防止的问题 |
|---|---|
| 固定最大帧长 | 动态分配、缓冲区溢出 |
| 每次读取预算 | UART 洪泛拖慢主循环 |
| version/message/length | 把其他协议误当成有效命令 |
| CRC16 | 噪声或错位后误接受 |
| 轴范围验证 | 上位机单位错误直接进入策略层 |
| `received_ms` | 后续消费者可以判断数据是否过期 |
| frontend 不调用 motors | 通信健康与控制准入解耦 |

## 10. 复盘问题

1. 新库为什么必须加入 `ArduSub/wscript`，而实验 01 的车辆 `.cpp` 不需要？
2. `_frame_index` 在第二个 header 错误时怎样重新同步？
3. `healthy(timeout_ms)` 为什么用接收时间，而不是只看 `_has_command`？
4. 如果未来接收的是传感器测量，坐标系、单位、质量和时间戳还需要增加什么字段？
