# Demo 03：DVL 参数、USB-TTL 板载实验与数据转发

## 学习目标

这个 demo 在虚拟 DVL backend 外围补齐一条可操作的基础平台：PC 生成数据，经 USB 转 TTL 串口送入 Pixhawk；Pixhawk 校验和保存状态，再通过同一串口回状态，并通过另一条 MAVLink 链路发给地面站。

它演示参数和数据通路，不演示 DVL 导航融合。

## 参数设计

车辆顶层为 DVL 对象分配了新的参数 identity `245`，对象内使用固定 group index 1–4。没有移动或复用任何 ArduSub 4.7.0 已有参数索引。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `DVL_ENABLE` | 0 | 启用实验接收器；需重启 |
| `DVL_TIMEOUT` | 500 ms | 最后有效帧超过此年龄后 unhealthy |
| `DVL_MAX_VEL` | 5.0 m/s | 单分量绝对值上限，超限帧拒绝 |
| `DVL_TX_HZ` | 2 Hz | 串口状态回包和 MAVLink Named Value Float 的发送频率；0 关闭发送但保留接收/日志 |
| `SERIALn_PROTOCOL` | 按端口设置 | 设为 `51`，选择 Learning DVL |
| `SERIALn_BAUD` | 按设备设置 | 示例 `115`，即 115200 baud |

`n` 不是任意照抄的数字。必须根据目标 Pixhawk 的 hwdef 和物理接口映射选择空闲 UART；若该口正承担 MAVLink、GPS 或其他设备，不能直接覆盖。

## Pixhawk 接线

推荐把两条链路分开：

```text
PC ── USB-to-3.3V-TTL ── Pixhawk 空闲 TELEM/UART（虚拟 DVL）
PC/GCS ── Pixhawk 原生 USB 或另一 TELEM 口（MAVLink 观察）
```

USB-TTL 与飞控 UART 的最小接线：

| USB-TTL | Pixhawk UART |
|---|---|
| TX | RX |
| RX | TX |
| GND | GND |

安全要求：

- 使用与飞控兼容的 3.3 V TTL 电平；RS-232 电压不能直接接 Pixhawk。
- 不确定供电结构时不要连接适配器的 5 V/3.3 V 电源脚，只共地和接 TX/RX。
- 上电前用目标板原理图/hwdef 核对接口、电平和 `SERIALn` 映射。
- 推进器必须拆桨或物理隔离；本实验无需 armed。

## Pixhawk 到外部的两个出口

### 1. 同一 DVL UART 的状态回包

```text
$DVLA,<last_seq>,<healthy>,<good_frames>,<bad_frames>*HH\r\n
```

PC 脚本会解析并显示回包，可以快速证明 TX/RX 交叉接线、波特率、校验和健康度均正常。

### 2. MAVLink Named Value Float

| 名称 | 内容 |
|---|---|
| `DVL_VX`、`DVL_VY`、`DVL_VZ` | 最新机体系速度 m/s |
| `DVL_QUAL` | 质量 0–100 |
| `DVL_SEQ` | 最新序号；注意 MAVLink 此消息用 float，长时间运行后不适合当作无损 32 位序号 |
| `DVL_HLTH` | 1 healthy，0 unhealthy |

这是教学上最短的可视出口。产品化时应定义专用 MAVLink 消息或复用语义匹配的标准消息，并处理带宽、时间戳、协方差和多实例。

## PC 发生器

先安装 PySerial：

```bash
python3 -m pip install pyserial
```

Linux/WSL 直连示例：

```bash
python3 Sub_learning/experimental_demos/tools/send_virtual_dvl.py \
  --port /dev/ttyUSB0 --baud 115200 --rate 10 \
  --vx 0.25 --vy -0.10 --vz 0.05 --quality 80
```

Windows 示例：

```powershell
python Sub_learning\experimental_demos\tools\send_virtual_dvl.py `
  --port COM8 --baud 115200 --rate 10 `
  --vx 0.25 --vy -0.10 --vz 0.05 --quality 80
```

脚本的 `TX` 是发出的 `DVLD` 帧，`RX` 是飞控返回的 `DVLA` 状态。按 `Ctrl-C` 结束。

## SITL 复现实验

样例把 SITL 的 SERIAL5 暴露在 TCP 6795：

```bash
build/sitl/bin/ardusub \
  --model vectored \
  --serial5 tcp:6795 \
  --defaults Sub_learning/experimental_demos/sitl/virtual_dvl.parm \
  --wipe
```

另一个终端运行：

```bash
python3 Sub_learning/experimental_demos/tools/send_virtual_dvl.py \
  --port socket://127.0.0.1:6795 --rate 10 --count 20 \
  --vx 0.25 --vy -0.10 --vz 0.05 --quality 80
```

验证时至少检查：有效帧计数增加、坏帧为 0、healthy 为 true、停止发送超过 500 ms 后 `DVL_HLTH` 变为 0、超限/坏 checksum 被拒绝、`DDVL` 日志坐标和单位正确。

## 初学者应看到的架构关系

- `AP_Param` 决定配置的持久身份，不负责读串口。
- `AP_SerialManager` 决定哪个物理 UART 分配给哪种协议。
- scheduler 决定接收和转发何时运行。
- backend 负责解析与健康度，不应直接偷偷修改控制目标。
- GCS 转发是观测出口，不等于估计器输入。
