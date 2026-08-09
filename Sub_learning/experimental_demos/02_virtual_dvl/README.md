# Demo 02：增加一个虚拟 DVL 传感器 backend

## 学习目标

这个 demo 不假装实现某个商业 DVL 的真实驱动，而是用一个可控文本协议讲清最小传感器通路：串口发现、非阻塞解析、坐标/单位、有效性校验、健康度、日志和观测出口。

虚拟 DVL 的数据只被记录和转发，不融合到 AHRS/EKF，不参与飞行模式和推进器控制。这是故意设置的安全边界。

## 坐标和单位

速度向量使用机体系 FRD：

| 字段 | 方向 | 单位 |
|---|---|---|
| `VX` | 机头向前为正 | m/s |
| `VY` | 机体向右为正 | m/s |
| `VZ` | 机体向下为正 | m/s |
| `quality` | 0–100；0 表示健康度为 false | % |

它不是 North/East/Down 地固速度。未来接入真实设备时，必须根据设备 datasheet 审计轴向、安装旋转、时间戳和速度协方差，不能只换串口报文格式。

## 输入协议

```text
$DVLD,<seq>,<vx>,<vy>,<vz>,<quality>*HH\r\n
```

示例：

```text
$DVLD,7,0.250,-0.100,0.050,80*09
```

`HH` 是从 `D`（`DVLD` 的首字符）到 `quality` 末字符的逐字节 XOR，输出两位十六进制。以仓库脚本实际生成的校验值为准，不要手工照抄示例构造测试。

解析器在 `DemoDVL::update()` 中每次最多消费 256 字节，并逐字符累积一行；不会等待整帧而阻塞 fast loop。以下情况计入 `bad_frames`：

- 行溢出；
- 帧格式或字段数量错误；
- 十六进制或 XOR 校验失败；
- 浮点数不是有限值；
- quality 超出 0–100；
- 任一速度分量超过 `DVL_MAX_VEL`。

## 状态和健康度

`DemoDVL::State` 保存：

- 最新机体系速度；
- sequence；
- 最新有效帧时间；
- good/bad frame 计数；
- quality。

健康度同时要求：功能已启用、找到了 UART、至少收到一帧、quality 非零、数据年龄不超过 `DVL_TIMEOUT`。这里的健康度只是 backend 状态，不会触发任何 failsafe。

## 源码入口

| 文件 | 职责 |
|---|---|
| `ArduSub/demo_dvl.h/.cpp` | frontend 状态与实验 backend 合并在一个车辆局部类中，便于初学者阅读 |
| `libraries/AP_SerialManager/AP_SerialManager.h` | 新协议 identity `SerialProtocol_LearningDVL = 51` |
| `ArduSub/system.cpp` | GCS UART 初始化完成后调用 `demo_dvl.init()` |
| `ArduSub/Sub.cpp` | 以 50 Hz 调度接收解析 |
| `ArduSub/sensors.cpp` | 调用 update，并在另一个低频任务中转发观测值 |

生产级共享传感器通常应拆成 `AP_*` frontend/backend，并考虑多实例、总线探测、线程安全和跨车辆复用。这个单实例车辆局部类只为缩短教学路径，不应直接复制为正式库设计。

## 日志

每个有效帧写入 `DDVL`：

```text
TimeUS, Seq, VX, VY, VZ, Qual
```

日志是检查丢帧、方向、单位、时间和数据范围的主要证据。看到日志不等于数据已经进入 EKF。

## 协议自测

```bash
python3 Sub_learning/experimental_demos/tools/send_virtual_dvl.py --self-test
```

这个测试只覆盖编码、XOR 和回包解码。SITL 串口集成与硬件接线见 Demo 03。

## 从虚拟 DVL 走向真实 DVL 前必须补齐

1. 锁定设备型号和官方协议版本。
2. 区分 bottom-track、water-track、无底锁和无效解。
3. 处理设备时间戳、延迟、帧率和丢包。
4. 定义安装位置/姿态补偿和速度协方差。
5. 设计 EKF 接口、创新门限、故障隔离和 failsafe；先 shadow/log-only 对比，再考虑闭环。
