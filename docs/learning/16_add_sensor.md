# 第 16 章：新增传感器的两条路径

新增传感器之前先决定：Pixhawk 需要原始测量、统一测量，还是伴随计算机已经融合后的状态。

~~~text
路径 A：设备直接接 Pixhawk
    设备协议 → Backend → Frontend → EKF/车辆

路径 B：设备接香橙派
    设备 → 香橙派融合 → MAVLink ODOMETRY → ExternalNav
~~~

两条路径可以由产品配置兼容，但同一组高度相关数据通常不应同时送入 EKF。

## 1. 开发前必须拿到的资料

- 准确型号和固件版本；
- 电气接口、电压和隔离；
- 串口波特率或 I2C/SPI 地址；
- 完整协议手册；
- 帧头、长度、字节序、CRC；
- 坐标系和安装方向；
- 单位、量程、分辨率；
- 设备时间戳；
- quality/status 定义；
- 启动、复位和故障行为；
- 实际录制的原始数据。

缺这些材料时可以设计架构，不能诚实地完成可上机驱动。

## 2. 先写数据契约

用一页表定义：

| 项目 | 例子 |
|---|---|
| 物理量 | body velocity / local position |
| 坐标 | sensor frame、BODY_FRD 或 NED |
| 单位 | m/s、m、Pa |
| 频率 | 10/50/100 Hz |
| 时间 | 设备时钟或接收时钟 |
| 延迟 | 固定、可变 |
| 质量 | 0～100 或厂商状态 |
| 无效值 | NaN、特定码、丢帧 |
| reset | 重定位、重启 |

如果这张表无法填写，暂时不要写 EKF 接口。

## 3. 路径 A：选择现有 Frontend

先问设备属于哪一类：

- 压力/深度：AP_Baro；
- 距离：AP_RangeFinder；
- 磁场：Compass；
- IMU：AP_InertialSensor；
- 外部姿态系统：AP_ExternalAHRS；
- 外部位姿/速度：AP_VisualOdom 或明确的 AHRS 外部导航接口。

能复用现有 Frontend 时新增 Backend，不要另建一套车辆专属全局变量。

若没有语义匹配的 Frontend，再讨论建立新库；不能因为名字相近把 DVL 塞进 Airspeed。

## 4. UART Backend 的真实参考

可阅读 AP_RangeFinder_Backend_Serial：

~~~text
serial_instance
    ↓
AP::serialmanager().find_serial(
    SerialProtocol_Rangefinder, instance)
    ↓
AP_HAL::UARTDriver
    ↓
各型号 Backend 解析自己的帧
~~~

这个参考展示串口获取和多实例，但不能直接复制其距离数据接口作为 DVL 速度接口。

## 5. UART 解析状态机

解析器应能处理：

~~~text
寻找帧头
→ 读取长度/类型
→ 等待完整 payload
→ 校验 CRC
→ 解码字段
→ 检查范围和状态
→ 发布
~~~

同时覆盖：

- 一次只到一个字节；
- 两帧粘在一起；
- 从半帧开始；
- payload 中出现帧头字节；
- 坏 CRC 后重新同步；
- 超时丢弃半帧；
- UART buffer 溢出。

不要把 UART 缓冲区强制 reinterpret_cast 成 C struct；这会引入对齐、字节序和长度风险。

## 6. Backend 生命周期

一个规范 Backend 通常包含：

1. probe 或显式创建；
2. 构造但不执行可能失败的大动作；
3. init 获取设备并确认身份；
4. register sensor instance；
5. 周期 update 或 HAL callback；
6. 发布统一数据；
7. 更新 last update 和 health；
8. 析构/失败时释放设备。

内存分配失败必须安全返回，不能保留半初始化指针。

## 7. 参数设计

可能需要：

- TYPE/ENABLE；
- serial protocol 和 instance；
- 安装位置；
- orientation；
- delay；
- scale；
- noise；
- quality minimum；
- address/bus；
- timeout。

优先复用 Frontend 已有参数。新增参数按第 5 章保持索引兼容和完整元数据。

## 8. 编译开关

非核心 Backend 用 AP_xxx_ENABLED 包围：

- 头文件 include；
- enum/type；
- probe；
- 对象创建；
- 运行调用。

若功能体积明显，还要在 build_options.py 表达选项和依赖。必须验证 enabled 与 disabled 两种构建。

## 9. MS5837 是现成范例

当前项目不需要重写 MS5837：

~~~text
AP_Baro::init
→ 外部 I2C probe
→ AP_Baro_MS5837::probe
→ PROM/标定/周期转换
→ BARO_TYPE_WATER
→ Sub 选择 depth_sensor_idx
→ sensor_health.depth
~~~

学习它的目标是掌握 Backend 模式，而不是复制一份私有 MS5837 驱动。

## 10. DVL 直接接 Pixhawk

当前仓库没有通用 AP_DVL。实际实现前要决定 DVL 输出：

- body velocity；
- bottom/water track 状态；
- altitude above bottom；
- delta position；
- 完整内部融合位姿。

不同输出可能进入不同接口，不能用一个 Vector3f 全部代替。

推荐设计评审先回答：

1. 是否创建独立 DVL Frontend；
2. 是否只做某厂商 UART Backend；
3. 数据以 raw measurement 还是 external navigation 送 EKF；
4. bottom lock 丢失怎样健康降级；
5. 与香橙派 ODOMETRY 如何互斥。

## 11. USBL 直接接 Pixhawk

USBL 常给低频绝对位置，可能还有时间延迟和离群点。驱动除串口解析外，必须处理：

- 坐标原点；
- 经纬度或局部坐标；
- 声学测量延迟；
- quality；
- 跳点；
- covariance；
- 与 DVL 高频速度的关系。

在没有时间同步和坐标定义前，不应直接把每帧位置写进 EKF。

## 12. 路径 B：香橙派融合

现有路径：

~~~text
设备驱动
    ↓
香橙派统一时间和坐标
    ↓
融合器
    ↓
MAVLink ODOMETRY
    ↓
GCS handle_odometry
    ↓
AP_VisualOdom_MAV
    ↓
AHRS writeExtNavData / VelData
    ↓
EKF ExternalNav source
~~~

优先使用已有 ODOMETRY，除非新数据语义确实无法表达。

## 13. 伴随计算机发送端验收

- frame_id 为 LOCAL_FRD；
- child_frame_id 为 BODY_FRD；
- q 已归一化；
- velocity 确实是 body FRD；
- time_usec 单调且语义一致；
- covariance 不是随意全零；
- reset_counter 仅在重置时变化；
- quality 有明确映射；
- 频率稳定；
- 断流可检测。

## 14. SITL 模拟

为新传感器建立可控输入：

- 正常匀速；
- 噪声；
- 固定 bias；
- 可变延迟；
- 掉线；
- CRC 错；
- quality 降低；
- reset 跳变。

测试不仅检查“能读到数字”，还检查 EKF、模式、pre-arm 和 failsafe 的反应。

## 15. 日志

至少记录：

- raw/decoded measurement；
- sample timestamp 与 receive timestamp；
- status/quality；
- covariance/noise；
- instance 和 bus；
- accepted/rejected；
- reject reason；
- health transition。

避免在高频循环发送文本代替二进制日志。

## 16. 合并前验收

- 编译开和关；
- SITL 正常/错误/超时；
- 参数保存重启；
- 多实例；
- 断线恢复；
- QGC 状态；
- Pixhawk1 Flash/RAM；
- 无推进器台架；
- 逻辑分析仪核对帧；
- 受控水池；
- 文档说明两条路径互斥策略。

## 17. 实施顺序

1. 先用香橙派 + ODOMETRY 跑通系统闭环；
2. 保存一组真实数据和基准日志；
3. 再开发 Pixhawk UART Backend；
4. 用相同运动数据对比两条路径；
5. 加参数选择输入路径；
6. 验证切换、重启和失联；
7. 最后才考虑自动切换。

自动无缝切换涉及估计状态对齐，不应作为第一版功能。
