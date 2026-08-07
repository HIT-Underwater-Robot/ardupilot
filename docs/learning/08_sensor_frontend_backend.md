# 第 8 章：传感器前端、后端与自动探测

ArduPilot 为同类传感器写很多具体驱动，不是因为系统设计混乱，而是不同芯片的寄存器、时序、标定公式、错误恢复和总线能力确实不同。

它用 Frontend/Backend 把差异隔开：

~~~text
车辆和 EKF
    ↓ 使用统一接口
Frontend
    ├─ 参数
    ├─ 多实例
    ├─ primary
    ├─ health
    └─ 对外单位
        ↓
Backend A / B / C
    ↓
HAL I2C / SPI / UART / CAN
~~~

## 1. 文件地图

以深度传感器为主线：

1. **ArduSub/system.cpp**：初始化并选择水压实例；
2. **ArduSub/sensors.cpp**：周期读取和车辆健康状态；
3. **libraries/AP_Baro/AP_Baro.h/.cpp**：Frontend；
4. **libraries/AP_Baro/AP_Baro_Backend.h/.cpp**：Backend 基类；
5. **libraries/AP_Baro/AP_Baro_MS5611.h/.cpp**：MS56xx 与 MS5837 驱动；
6. **libraries/AP_HAL/I2CDevice.h**：总线抽象。

## 2. Frontend 负责什么

AP_Baro 是上层唯一看到的统一入口，负责：

- 保存各实例 pressure、temperature、altitude；
- 保存类型、bus ID 和更新时间；
- 提供 healthy、get_pressure、get_altitude；
- 管理 primary；
- 保存公共参数；
- 创建和更新各 Backend；
- 向日志和 EKF 提供一致数据。

它不应包含每种芯片的全部寄存器公式。

## 3. Backend 负责什么

具体 Backend 知道：

- 芯片地址或 SPI 设备；
- 初始化命令；
- PROM/寄存器含义；
- 原始 ADC 读取时序；
- 温度补偿；
- 压力换算；
- 设备 ID；
- 失败重试。

Backend 最终把统一单位的数据发布给 Frontend。上层不需要知道它来自 MS5837-30BA 还是 Keller。

## 4. 为什么不能只保留一个传感器文件

即使项目目前只装 MS5837，AP_Baro 仍承担：

- Pixhawk1 板载 MS5611；
- 水压实例 MS5837；
- SITL 模拟 Backend；
- 日志和 EKF 公共接口；
- 多实例健康和选择。

删除其他驱动可以在编译开关层讨论，但不能把 Frontend/Backend 架构一起删除，否则板载检测、SITL 和公共接口都会破坏。

## 5. Backend 怎样被创建

AP_Baro::init() 根据平台和配置执行多条探测路径：

- SITL 模拟；
- DroneCAN；
- ExternalAHRS；
- hwdef 生成的 HAL_BARO_PROBE_LIST；
- 旧板型检测；
- 指定外部 I2C bus；
- 外部 I2C bus mask 和 BARO_PROBE_EXT；
- MSP。

探测函数概念过程：

~~~text
取得 AP_HAL::Device
    ↓
调用 AP_Baro_MS5837::probe
    ↓
分配 Backend
    ↓
读芯片并校验
    ├─ 失败：释放，不登记
    └─ 成功：register_sensor
              ↓
           add_backend
~~~

probe 返回 nullptr 表示当前地址不是该设备或初始化失败。

## 6. instance 是什么

Frontend 为成功登记的传感器分配实例编号。

instance 不是固定物理插座，也不一定每次配置变化后保持相同。代码应配合类型、bus ID、primary 和参数理解实例。

ArduSub 初始化遍历 barometer.num_instances()，找到类型为 BARO_TYPE_WATER 的实例，记录 depth_sensor_idx，并把它设为 primary。

这比硬编码“第二个 barometer 永远是深度计”可靠。

## 7. MS5837 实际探测

当前驱动位于 AP_Baro_MS5611 文件中，是因为 MS5837 与 MS56xx 共享大量命令和转换框架。

关键步骤：

1. I2C 地址默认 0x76；
2. 发送 reset；
3. 读取 PROM 标定系数；
4. 登记 sensor instance；
5. 设置设备类型和 bus ID；
6. 注册约 100 Hz 周期回调；
7. 根据标定系数区分 02BA/30BA；
8. 标记 Frontend 类型为 BARO_TYPE_WATER。

文件名包含 MS5611 不代表里面只能有 MS5611。

## 8. 原始值如何变为深度信息

~~~text
MS5837 D1/D2 原始 ADC
    ↓
Backend 按 datasheet 做一阶/二阶温度补偿
    ↓
pressure + temperature
    ↓
copy_to_frontend
    ↓
AP_Baro::update
    ↓
压力/参考压力/水密度相关换算
    ↓
barometric altitude/depth 语义
    ↓
ArduSub read_barometer
~~~

不要在 I2C 驱动内直接写推进器控制；驱动只提供测量和健康。

## 9. update 与周期回调

很多总线 Backend 用 HAL 的周期回调持续采样，把结果累积在受保护状态中。车辆任务调用 Frontend update 时再取出稳定结果。

这意味着看到 Sub 以 10 Hz 调用 read_barometer，不代表芯片只以 10 Hz 发起所有底层转换。

需要分别查：

- Backend 的采样频率；
- Frontend 的 publish/update 频率；
- EKF 的观测融合时刻；
- 车辆健康检查频率。

## 10. healthy 不等于数值看起来正常

健康通常综合：

- 是否收到过更新；
- 距最后更新是否超时；
- 是否完成校准；
- 数值是否有限；
- Backend 是否报告错误；
- 实例是否有效。

ArduSub 还把 water barometer 的 healthy 结果复制到 sensor_health.depth，供解锁和 failsafe 使用。

传感器新增时必须设计：

- 未连接；
- 启动一半；
- 断线；
- 卡住不更新；
- 发送 NaN/极值；
- 质量下降；
- 恢复连接。

## 11. primary 的含义

primary 是 Frontend 对外默认实例，不等于其他实例停止更新，也不等于 EKF 无条件相信它。

ArduSub 找到 water 类型后将其设为 primary，是为了让深度相关的 barometric altitude 接口使用水压实例。

## 12. UART 传感器的标准 Backend 形状

若设备直接接 TELEM2 且协议不是 MAVLink，典型结构是：

~~~text
参数选择 SerialProtocol
    ↓
SerialManager find_serial(protocol, instance)
    ↓
Backend 保存 UARTDriver
    ↓
init 配波特率/缓冲
    ↓
周期 update 读取字节
    ↓
状态机找帧头、长度、CRC
    ↓
转换单位和坐标
    ↓
发布 Frontend 数据、时间和质量
~~~

解析器必须支持半帧、粘包、错位、坏 CRC 和超时，不能假设一次 read 就返回完整结构体。

## 13. DVL/USBL 当前边界

本仓库检索不到通用 AP_DVL Frontend/Backend。ExternalAHRS 的某些厂商后端可能包含 DVL 数据，但它不是任意 DVL 串口的通用驱动。

所以两条方案是：

### 方案 A：香橙派融合

~~~text
DVL + USBL + 视觉/声学
    ↓
香橙派时间同步、坐标转换、融合
    ↓ MAVLink ODOMETRY
AP_VisualOdom
    ↓
EKF ExternalNav
~~~

这是现有架构最直接的路径。

### 方案 B：直接接 Pixhawk

需要为设备协议新增驱动或适配 Backend，并明确输出到哪个现有 Frontend/EKF 接口。仅配置 UART 不会自动融合。

## 14. 自动探测的代价

优点：

- 同一固件兼容多个传感器；
- 换型号无需改车辆代码；
- 多实例和冗余更容易。

代价：

- 固件包含更多驱动；
- 启动探测更复杂；
- 地址冲突和误探测要处理；
- 阅读文件较多。

教学精简应先用编译开关验证依赖，不应把所有 Backend 手工揉成一个大文件。

## 15. 新传感器审查表

- 物理总线和电气要求；
- 帧/寄存器协议；
- 输出单位和坐标；
- 时间戳来自哪里；
- 采样与发布频率；
- 标定参数；
- 健康、质量和超时；
- 多实例；
- 日志；
- SITL 模拟；
- 解锁与失效行为；
- 直接接入和伴随计算机路径是否会重复输入。

## 16. 验收问题

1. Frontend 与 Backend 分别负责什么？
2. 为什么 MS5837 与 MS5611 在同一驱动文件？
3. probe 失败应如何表现？
4. instance 为什么不能简单当作插座编号？
5. 10 Hz Frontend 调用为何不等于 10 Hz 底层采样？
6. healthy 应覆盖哪些故障？
7. primary 是否会停止其他实例？
8. UART 解析为何必须用状态机？
9. 当前仓库是否有通用 AP_DVL？
10. 直接 UART 接 DVL 还缺哪一层？
