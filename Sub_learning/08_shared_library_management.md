# 公共库管理：复用、扩展与新增边界

## 1. 公共库不是产品范围承诺

本仓库只维护 ArduSub 和 Pixhawk 类飞控板，但 `libraries/` 仍保留大量跨车辆组件。它们可能通过两条路径进入固件：

1. [ArduSub/wscript](../ArduSub/wscript) 明确列出的直接依赖，例如 `AC_AttitudeControl`、`AC_WPNav` 和 `AP_Motors`。
2. [ap_common_vehicle_libraries()](../Tools/ardupilotwaf/ardupilotwaf.py) 提供的公共车辆依赖，例如 `AP_AHRS`、`AP_NavEKF3`、`AP_Param`、`GCS_MAVLink` 和 `SRV_Channel`。

保留共享库表示 ArduSub 的编译和运行依赖它们，不表示本仓库要扩展为其他车辆产品。删除或修改公共库时，影响面通常大于 `ArduSub/`，必须提高审查和测试等级。

## 2. 代码应该放在哪一层

| 需求 | 首选位置 | 判断依据 |
|---|---|---|
| 某个 ArduSub 模式的进入、退出、目标和降级策略 | `ArduSub/mode*.cpp` | 只属于潜艇产品行为 |
| 多个 ArduSub 模式复用的小型转换 | `ArduSub/Attitude.cpp` 或明确的车辆辅助接口 | 仍依赖 Sub 的坐标或控制语义 |
| 通用姿态、位置或轨迹算法 | 现有 `AC_*` 控制库 | 算法不应知道具体车辆模式 |
| 通用传感器或估计输入 | 现有或新的 `AP_*` frontend/backend | 数据语义与车辆无关，可能有多个硬件后端 |
| 板卡引脚、总线、传感器实例和定时器 | `AP_HAL_ChibiOS/hwdef/<board>/` | 属于硬件描述，不属于控制模式 |
| MAVLink 消息解析和公共传输 | `GCS_MAVLink` 或现有库入口 | 协议行为跨车辆复用 |
| 伴随计算机驱动、ROS 节点或融合程序 | 独立仓库 | 不进入本固件仓库产品边界 |

先决定职责归属，再决定文件位置。为了少写几行而让共享库 include `ArduSub/Sub.h`，或让模式直接操作 STM32 GPIO，都会形成错误的反向依赖。

## 3. 默认采用最小复用顺序

实现新功能时按以下顺序选择：

```text
配置并复用已有能力
    -> 在 ArduSub 增加薄适配层
    -> 小幅扩展现有公共 API
    -> 为现有 frontend 增加 backend
    -> 最后才创建新的公共库
```

每向下一步移动，都要给出上一层无法满足需求的源码证据。新建 `AP_<Name>` 不应只是为了让目录“更整齐”，而应代表一个稳定、车辆无关且值得长期维护的抽象。

## 4. frontend/backend 是常用边界

[AP_VisualOdom](../libraries/AP_VisualOdom/) 展示了典型结构：

- `AP_VisualOdom` frontend 持有参数、统一健康状态、质量门槛和对外 API。
- `AP_VisualOdom_Backend` 定义不同数据来源都必须满足的接口。
- `AP_VisualOdom_MAV`、`AP_VisualOdom_IntelT265` 实现具体后端。
- `AP_VisualOdom_config.h` 定义 `HAL_VISUALODOM_ENABLED` 等编译条件。
- EKF 和车辆代码依赖 frontend，不依赖具体设备类。

增加新后端时应尽量保持 frontend 的物理语义不变。若新设备需要 frontend 无法表达的核心状态，例如 DVL 的 bottom lock、波束有效性和离底高度，应先判断这些状态是否属于一个新的通用传感器抽象，不能把它们伪装成某个现有字段。

## 5. 什么时候值得创建新的 AP_DVL

只有同时具备以下证据时，才评估新的 `AP_DVL`：

- 已知准确设备型号和完整协议，而不是猜测“通用 DVL 串口格式”。
- 有真实原始录包、厂商解码结果、字节序、比例、校验和时间戳证据。
- 能定义车辆无关的 DVL 数据模型：速度坐标系、bottom/water track、质量、波束、离底高度和健康状态。
- 能说明 frontend 与一个或多个 backend 的边界。
- 现有 ExternalNav、RangeFinder 或其他接口无法无损表达需求。
- 有 parser 单元测试、超时/错误帧测试和目标板容量预算。

如果伴随计算机已经输出融合后的局部位置和速度，优先复用 MAVLink `ODOMETRY`、`AP_VisualOdom` 和 EKF3；这时新增 `AP_DVL` 反而会重复已经存在的数据链。

## 6. 公共 API 的设计要求

公共库接口至少要明确：

- 输入和输出的坐标系、正方向、单位和时间基准。
- 数据新鲜度、质量、有效性和重置语义。
- 所有权与生命周期：谁创建 frontend，谁创建 backend，谁可以持有指针。
- 调用频率、线程/调度上下文和是否允许阻塞。
- feature disabled 时的编译行为。
- 日志、参数和错误报告由哪一层负责。

不要返回含义模糊的 `bool good()`，再让每个调用者猜“good”是否包含超时、质量、初始化或 EKF 接受状态。安全关键状态应拆成可以被测试和记录的契约。

## 7. 参数属于稳定接口

公共库参数通常由 frontend 的 `var_info[]` 管理。维护时：

- 不改变已有 `AP_GROUPINFO` 索引。
- 新参数使用未占用索引并写完整元数据。
- 不复用历史保留索引来压缩编号。
- 默认值应保证旧配置仍有可预测行为。
- 参数名、单位和范围与 API 语义一致。
- backend 专用参数只有在 frontend 能稳定管理时才进入公共参数树。

如果一个实验功能还在频繁改变语义，先使用编译时常量、测试夹具或独立分支，不要过早固定公开参数。

## 8. feature guard 与体积

可选公共能力应有清晰 guard，例如 AP_VisualOdom 当前使用 `HAL_VISUALODOM_ENABLED`。审查时检查：

1. guard 关闭时所有声明、实现、日志和调用点仍能编译。
2. 核心组件不依赖可选组件；依赖方向只能从可选功能指向核心。
3. 板卡容量条件和默认启用策略有依据。
4. 新代码的 flash/RAM 增量有前后对比。
5. guard 名称表达能力，不绑定某一个产品型号。

不要为了让某块小容量板编译而删除安全检查，也不要在多个文件中复制互不一致的默认条件。

## 9. Waf 依赖管理

增加依赖前回答：

- 是 `ArduSub/wscript` 的车辆直接依赖，还是公共车辆依赖？
- 是否只因一个 include 就引入了整套可选组件？
- feature 关闭后链接依赖是否仍存在？
- 新库是否反向依赖车辆层？
- 是否误把 `modules/` 中的外部代码当作本仓库可直接修改的库？

删除库前使用 Waf 任务图、include、链接结果和固件尺寸证明 dependency closure。一次只删除一个逻辑依赖，并同时执行 SITL 与目标板构建。

## 10. 公共库测试分层

| 层 | 应证明的内容 |
|---|---|
| parser/unit test | 有效帧、截断帧、校验错误、边界值、NaN/Inf、字节序和比例 |
| frontend/backend test | 初始化、选择 backend、超时、质量下降、重置和状态恢复 |
| consumer integration | EKF、模式或控制器是否按契约使用数据 |
| SITL/autotest | 消息到控制行为、失效到降级的完整链路 |
| Pixhawk4 build | ChibiOS 编译、链接、flash/RAM 和 feature guard |
| 拆桨台架/水池 | 真实时序、噪声、方向、failsafe 和水动力影响 |

只测试车辆模式不能覆盖 parser；只测试 parser 也不能证明模式会安全降级。

## 11. 公共库变更交付清单

- 说明为什么修改公共库而不是 ArduSub 适配层。
- 列出所有消费者和 feature guard。
- 给出 API、参数、日志和存储兼容性影响。
- 给出 Waf 依赖和固件尺寸变化。
- 记录 unit、SITL、Pixhawk4 与硬件测试的真实结果。
- 说明上游稳定版升级时可能产生的冲突。
- 准备可以单独回退的提交，不与模式 UI 或大规模格式化混在一起。
