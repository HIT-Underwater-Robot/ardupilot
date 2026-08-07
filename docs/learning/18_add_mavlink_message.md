# 第 18 章：新增 MAVLink 消息

第一步不是写 XML，而是证明现有消息无法正确表达需求。

优先考虑：

- ODOMETRY：位姿、速度和质量；
- DISTANCE_SENSOR：距离；
- SCALED_PRESSURE：压力类遥测；
- COMMAND_LONG/INT：一次命令；
- NAMED_VALUE 仅作临时调试，不适合作为稳定控制协议。

## 1. 两种扩展位置

### 私有 dialect

适合独立设备和快速迭代，但所有端点都要生成同一 dialect。

### ardupilotmega/common 上游扩展

适合通用需求，需要 ID 分配、规范评审、兼容性和 MAVLink 上游流程。

不能直接在已生成头文件中手写结构体。

## 2. modules/mavlink 是子模块

当前仓库记录 MAVLink 仓库的一个 commit。贡献规则禁止把它当普通目录随意修改。

规范流程：

~~~text
fork MAVLink 仓库
    ↓
在 MAVLink 分支修改 XML
    ↓
生成和验证
    ↓
提交 MAVLink commit
    ↓
ArduSub 仓库更新 submodule pointer
    ↓
重新 Waf configure/build
~~~

若只是产品私有协议，也要用团队管理的 MAVLink fork 或独立 dialect，而不是只改某台电脑。

## 3. XML 设计

定义前写清：

- message name；
- 单一职责；
- 字段单位；
- frame；
- 时间戳；
- array 长度；
- 无效值；
- quality/status；
- extension fields；
- 最大频率；
- 来源和目标；
- 重复/乱序行为。

字段一旦发布，重排或改变类型会破坏 wire compatibility。

## 4. ID

消息 ID 必须在相应 dialect/生态允许范围内唯一。不能从 all.xml 看见一个空数字就占用，因为其他 dialect 和已发布组件可能冲突。

上游扩展应按 MAVLink 项目当前流程申请/评审。私有 ID 也要在组织内登记。

## 5. 枚举还是消息

枚举扩展用于给已有字段增加语义；新消息用于新的 payload 结构。

若只是新传感器类型，可能只需扩展某个枚举并复用已有消息。若数据字段完全不同，才考虑新消息。

## 6. extension fields

MAVLink 2 extension 可在保留旧基础字段的情况下追加信息。旧实现可以忽略尾部扩展。

但 extension 不能修复已经错误定义的坐标或单位，也不能保证 MAVLink 1 链路携带新字段。

## 7. 代码生成

当前 Waf：

~~~text
wscript _build_dynamic_sources
→ all.xml
→ Tools/ardupilotwaf/mavgen.py
→ build/板卡/libraries/GCS_MAVLink/include/mavlink/v2.0
~~~

修改 XML 后应看到 mavgen 任务重新运行，并在生成目录找到：

- message struct；
- MSG_ID；
- LEN/MIN_LEN；
- CRC；
- pack/encode/decode/send。

不要提交 build 生成目录。

## 8. Pixhawk 接收处理

若消息是所有车辆通用，处理通常进入 GCS_MAVLINK 公共层；若强依赖 Sub 模式，则进入 GCS_MAVLINK_Sub。

处理器至少：

1. decode；
2. 验证 source/target；
3. 验证 frame 和单位；
4. 检查有限值和范围；
5. 检查时间戳/序号；
6. 转给 Frontend 或车辆方法；
7. 更新健康和日志；
8. 对命令返回 ACK。

不要在 switch 中堆完整传感器算法。

## 9. Pixhawk 发送

新增状态消息需要：

- ap_message 或直接发送策略；
- try_send_message 分支；
- 默认/可配置间隔；
- payload space 检查；
- 链路拥塞行为；
- capability 或版本说明。

高频原始传感器不一定适合通过低带宽遥测持续发送。

## 10. 香橙派和 QGC

新增 XML 后，三端都要更新生成代码：

~~~text
传感器/香橙派发送端
Pixhawk dialect
QGC 或解析工具
~~~

仅 Pixhawk 能 decode 不会让旧 QGC 自动出现新面板。QGC 还需要 dialect 支持和 UI/Inspector 解析能力。

若 QGC 只做透明链路转发，它可能无需理解 payload；若要显示字段，则必须更新。

## 11. TELEM2 示例流程

### 设备直接发送 MAVLink

~~~text
设备 UART TX
→ Pixhawk TELEM2/USART3
→ SERIAL2_PROTOCOL = MAVLink2
→ frame parser
→ routing
→ handle_message
→ Sensor Frontend
→ Pixhawk 另一路 MAVLink
→ 香橙派/QGC
~~~

必须规划 system/component ID，避免把传感器伪装成主 autopilot。

### 设备使用私有串口协议

先由 UART Backend 解码，再由 Pixhawk 选择是否发送 MAVLink 状态。不能把非 MAVLink 字节交给 MAVLink parser 期望自动识别。

## 12. 兼容策略

- 发送端协商/检测能力；
- 接收端忽略未知消息；
- 新字段用 extension；
- 保持旧字段语义；
- 版本或 capability；
- 限制发送频率；
- 同时支持旧版时有明确退化；
- 日志记录当前协议版本。

## 13. 测试

- XML 校验和生成；
- C/C++ pack→decode 往返；
- Python/pymavlink 往返；
- CRC 错误；
- MAVLink1/2；
- extension 缺失；
- 大小端由生成器处理；
- 多 channel routing；
- 目标过滤；
- 带宽压力；
- 旧端忽略未知消息；
- Pixhawk1 和 SITL build。

## 14. 提交边界

通常分为：

1. MAVLink 定义仓库提交；
2. ArduSub 更新子模块与处理代码；
3. 伴随计算机库更新；
4. QGC 支持。

每个提交说明依赖的协议 commit，避免一半部署后无法通信。

## 15. 什么时候不要新增

- 已有 ODOMETRY 足够；
- 只是临时看一个 float；
- 字段定义尚未稳定；
- 只有一个内部函数使用；
- 没有时间戳/frame/单位；
- 无法维护 QGC 和伴随端；
- 仅为了绕过现有健康检查。
