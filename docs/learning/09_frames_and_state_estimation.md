# 第 9 章：坐标系、单位与状态估计

控制系统最危险的错误往往不是 C++ 语法，而是“数字正确，含义错误”：

- 把 Down 当 Up；
- 把米当厘米；
- 把 body velocity 当 NED velocity；
- 把测量时间当接收时间；
- 把目标位置当当前位置。

## 1. 文件地图

1. **libraries/AP_Math/**：向量、四元数和旋转；
2. **libraries/AP_InertialSensor/**：IMU Frontend；
3. **libraries/AP_AHRS/**：统一姿态/位置接口；
4. **libraries/AP_NavEKF3/**：EKF3；
5. **libraries/AP_NavEKF/AP_NavEKF_Source.***：观测源选择；
6. **libraries/AP_VisualOdom/**：外部里程计入口；
7. **libraries/GCS_MAVLink/GCS_Common.cpp**：ODOMETRY 解码；
8. **ArduSub/inertia.cpp**：车辆取得估计状态。

## 2. 四种常见坐标

### 2.1 Body FRD

随车辆旋转：

- X：Forward；
- Y：Right；
- Z：Down。

DVL 给出的速度常以设备或机体系表达，但必须以该设备协议为准，还要补传感器安装方向。

### 2.2 Local NED

局部固定坐标：

- X：North；
- Y：East；
- Z：Down。

EKF 和 MAVLink 导航消息常用 NED。

### 2.3 NEU

ArduSub 某些位置控制接口使用：

- X：North；
- Y：East；
- Z：Up。

因此 SET_POSITION_TARGET_LOCAL_NED 进入 Guided 时可看到 z 和 vz 取负。

### 2.4 地理坐标

Location 使用 latitude、longitude 和 altitude frame。它不是简单 Vector3f，还包含高度参考系。

无 GPS 场景仍可能需要一个 EKF origin，把局部坐标与 Location 接口连接起来。

## 3. 单位后缀是接口的一部分

常见约定：

| 后缀/上下文 | 单位 |
|---|---|
| _m | 米 |
| _cm | 厘米 |
| _mps | 米每秒 |
| _cms | 厘米每秒 |
| _rad | 弧度 |
| _deg | 度 |
| _cd / _cds | 百分之一度 / 每秒 |
| _ms / _us | 毫秒 / 微秒 |

不能只看 Vector3f 类型判断单位。类型系统不会阻止把米向量传给厘米接口。

## 4. INS、AHRS、EKF 的分工

### INS

管理陀螺仪和加速度计，提供校准、滤波、实例选择、采样时刻和 delta angle/velocity。

### EKF

根据运动模型与 IMU 高频预测，再用水压、外部导航、GPS、罗盘等观测校正，估计姿态、速度、位置和偏置。

### AHRS

向车辆层提供统一的姿态和导航状态接口，并管理选中的估计后端。车辆代码通常通过 AP::ahrs() 取状态，而不是直接读取 EKF3 内部数组。

~~~text
传感器测量
    ↓
INS / Baro / VisualOdom Frontend
    ↓
EKF 后端
    ↓
AP_AHRS 统一状态
    ↓
Sub::read_AHRS / read_inertia
    ↓
控制器
~~~

## 5. 原始测量、估计状态和控制目标

三者绝不能混在一个变量概念里。

| 层次 | 示例 |
|---|---|
| measurement | DVL 本帧速度、水压压力值 |
| estimate | EKF 当前 NED 速度和位置 |
| target | Guided 希望达到的位置和速度 |

控制器比较 estimate 与 target 产生误差。measurement 通常先经过时间对齐和融合，不直接等于控制反馈。

## 6. ODOMETRY 当前接受的坐标要求

GCS_MAVLINK::handle_odometry 当前明确要求：

~~~text
frame_id       = MAV_FRAME_LOCAL_FRD
child_frame_id = MAV_FRAME_BODY_FRD
~~~

不满足就直接返回。

消息提供：

- time_usec；
- position x/y/z；
- quaternion q；
- linear velocity vx/vy/vz；
- angular velocity；
- pose_covariance；
- velocity_covariance；
- reset_counter；
- estimator_type；
- quality。

当前 ArduPilot 处理代码使用其中姿态、位置、线速度、pose covariance、reset counter 和 quality 等信息；不要因为 XML 有字段就假定当前处理函数融合每个字段。

## 7. ODOMETRY 的速度转换

当前实现把消息 velocity 解释在 BODY_FRD child frame，再用四元数旋转到 NED：

~~~text
vel_body_frd
    ↓ quaternion
vel_ned
    ↓
handle_vision_speed_estimate
~~~

如果香橙派已经发送 NED 速度却仍标 BODY_FRD，Pixhawk 会再次旋转，结果错误。

## 8. covariance 如何使用

代码从 pose_covariance 的对角元素组合位置误差和角度误差，再交给 AP_VisualOdom。

协方差表达“不确定性”，不是越小越高级。填得过小会让 EKF 过度相信外部数据，跳变或漂移可能直接传到控制反馈；填得过大则可能几乎不起作用。

若 covariance 首项是 NaN，当前处理采用默认/后续最小噪声约束路径。发送端应按当前 MAVLink 与 ArduPilot 实现验证，不要随意用全零数组代表未知。

## 9. 时间戳为什么重要

数据经过：

~~~text
传感器采样
→ 香橙派读取
→ 融合
→ 串口/网络
→ Pixhawk 接收
~~~

到达 Pixhawk 时已经延迟。EKF 需要尽可能按测量发生时刻对齐历史状态。

ArduPilot 会校正外部时间戳，并结合 VISO_DELAY_MS 等延迟设置。错误时间戳会造成相位滞后，闭环中可能表现为振荡或跟踪变差。

## 10. reset_counter

外部估计器可能重定位、回环校正或重启，位置发生非连续跳变。reset_counter 变化用于告诉接收端这不是普通运动。

不要每帧递增；应仅在估计坐标发生重置时改变，并保持 uint8 环绕语义可处理。

## 11. quality 与健康

AP_VisualOdom 保存质量，并与 VISO_QUAL_MIN 比较。质量不达标时可记录数据但不交给 EKF消费。

healthy 还关注最近是否持续更新。因此：

- quality 高但断流仍会不健康；
- 持续收帧但 quality 低也不应被融合；
- 收到 MAVLink 帧不等于观测被 EKF 接受。

## 12. AP_VisualOdom 到 EKF

~~~text
GCS handle_odometry
    ↓
AP_VisualOdom::handle_pose_estimate
    ↓
AP_VisualOdom_MAV
    ├─ 比例和安装修正
    ├─ 噪声下限
    ├─ quality 检查
    └─ AP::ahrs().writeExtNavData
             ↓
          EKF2/EKF3

velocity
    ↓
AP::ahrs().writeExtNavVelData
    ↓
EKF
~~~

只有配置了 VisualOdom MAV backend，并把 EKF source 选择为 ExternalNav 的相应轴，数据才可能成为主估计观测。

## 13. EKF source 是按分量选择的

EK3_SRCx 参数分别选择：

- POSXY；
- VELXY；
- POSZ；
- VELZ；
- YAW。

ExternalNav 的枚举值在当前源码中为 6，但配置时应以当前固件参数元数据为准。

这允许你的方案采用：

- 香橙派融合的 XY 位置和速度；
- MS5837/Baro 的 Z 位置；
- 外部导航或其他来源的 yaw。

不要因为发送 ODOMETRY 同时带 z 和 yaw，就默认 EKF 必须融合所有分量。

## 14. 适合当前项目的大小脑边界

~~~text
香橙派“大脑”
    ├─ DVL/USBL/视觉采集
    ├─ 时间同步
    ├─ 外部多传感器融合
    ├─ 路径规划
    ├─ 输出 ODOMETRY
    └─ 输出 Guided setpoint

Pixhawk“小脑”
    ├─ 再检查消息和健康
    ├─ EKF 接收 ExternalNav
    ├─ 水压深度反馈
    ├─ 姿态/位置闭环
    ├─ 混控和电机输出
    └─ 解锁与 failsafe
~~~

外部融合不代表 Pixhawk 与估计无关。Pixhawk 仍要把外部状态放入统一估计接口，控制器才能得到位置和速度反馈。

## 15. 避免重复融合

若香橙派已经把 DVL 与 USBL 融合成 ODOMETRY，同时又把原始 DVL 通过另一 Pixhawk Backend 融合，两个输入可能高度相关却被 EKF 当作独立观测。

设计时明确：

- 哪一端负责融合；
- Pixhawk 接收融合结果还是原始测量；
- 两条接入路径是否互斥；
- 切换时 source set 和 reset_counter 如何处理。

兼容两种接法不等于同时启用两种接法。

## 16. Sub 怎样读取最终状态

Sub::read_inertia() 调用 pos_control.update_estimates()，再从 AHRS 取得位置，并在存在 VERT_POS 状态时更新高度。

车辆模式读取的是统一估计，而不是直接读取最近一帧 ODOMETRY。

## 17. 上机前验证顺序

1. 静止时检查 body FRD 符号；
2. 单轴移动验证 N/E/D；
3. 单独旋转 yaw，确认 body velocity 转换；
4. 检查时间戳单调和延迟；
5. 检查 reset_counter；
6. 从保守 covariance 开始；
7. 查看 VisualOdom 日志；
8. 查看 EKF innovation 和 source；
9. 未装推进器先验证；
10. 系留低推力水池测试。

## 18. 验收问题

1. Body FRD、Local NED 和 NEU 的 Z 轴分别怎样？
2. 为什么 Vector3f 不能表达单位安全？
3. INS、EKF、AHRS 各做什么？
4. ODOMETRY 为什么不是控制目标？
5. 当前 ODOMETRY 接受哪些 frame？
6. body velocity 怎样转成 NED？
7. covariance 太小有什么风险？
8. reset_counter 何时变化？
9. 发送了数据后还需要哪些配置才能融合？
10. 为什么两条兼容输入路径通常不能同时融合？
