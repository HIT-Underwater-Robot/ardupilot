# Sub_learning 动手实验：代码不会自动进入固件

本目录保存“参考实现”和“手工接入教程”。这些源码默认位于 `Sub_learning/`，根目录 Waf 不会编译它们。学习者必须在自己的实验分支中复制文件、修改注册点、重新构建并解释每一处改动。

这正是课程设计的一部分：如果只把现成能力编进固件，初学者很容易记住命令，却没有理解 ArduSub 的对象生命周期、调度器、模式分派、库依赖和 failsafe 边界。

## 实验地图

| 顺序 | 实验 | 新接触的架构边界 | 是否能请求推进器输出 |
|---|---|---|---|
| 0 | [建立安全实验分支和验证方法](00_environment_and_method/README.md) | Git、Waf、差异审查、烧录前检查 | 否 |
| 1 | [UART Ping：PC 与 Pixhawk1 串口收发](01_uart_ping/README.md) | `AP_SerialManager`、UART、初始化、scheduler | 否 |
| 2 | [带帧校验的串口命令库](02_framed_serial_library/README.md) | `libraries/`、Waf 白名单、状态机、CRC、超时 | 否 |
| 3 | [新增 Precision Manual 模式](03_precision_manual_mode/README.md) | mode 类、分派、GCS、RC 参数、控制器契约 | 是，仅拆桨台架 |
| 4 | [串口命令模式](04_serial_manual_mode/README.md) | 串口数据源 → 模式策略 → motors；失联锁存 | 是，仅高级拆桨实验 |

推荐严格按顺序完成。实验 4 依赖实验 2 和实验 3；前两个实验没有任何推进器控制代码，适合先验证工具链和串口接线。

## 每个实验都要回答的七个问题

在查看参考答案前，先在实验记录中写出：

1. 输入从哪里进入？使用什么坐标、单位和范围？
2. 谁拥有这个对象？构造、`init()` 和 `update()` 分别何时发生？
3. 哪一个 Waf 或源码注册点使代码真正进入固件？
4. 正常数据、坏帧、断线和重连时分别是什么状态？
5. disarmed、arming、spool 和现有 failsafe 是否仍然有效？
6. 怎样证明“编译成功”之外的行为正确？
7. 怎样恢复到实验前状态？

## 固定安全边界

- 所有改动只能进入新建的学生实验分支，不得直接提交到 `master`。
- 串口实验使用 Pixhawk1 TELEM2，并占用 `SERIAL2`。调试期间 GCS 应走 USB 或 TELEM1。
- USB 转 TTL 必须是 3.3 V 逻辑；只接 TX、RX、GND，TX/RX 交叉，禁止把适配器 5 V/VCC 接入飞控。
- 实验 1、2 不得调用 `AP_Motors`。
- 实验 3、4 的任何 armed 测试必须拆桨或物理隔离推进器，并准备独立断电。
- 实验 4 的自定义串口不能替代 GCS/RC failsafe；保持正常 GCS 心跳和驾驶输入链路。
- 任何功能在进入 `master` 前都要重新设计、审查并完成硬件验证；这里的代码是教学参考，不是产品功能。

## 目录约定

每个实验中的 `files/` 模拟目标仓库路径。例如：

```text
Sub_learning/labs/02_framed_serial_library/
└── files/
    └── libraries/
        └── AP_LearningSerial/
            ├── AP_LearningSerial.cpp
            └── AP_LearningSerial.h
```

学习时把 `files/libraries/AP_LearningSerial/` 复制到仓库根目录的 `libraries/`，但不要把整个实验说明目录复制进固件源码。

## 完成标准

一个实验只有同时满足以下条件才算完成：

- 能画出本实验的数据流和对象生命周期。
- 能逐项解释自己修改的文件，而不是只展示最终 diff。
- `git diff --check` 无空白错误。
- Pixhawk1 在全新输出目录中构建成功。
- 完成实验规定的串口或拆桨台架观察。
- 记录真实结果和未验证项，不把构建成功写成实机功能通过。
