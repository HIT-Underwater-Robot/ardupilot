# 安全变更工作流

## 1. 开始前先冻结事实

每个任务开始时先执行只读检查：

```bash
git status --short --branch
git rev-parse HEAD
git submodule status
```

区分本任务变更、用户已有变更、未跟踪资料和构建产物。发现 dirty worktree 时先确认归属；不得覆盖、移动或清理不属于当前任务的内容。

## 2. 把需求写成可验证的问题

控制或硬件需求至少包含：

- 当前行为和期望行为。
- 复现模式、参数、输入、传感器状态和 failsafe 状态。
- 机体系/地固坐标系、正方向和单位。
- 目标板、输出协议、推进器布局和测试安全条件。
- 可接受的回退行为和失败判据。

“让启动更平滑”不是充分需求；应明确是哪个模式、哪一种目标、最大速度/加速度/jerk、允许响应时间以及如何从日志判定。

## 3. 先画真实数据链，再改代码

通用审计链：

```text
input/message/sensor
 -> mode selection and init
 -> target generation
 -> coordinate/unit conversion
 -> attitude/position controller
 -> AP_Motors desired/actual state
 -> 6DOF mixer and limits
 -> SRV/HAL output
 -> log/test evidence
```

只修改链中的最小必要节点。不要为了一个模式问题重构整个 `AC_*` 控制体系，也不要把板级 GPIO 特例写进 ArduSub 模式代码。

## 4. 安全关键代码规则

### 参数

- 不改变已有 `AP_GROUPINFO` 索引。
- 新参数使用未占用索引，提供完整 `@Param` 文档、范围、单位和用户级别。
- 检查参数全名长度、默认值、持久化兼容和固件升级行为。

### Feature guard

- 保留现有 `#if AP_<FEATURE>_ENABLED`。
- 核心组件不能依赖可选组件。
- 关闭可选特性时仍应可编译；新增体积明显时评估 build option。

### HAL 与板卡

- 共享库不直接依赖某块板的 GPIO/定时器实现。
- 板级定义优先进入 `AP_HAL_ChibiOS/hwdef/<board>/`。
- 不把 Linux 伴随计算机路径扩展成本仓库的实时飞控产品目标。

### 子模块和依赖

- 不直接修改 `modules/`。
- 不凭目录名判断“无用库”。
- 删除依赖前必须证明 Waf 依赖闭包、目标板构建、SITL/测试和固件尺寸变化。

## 5. 小步提交策略

一个安全变更通常按以下边界拆分：

1. 测试或复现证据。
2. 最小行为修复。
3. 必需的参数/文档更新。
4. 板卡定义或工具调整（仅在确实独立时）。

避免同时进行大规模格式化、文件移动和控制逻辑修改。这样 review、`git bisect` 和回退才能定位到单一原因。

## 6. 变更后的检查顺序

```bash
git diff --check
git diff --stat
git diff --name-status
```

随后按风险执行：

1. 相关静态/格式检查。
2. 相关单元测试。
3. SITL build。
4. 针对性 ArduSub autotest 和日志检查。
5. Pixhawk4 或目标板 build。
6. 拆桨/隔离台架测试。
7. 受控水池/实艇回归。

任何未执行项都要明确写为“未执行”，不能用推测替代结果。

## 7. 推进器相关评审问题

代码合入前逐项回答：

- 未解锁时是否仍强制零能量输出？
- interlock 或 safety 失效时是否立即进入安全状态？
- 输入超时、pilot failsafe、EKF/位置失效时目标怎样变化？
- spool desired state 和 actual state 的转换是否仍单调、限速、可中止？
- 六轴混控是否可能新增饱和或符号反转？
- 方向参数和 SRV 功能是否保持向后兼容？
- 是否有一条不依赖通信的物理断电路径？

## 8. 回退方案不是一句“恢复旧版本”

交付时应准备：

- 已验证的前一固件及 SHA。
- 参数备份和参数迁移差异。
- bootloader/SWD 恢复方式（若涉及板卡）。
- 回退触发条件和负责人。
- 回退后需要重新确认的通道方向、failsafe 和传感器状态。

不得用 `git reset --hard`、`git checkout --`、`git clean` 或强制推送处理普通开发回退。这些命令可能破坏用户内容和维护历史。

## 9. 稳定版升级流程

本仓库固定在 ArduSub 4.7.0 产品基线，不自动升级上游 `master`。确需升级时：

1. 建立独立升级分支。
2. 确定目标官方稳定标签和完整 commit range。
3. 审计 ArduSub、共享控制器、AP_Motors、MAVLink、ChibiOS 和参数差异。
4. 重放本仓库聚焦修改并解决冲突。
5. 执行 SITL、ArduSub autotest、Pixhawk4/目标板和实机回归。
6. 准备固件、参数和 bootloader 回退方案。
7. 人工评审后再切换维护基线。
