# WSL 构建、测试与发布

## 1. 固定构建环境

本仓库默认在 WSL/Ubuntu 中构建，所有命令从仓库根目录执行。首次准备环境：

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

禁止使用 `sudo ./waf`。如果工具链不可见，先修复当前用户的 PATH 和依赖，不要以 root 构建来掩盖权限问题。

## 2. 子模块

克隆时初始化子模块：

```bash
git clone --recurse-submodules https://github.com/HIT-Underwater-Robot/ardupilot.git
cd ardupilot
```

已有工作树缺少必要子模块时：

```bash
git submodule update --init --recursive
```

`modules/` 中的内容属于各自上游仓库。不要在主仓库任务中直接修改、提交或“清理”子模块；也不要只为了让状态看起来整齐而擅自更新 submodule SHA。

## 3. Waf 的职责

- 根目录 `waf` 是启动器。
- `modules/waf` 提供通用 Waf 引擎。
- 根 `wscript` 处理板卡、工具链、全局选项和构建配置。
- `Tools/ardupilotwaf` 提供车辆、库、板卡和固件规则。
- [ArduSub/wscript](../ArduSub/wscript) 声明 `ardusub` 程序、车辆静态库及直接依赖。

Waf 按任务依赖图和输入签名做增量构建，输出位于 `build/<board>/`。构建目录是可再生输出，不是源码；不要从中提取所谓“精简源码”，也不要把它加入提交。

## 4. SITL 基线构建

```bash
./waf configure --board sitl
./waf sub -j"$(nproc)"
```

输出：

```text
build/sitl/bin/ardusub
```

SITL 是控制逻辑和集成路径的第一层验证，不证明 Pixhawk 工具链、flash 容量、定时器输出或真实水动力。

## 5. Pixhawk4 基线构建

```bash
./waf configure --board Pixhawk4
./waf sub -j"$(nproc)"
```

输出：

```text
build/Pixhawk4/bin/ardusub.apj
```

Pixhawk 2.4.8 对应 Waf 板卡名通常为 `Pixhawk1`。任何实际目标板名称都应先用 `./waf list_boards` 和对应 `hwdef` 核验，不能根据商品名称猜测。

## 6. ArduSub autotest

完整 ArduSub 测试入口：

```bash
Tools/autotest/autotest.py build.Sub test.Sub
```

指定测试示例可按当前脚本帮助信息使用：

```bash
Tools/autotest/autotest.py build.Sub test.Sub.DiveManual
```

测试实现和场景证据位于 [Tools/autotest/ardusub.py](../Tools/autotest/ardusub.py) 及 `Tools/autotest/ArduSub_Tests/`。只运行构建不能声称控制行为已经通过回归。

## 7. 默认验证矩阵

| 变更类型 | 最低验证 |
|---|---|
| 仅 Markdown 文档 | Markdown/链接检查、`git diff --check` |
| ArduSub 非控制逻辑 | SITL build、相关单元/集成测试、Pixhawk4 build |
| 模式、位置或姿态控制 | SITL build、针对性 ArduSub autotest、日志审查、Pixhawk4 build |
| AP_Motors、spool、混控、failsafe | 上述全部，加拆桨/隔离台架测试和回退固件 |
| `hwdef` 或新板卡 | bootloader build、ArduSub board build、接口电气测试、逐级上电验证 |
| 参数变更 | 构建/测试，加参数索引、默认值、升级和参数存储兼容性检查 |

## 8. 如何记录真实结果

每次交付至少记录：

- 精确分支和 HEAD SHA。
- 构建环境、板卡名和完整命令。
- 命令退出状态和关键输出，不写“应该通过”。
- 固件路径、flash/RAM 或尺寸变化。
- 执行了哪些测试、哪些没有执行。
- 实机条件：是否拆桨、使用何种负载、failsafe 如何触发。
- 已知风险和一条可操作的回退路径。

本聚焦基线在建立时已真实通过 SITL 与 Pixhawk4 的 `waf sub` 构建；这只是当时提交的证据，后续变更仍需重新验证，不能永久继承“通过”结论。

## 9. 提交与发布

提交标题使用：

```text
Subsystem: short description
```

例如 `ArduSub: ...`、`AP_Motors: ...`、`Tools: ...`、`Docs: ...`。一个提交只包含一个逻辑变更，并在提交或 PR 中说明 AI 协助。未经明确要求，不提交、不推送、不创建 PR；发布时禁止强制推送默认维护基线。
