# 第 20 章：安全裁剪与长期维护

一个“看起来文件少”的仓库，不一定比可验证的仓库更简单。真正有价值的最简工程应满足：

- 只呈现 ArduSub 主线；
- 仍能从干净环境构建；
- 子模块和工具完整；
- 与上游差异可解释；
- 可升级；
- 功能开关可回归；
- 不通过删除安全层换取简短。

## 1. 三种“用到”

判断库是否可删前区分：

### 源码依赖

某文件 include 或引用它。

### 编译依赖

Waf 把它编译成对象或静态库。

### 最终链接

链接器把某些 section 放进 ardusub ELF。

静态库中某驱动被编译，不一定全部进入最终 Flash；链接器和 section garbage collection 可能丢弃未引用部分。

所以“编译输出出现文件名”也不等于全部代码占用固件。

## 2. 先开关，后删除

推荐顺序：

~~~text
记录基准
→ 使用编译开关禁用
→ 编译 enabled/disabled
→ 比较 ELF/Map/尺寸
→ SITL/目标板回归
→ 确认无运行需求
→ 最后评估是否物理删除源码
~~~

直接删除文件会同时破坏上游同步、其他板卡条件分支和代码生成，定位成本更高。

## 3. 哪些层不做物理裁剪

本项目约定一级目录整体保留的关键部分包括：

- ArduSub；
- 被 ArduSub 使用的 libraries 一级库及其完整内容；
- modules 中所需子模块；
- ChibiOS 和 hwdef；
- Waf 与 Tools/ardupilotwaf；
- 必要 autotest；
- 参数和代码生成工具。

库内部暂时保留其他传感器 Backend，有助于上游同步和教学。

## 4. 可以重点裁剪的外围

在确认无用途后，通常优先考虑：

- 其他 vehicle 顶层源码；
- 与本项目无关的文档站点内容；
- CI 矩阵中的其他车辆；
- Docker/云端包装；
- 不使用的开发环境脚本；
- 与团队流程无关的发布工具。

但删除前要检查 Waf configure、测试脚本和子模块初始化是否引用路径。

## 5. Tools 不等于运行时固件

Tools 中多数文件不进入 Pixhawk Flash，但可能是构建、生成、测试和维护所需。

保留原则：

- ardupilotwaf：构建必需；
- autotest：SITL 回归；
- scripts 中实际依赖脚本；
- bootloaders/IO firmware：板卡构建；
- ros2/DDS、MATLAB：团队明确需要的集成；
- CodeStyle：代码检查。

“不进固件”不能直接推出“仓库中无用”。

## 6. modules 不按文件删

modules 是独立上游仓库，由 Git 子模块 commit 管理。例如：

- ChibiOS；
- mavlink；
- gtest 等。

不要删除子模块内部看似无关目录来做私有最简版。应保留子模块完整 commit，或在主仓库明确不再依赖整个子模块。

## 7. 依赖审计证据

每次裁剪记录：

- 删除/关闭对象；
- Waf 依赖来源；
- include 搜索；
- compile_commands；
- ELF symbols；
- Linker.map；
- Flash/RAM before/after；
- SITL；
- Pixhawk1 build；
- 功能测试；
- 恢复 commit。

没有证据就只标记“候选”，不删除。

## 8. 稳定版与教学版

推荐两条分支，而不是两个互相复制的目录：

~~~text
main/stable
    官方为主、只去除明确外围
    保持可编译 ArduSub

teaching
    中文课程、导航注释、实验
    定期 rebase/merge stable
~~~

二次开发再从 stable 或 teaching 建独立 feature branch。

分支比复制目录更容易比较、回滚和合并上游。

## 9. 上游同步

维护记录：

- upstream remote；
- 当前 upstream commit；
- 自己的删减 commit；
- 中文文档 commit；
- 功能 commit；
- 子模块 commit。

同步顺序：

1. fetch upstream；
2. 阅读 ArduSub release notes；
3. rebase/merge 到临时分支；
4. 解决车辆和构建冲突；
5. 初始化正确子模块；
6. 全部回归；
7. 再更新 stable。

大规模删除应集中在少数清晰 commit，避免每次同步无法分辨业务变化。

## 10. 提交拆分

建议：

1. Repository: remove unrelated vehicle entry points；
2. Docs: add Chinese learning map；
3. ArduSub: add navigation comments；
4. AP_xxx: add specific feature；
5. Tests: cover feature。

不要把删除几千文件、改控制器和更新参数索引混在同一提交。

## 11. 构建矩阵

最低：

| 目标 | 用途 |
|---|---|
| SITL Sub | 逻辑和集成 |
| Pixhawk1 Sub | 当前硬件和容量 |
| 未来 Pixhawk6 目标 | 升级兼容 |
| feature disabled | 可选依赖边界 |
| feature enabled | 新功能 |

若修改 MAVLink，再加伴随端生成/解析测试；若修改板卡，再加 Bootloader。

## 12. 固件尺寸

每次记录：

- Text；
- Data；
- BSS；
- Total Flash；
- Free Flash；
- APJ 文件大小。

BSS 占 RAM 不占同等 Flash；APJ 可能压缩/封装。不要只看一个数字。

尺寸增长应能对应到功能收益和 Map 证据。

## 13. 文档维护

每个功能文档包含：

- 目的；
- 数据流；
- 参数；
- 接口；
- 坐标和单位；
- 失败行为；
- 测试；
- 已知限制；
- 对应 commit/version。

中文导航用于学习，参数元数据和对外协议仍要保持工具能识别的格式。

## 14. 不修改逻辑的注释验证

对注释批次可做：

- Python AST 比较；
- C/C++ 去注释 token 比较；
- hwdef 去注释数据比较；
- git diff --check；
- 实际编译。

这些不能证明原代码本身正确，但能证明注释工作没有偷偷改变可执行 token。

## 15. 删除判定表

只有同时满足才考虑物理删除：

- 不在 Waf/source 依赖；
- 不在 codegen/configure；
- 不在目标板；
- 不在 SITL/test；
- 不在团队工具；
- 不在未来明确路线；
- 删除后干净 clone 可构建；
- 有尺寸或维护收益；
- 有恢复 commit；
- 上游同步成本可接受。

## 16. 发布前清单

- 干净 clone；
- submodule 完整；
- README 从零可执行；
- 虚拟环境/编译器版本；
- SITL build；
- Pixhawk1 build；
- 固件 hash；
- 参数备份；
- Release notes；
- 已知限制；
- 测试证据；
- AI 辅助贡献披露；
- 人工理解和审批。

## 17. 最终心智模型

~~~text
Waf/hwdef/modules
    生成并构建平台
        ↓
HAL + AP_Vehicle + Scheduler
    提供运行骨架
        ↓
MAVLink / Sensor Frontends
    提供输入
        ↓
AHRS/EKF
    提供状态
        ↓
Mode/Mission
    提供目标
        ↓
Pos/Attitude controllers
    提供六轴控制量
        ↓
Motors/SRV/RCOutput
    驱动推进器
        ↑
Arming/Failsafe/Logger
    贯穿全链安全与证据
~~~

能沿这张图从任意一层向上找输入、向下找输出，就已经具备独立继续学习和二次开发 ArduSub 的框架能力。

## 18. 课程完成后的实际顺序

1. 用现有 Pixhawk1 + MS5837 跑通基准；
2. 香橙派发送 ODOMETRY；
3. Guided 发送局部目标；
4. 建立日志和失联测试；
5. 再开发指定型号 DVL/USBL UART Backend；
6. 新控制模式；
7. 确有必要才扩展 MAVLink；
8. 最后迁移新板。

这个顺序让每一阶段都有可工作的上一个版本可对照。
