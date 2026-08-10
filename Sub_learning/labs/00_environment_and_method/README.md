# 实验 00：建立可回退的学习环境

## 学习目标

本实验不修改控制代码。完成后应能区分：

- `master`：完整、审慎准入的维护基线。
- `learning/minimal-pixhawk1`：极简教学基线。
- `student/...`：每名学习者自己的可丢弃实验分支。
- “源码被复制到仓库”与“源码被 Waf 编译、被对象持有、被 scheduler 调用”是四件不同的事。

## 第一步：建立学生分支

先确认当前仓库没有未保存的用户工作：

```bash
git status --short --branch
git branch --show-current
```

从极简教学基线创建自己的分支，名称中写明实验：

```bash
git switch -c student/01-uart-ping learning/minimal-pixhawk1
```

不要在 `master` 上练习。开始前记录基线：

```bash
git rev-parse HEAD
git status --short
```

## 第二步：理解默认构建

极简分支只支持 Pixhawk1。默认验证命令为：

```bash
./waf configure --board Pixhawk1 \
    --out build_student_baseline \
    --no-submodule-update
./waf sub -j4
```

禁止使用 `sudo ./waf`。输出目录是生成物，不是源码。每次实验最好使用新的 `--out` 名称，这样可以区分“增量构建恰好没重编”与“新源码确实从头进入依赖图”。

## 第三步：每次只改变一个架构边界

推荐循环：

```text
阅读真实调用点
    ↓
复制一个实验文件
    ↓
只修改对应注册点
    ↓
检查 git diff
    ↓
全新目录构建
    ↓
串口/拆桨台架观察
    ↓
记录结果并提交学生分支
```

每一步都运行：

```bash
git status --short
git diff --check
git diff -- ArduSub libraries
```

## 第四步：学会辨认四类“接入”

| 接入类型 | 本课程中的例子 | 忘记后的典型现象 |
|---|---|---|
| 源文件接入 | 把 `.cpp/.h` 复制到 `ArduSub/` 或 `libraries/` | include 找不到，或源码根本不存在 |
| 构建接入 | 在 `ArduSub/wscript` 白名单加入新库 | 头文件可能找到，但链接缺少实现 |
| 对象接入 | 在 `Sub.h` 增加成员 | 没有实例，不能保存状态或被调度 |
| 运行接入 | 在 `init_ardupilot()` 和 scheduler 注册 | 能编译，但硬件没有初始化、`update()` 从不执行 |
| 策略接入 | 模式枚举、分派、GCS 列表和 failsafe 契约 | 模式不可选，或控制器/failsafe 行为错误 |

`ArduSub/` 顶层 `.cpp` 会由车辆静态库规则自动发现，因此新增模式通常不需要把文件名逐个写进 `wscript`。但新增一个 `libraries/AP_*` 库时，本极简分支必须显式加入 `ArduSub/wscript` 的 `ap_libraries` 白名单。

## 第五步：恢复方式

最安全的回退是让每个实验独占一个小提交。实验失败时先保存证据：

```bash
git status --short
git diff > /tmp/student-lab.diff
```

确认没有需要保留的用户内容后，删除这个学生分支并从教学基线重新开始。不要在共享工作区使用 `git reset --hard`、`git clean` 或批量覆盖命令。

## 交付记录模板

```text
实验：
基线提交：
修改文件：
构建命令：
构建结果：
硬件连接：
观察结果：
未验证项：
安全措施：
回退方法：
```
