# Quadruped Project Code Understanding Task

这份文档不是用于继续某一个单点开发任务，而是用于交接一个新的长期任务：

**带用户从整体框架到各部分实现细节，系统理解当前四足机器人项目仓库，最终达到“用户能够独立完成 NMPC 相关修改与开发，同时具备继续维护整个项目的能力”的目标。**

这项任务覆盖范围**不只 NMPC**，还包括：

- 控制器整体架构
- FSM 状态机
- 步态与 gait 相关代码
- 估计器、平衡控制、机器人模型
- 输入链路
- 仿真/硬件接线
- 配置文件组织
- 其他 controller 的参考价值

但要始终记住：

**这些内容之所以要系统梳理，不是为了泛泛地“看懂仓库”，而是为了支撑用户最终能够独立进行 `unitree_guide_controller` 中的 NMPC 修改、定位问题、扩展能力和继续开发。**

截至 `2026-05-12 13:52:02 CST`，这个任务的首要目标已经进一步收敛为：

**先暂停继续做新修改，优先带用户彻底理解当前已经实现出来的 NMPC 主链、执行层结构与代码分工，直到用户能够独立解释、定位、修改并继续实现 MPC/NMPC 相关功能。**

因此，这份文档现在应被视为一份：

- 学习路线图
- 讲解任务清单
- 理解检查清单
- 进度交接文档

---

## 1. 任务目标

新的 AI 接手后，应围绕下面这个主目标推进：

**通过“提示 + 提问 + 逐步确认理解 + 持续写入进度文档”的方式，帮助用户系统掌握 `quadruped_ros2_control` 仓库，直到用户能独立阅读、定位、解释并继续维护当前项目，尤其能独立完成 NMPC 相关开发。**

这个任务的关键不是“替用户总结一遍代码”，而是：

1. 带用户建立代码地图
2. 一层一层拆解关键模块
3. 通过提问确认用户是否真的理解
4. 把当前理解进度持续记录下来
5. 最终让用户自己具备独立读代码和独立做 NMPC 修改的能力

补充要求：

- 讲解必须遵循：
  - 先大框架
  - 再具体代码
  - 再通过代码提问确认理解
  - 再进入下一个模块
- 不能只给最终结论，必须把代码入口、数据流和模块边界讲清楚
- 最终目标不是让用户“记住答案”，而是让用户能独立再次推导出答案

### 1.1 最终能力目标

新的 AI 在推进这个任务时，必须始终围绕下面这组最终能力目标来判断“用户是否真的学会了”：

1. 用户能独立描述当前 `unitree_guide_controller` 的控制主链
2. 用户能独立定位 NMPC 相关代码入口与数据流
3. 用户能独立修改：
   - `StateNMPC`
   - `CtrlComponent`
   - `nmpc::LeggedInterface`
   - `go1_description/config/ocs2/*.info`
4. 用户能独立根据日志判断问题更可能出在：
   - reference
   - observation
   - cost / constraints
   - execution
5. 用户能独立完成一类最小 NMPC 开发任务，例如：
   - 增加日志与验证点
   - 调整 `Q / R`
   - 修改 reference 更新逻辑
   - 调整 policy 到执行层的映射

### 1.2 当前必须优先理解的 NMPC 主链

在后续讲解中，新的 AI 必须优先确保用户真正掌握当前已经跑通的这一条主链：

1. `StateNMPC::run()`
2. `CtrlComponent::pushCurrentObservationToMpc()`
3. 后台线程 `advanceMpc()`
4. 前台 `CtrlComponent::evaluateCurrentPolicy()`
5. `planned_mode -> solver_contact -> leg_roles`
6. 执行层分工：
   - `stance`: `support torque + NMPC stance hold`
   - `swing`: `reference foot target -> IK -> joint tracking`

如果用户还不能自己顺着这 6 步讲清楚，说明还不能进入更深的 cost、constraint 和 OCS2 配置层。

### 1.3 本任务的范围原则

本任务虽然覆盖 gait、FSM、输入链路、估计器等模块，但新的 AI 必须始终把它们作为：

- NMPC 开发所需上下文
- 理解控制器整体行为所需背景

来组织讲解。

不要把任务带偏成：

- 纯粹的仓库漫游
- 或平均分配精力给所有 controller

主线应该始终是：

**理解整个项目，是为了最终能独立改 NMPC。**

### 1.4 当前已知的理解基线

截至当前，已经不是“从零认识 NMPC”阶段，而是“基于已有实装结果做反向理解”阶段。

当前已经确认并应纳入讲解前提的事实有：

- MPC 主链已经打通：
  - `setCurrentObservation()`
  - `advanceMpc()`
  - `updatePolicy()`
  - `evaluatePolicy()`
- `solver mode` 已经是执行相位真源
- `UnifiedGaitScheduler` 当前不再是 OCS2 mode 输入真源，只保留：
  - 足端 reference
  - 相位/节律参考
- runtime recovery 主链已经停用
- 当前主要问题已经从“主链是否能跑”转为“执行层质量是否足够好”

这意味着：

- 讲解时不能再把重点放在“如何把 OCS2 接进来”
- 应优先放在“当前实现到底怎么工作、哪里已经稳定、哪里还只是临时执行层方案”

---

## 2. 当前仓库理解任务的覆盖范围

当前主要工作目录：

- 仓库根：
  - `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control`
- 当前维护重点 controller：
  - `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller`

理解任务应覆盖的主要模块包括：

### 2.1 commands

- `commands/control_input_msgs`
- `commands/keyboard_input`
- `commands/joystick_input`
- `commands/unitree_joystick_input`

用户需要最终搞清楚：

- 控制输入消息是怎么定义的
- 键盘/手柄输入如何进入系统
- 输入是如何传到 controller 的
- `lx / ly / rx / command` 这些字段最终影响哪些行为

### 2.2 controllers

至少需要让用户形成对下面几个 controller 的基本认识：

- `controllers/unitree_guide_controller`
- `controllers/leg_pd_controller`
- `controllers/ocs2_quadruped_controller`
- `controllers/rl_quadruped_controller`

其中重点最深的是：

- `unitree_guide_controller`

但新的 AI 不要只讲它一个包，也应帮助用户建立：

- 当前维护 controller
- 其他 controller
- 可复用思路
- 架构差异

之间的关系图。

这里要特别强调：

- `ocs2_quadruped_controller` 的意义主要是帮助用户建立 NMPC 对照参考
- `leg_pd_controller` 的意义主要是帮助用户理解最简单的执行层
- `rl_quadruped_controller` 的意义主要是帮助用户看到另一类控制路线

新的 AI 讲这些包时，重点不是“平均展开”，而是回答：

- 这些包如何帮助用户更好理解和修改当前 `unitree_guide_controller` 的 NMPC 路线？

### 2.3 descriptions

需要让用户理解：

- 各机器人 description 包的角色
- `go1_description` 里的：
  - `gazebo.yaml`
  - `robot_control.yaml`
  - `urdf`
  - `ocs2/*.info`
  的作用

### 2.4 hardwares

至少需要让用户知道：

- 仿真硬件和控制器是如何接上的
- `gz_quadruped_hardware`
- `hardware_unitree_mujoco`

它们与 controller 的边界是什么。

### 2.5 libraries

至少需要让用户理解：

- `libraries/controller_common`

在整个项目中的作用，例如：

- 公共接口
- 枚举
- 状态名
- 控制输入/控制接口抽象

---

## 3. `unitree_guide_controller` 需要最终看懂的内容

新的 AI 需要确保用户最终可以独立解释下面这些模块。

并且最终能把这些模块重新拼回一条完整链：

`输入 -> FSM -> gait / control / estimator / robot model -> NMPC -> execution -> hardware / simulation`

### 3.1 入口与生命周期

文件重点：

- `UnitreeGuideController.cpp`
- `UnitreeGuideController.h`
- `package.xml`
- `CMakeLists.txt`
- `unitree_guide_controller.xml`
- `launch/*.py`

用户最终要能回答：

- 这个 controller 是如何被加载的
- 生命周期节点做了哪些事
- 它如何拿到 state interfaces / command interfaces
- launch 文件如何把 description、hardware、controller 串起来

### 3.2 FSM 状态机

重点文件：

- `src/FSM/StateFixedStand.cpp`
- `src/FSM/StateFreeStand.cpp`
- `src/FSM/StateTrotting.cpp`
- `src/FSM/StateSwingTest.cpp`
- `src/FSM/StateBalanceTest.cpp`
- `src/FSM/StateNMPC.cpp`

用户最终要能回答：

- FSM 的状态切换逻辑在哪里
- 不同状态各自负责什么
- `run / enter / exit / checkChange` 的职责是什么
- `StateTrotting` 和 `StateNMPC` 在控制思路上有何差别

### 3.3 gait / 步态模块

重点文件：

- `src/gait/WaveGenerator.cpp`
- `src/gait/GaitGenerator.cpp`
- `src/gait/FeetEndCalc.cpp`
- 对应头文件

用户最终要能回答：

- 步态在这个项目里是如何表示的
- phase / contact / swing 是怎么组织的
- gait 逻辑和具体控制状态是什么关系
- 现有 trotting 相关逻辑如何影响腿端目标

### 3.4 control 模块

重点文件：

- `src/control/CtrlComponent.cpp`
- `src/control/Estimator.cpp`
- `src/control/BalanceCtrl.cpp`
- `src/control/LowPassFilter.cpp`
- 对应头文件

用户最终要能回答：

- `CtrlComponent` 为什么像“控制宿主容器”
- `Estimator` 提供了哪些状态
- `BalanceCtrl` 的责任边界是什么
- 当前 guide controller 的控制链是怎样串起来的

### 3.5 robot 模型模块

重点文件：

- `src/robot/QuadrupedRobot.cpp`
- `src/robot/RobotLeg.cpp`
- 对应头文件

用户最终要能回答：

- 机器人模型类保存了哪些信息
- leg 抽象如何组织
- controller 代码里哪里依赖了 robot model

### 3.6 NMPC 相关模块

重点文件：

- `src/nmpc/LeggedInterface.cpp`
- `src/FSM/StateNMPC.cpp`
- `src/control/CtrlComponent.cpp`
- `descriptions/unitree/go1_description/config/ocs2/*.info`

用户最终要能回答：

- 当前自有 `LeggedInterface` 为什么存在
- reference / observation / policy 是怎样流动的
- 当前 NMPC 主链走的是哪条路径
- 为什么当前阶段是 MPC first, WBC later

### 3.7 最终必须建立的 NMPC 开发心智图

新的 AI 需要确保用户最终能在脑中建立下面这张图，而不是只记住零散文件名：

1. NMPC 从哪里接收目标
   - `control_inputs_`
   - `StateNMPC`
   - `TargetTrajectories`
2. NMPC 从哪里接收当前状态
   - `Estimator`
   - joint state interfaces
   - `updateMeasuredRbdStateFromEstimator()`
3. NMPC 问题定义在哪里
   - `nmpc::LeggedInterface`
   - `task.info`
   - `reference.info`
   - `gait.info`
4. solver 如何被创建和推进
   - `CtrlComponent::setupMpcRuntime()`
   - 后台 `advanceMpc()`
   - `evaluateCurrentPolicy()`
5. policy 如何转成执行命令
   - `planned_mode`
   - `solver_contact / leg_roles`
   - `stance` 与 `swing` 分工
   - joint command / support torque

如果新的 AI 发现用户已经能把这 5 点串起来，说明已经接近具备独立 NMPC 开发能力。

---

## 4. 新的 AI 必须采用的工作模式

这是本任务最重要的部分。

### 4.1 主要方式不是“一次性讲完”，而是“讲解 + 指路 + 提问”

新的 AI 不应采用下面这种方式：

- 一次性输出大量总结
- 用户只是被动阅读
- 不确认用户是否真的理解

新的 AI 应采用下面这种方式：

1. 每次只讲一个模块
2. 先给用户一个高层定位
3. 再带用户看关键文件
4. 明确指出：
   - 这个模块在整个控制链里的位置
   - 输入是什么
   - 输出是什么
   - 谁调用它
   - 它再调用谁
5. 再用 2 到 4 个小问题确认理解
6. 根据用户回答判断是否进入下一模块
7. 每一轮结束后，把当前理解进度写回本任务文档

### 4.2 用户是学习者，不是单纯接收者

新的 AI 的任务不是“代替用户读完代码”，而是帮助用户形成自己的理解框架。

因此每一轮都应尽量包含：

- 一个模块目标
- 关键代码入口
- 阅读提示
- 关键概念解释
- 理解检查问题
- 当前进度记录

理解检查问题不要只问“记忆题”，而应优先问：

- 如果去掉这个模块，会发生什么？
- 这个模块和上一个模块的边界在哪里？
- 为什么这里不能直接照搬参考仓库？
- 当前实现和参考实现最大的差异是什么？

### 4.3 每次都要说明为什么先看这个模块

例如：

- 先看 `UnitreeGuideController.cpp`
  - 因为它是 controller 入口
- 再看 FSM
  - 因为它决定各控制状态怎么组织
- 再看 gait
  - 因为 trotting / swing / stance 都要依赖它
- 再看 control 和 robot
  - 因为它们提供状态、模型和控制支撑
- 最后深入 NMPC
  - 因为它建立在前面这些基础之上

新的 AI 不能只说“接下来看看这个文件”，必须告诉用户：

- 为什么现在看它最合适

### 4.4 讲解时必须持续区分“主链”与“执行层”

这是当前学习任务里最容易混淆的一点。

新的 AI 在讲解过程中，必须持续区分：

1. 主链问题

- observation 如何进入 OCS2
- policy 如何产生
- `planned_mode` 如何得到

2. 执行层问题

- policy 如何解释成 joint command
- swing / stance 如何分工
- support torque 和 joint tracking 如何叠加

只有把这两个层次分清楚，用户后面才能独立判断：

- 一个 bug 是 solver 问题
- 还是 execution mapping 问题

### 4.5 用户自己修改文档，或由 AI 代写总结都可以，但必须持续记录

每轮结束后，新的 AI 必须把进度同步到这份文档中，保证：

- 换一个 AI 也能立即接手
- 不会重复从头讲
- 能看到用户已经掌握了哪些模块
- 能看到用户哪些点还不牢

### 4.6 当前阶段最推荐的讲解顺序

由于当前主链已经跑通，新的 AI 接手后应优先按下面顺序带用户理解：

1. `StateNMPC.cpp`
   - 因为它是当前 NMPC 控制状态入口
   - 用户最容易在这里看到“当前实现到底怎么跑”
2. `CtrlComponent.h / CtrlComponent.cpp`
   - 因为它承接 OCS2 runtime
   - 是 observation / policy / planned_mode 的核心宿主
3. `nmpc/LeggedInterface.cpp`
   - 因为这里定义了当前自有 OCS2 问题搭建入口
4. `go1_description/config/ocs2/*.info`
   - 因为这里决定 task/reference/gait 设置
5. `Estimator / BalanceCtrl / QuadrupedRobot`
   - 因为这些决定执行层如何把 policy 落到机器人上

在当前阶段，不建议再次从仓库最外层泛泛漫游开始。

---

## 5. 推荐的整体学习顺序

新的 AI 不必机械执行，但应优先遵循下面的顺序。

注意：

- 顺序看起来是“从全局到局部”
- 但主线必须持续回扣到“这些知识怎样帮助独立做 NMPC”

### 阶段 A：建立仓库地图

目标：

- 让用户知道这个仓库有哪些大块
- 每块大概负责什么

建议阅读：

- `quadruped_ros2_control` 顶层目录
- `controllers`
- `commands`
- `descriptions`
- `hardwares`
- `libraries`

理解检查问题示例：

- `commands` 和 `controllers` 的边界是什么？
- `descriptions` 和 `hardwares` 的角色有何不同？
- 为什么一个控制器仓库里会有多个 controller 包？

### 阶段 B：理解 `unitree_guide_controller` 的入口与主干

目标：

- 让用户知道 controller 是如何启动、配置、运行的

建议阅读：

- `UnitreeGuideController.cpp`
- `UnitreeGuideController.h`
- `launch/*.py`

理解检查问题示例：

- 这个 controller 的主循环大致在哪里？
- 它如何拿到状态接口和命令接口？
- launch 文件里 description 和 controller 是怎么对应上的？

### 阶段 C：理解 FSM 结构

目标：

- 让用户知道“行为模式”是如何组织的

建议阅读：

- `StateFixedStand`
- `StateTrotting`
- `StateNMPC`
- 其他状态文件做对照

理解检查问题示例：

- `enter / run / exit / checkChange` 四个函数各自做什么？
- `StateTrotting` 和 `StateNMPC` 的职责差异是什么？
- 状态切换的触发条件从哪里来？

### 阶段 D：理解 gait / 步态逻辑

目标：

- 让用户知道步态不是“黑箱”

建议阅读：

- `WaveGenerator`
- `GaitGenerator`
- `FeetEndCalc`

理解检查问题示例：

- 当前项目如何表示支撑相和摆动相？
- gait 模块输出的是什么？
- gait 输出最终被谁使用？

### 阶段 E：理解估计器、平衡控制和机器人模型

目标：

- 让用户知道控制不是只靠状态机，还依赖状态估计和模型

建议阅读：

- `Estimator`
- `BalanceCtrl`
- `QuadrupedRobot`
- `RobotLeg`
- `CtrlComponent`

理解检查问题示例：

- `Estimator` 为哪些模块提供输入？
- `CtrlComponent` 为什么会同时持有这么多对象？
- `QuadrupedRobot` 和 `RobotLeg` 的抽象边界是什么？

### 阶段 F：理解输入链路

目标：

- 让用户知道键盘/手柄命令是怎么进控制器的

建议阅读：

- `control_input_msgs`
- `keyboard_input`
- `joystick_input`
- `controller_common` 中与输入相关的接口

理解检查问题示例：

- `lx / ly / rx / command` 的意义是什么？
- 输入是如何从消息进入 FSM 的？
- 为什么同一个控制器既支持键盘也支持手柄？

### 阶段 G：深入 NMPC

目标：

- 在前面基础上，真正理解当前 NMPC 改造

建议阅读：

- `UnitreeGuideController_NMPC_Modification_Plan.md`
- `StateNMPC.cpp`
- `CtrlComponent.cpp`
- `nmpc/LeggedInterface.cpp`
- `go1_description/config/ocs2/*.info`

理解检查问题示例：

- 当前自有 `LeggedInterface` 在替代什么角色？
- 当前 reference / observation / policy 的流向是什么？
- 当前为什么是 `solver mode` 主导执行相位？
- 为什么 `swing` 当前仍保留 `reference foot target -> IK`？
- 为什么当前不能把参考仓库里的 WBC 结果直接照搬？

### 阶段 H：横向对照其他 controller

目标：

- 帮助用户形成“这个项目里不同控制路线的坐标系”

建议阅读：

- `leg_pd_controller`
- `ocs2_quadruped_controller`
- `rl_quadruped_controller`

理解检查问题示例：

- guide controller 与 ocs2 controller 的主要差异是什么？
- RL controller 需要的支撑模块与传统 controller 有什么不同？
- 哪些模块是可复用的，哪些是强耦合的？

### 阶段 I：面向开发的 NMPC 实战读码

这是此前版本文档里没有强调清楚、但现在必须加入的一阶段。

目标：

- 不是停留在“理解 NMPC 主链”
- 而是推进到“用户能独立做小型 NMPC 修改”

建议阅读与演练：

- `StateNMPC.cpp`
- `CtrlComponent.cpp`
- `nmpc/LeggedInterface.cpp`
- `go1_description/config/ocs2/task.info`
- `UnitreeGuideController_NMPC_Modification_Plan.md`

建议演练任务示例：

1. 独立指出 reference 更新逻辑在哪
2. 独立指出 observation 更新逻辑在哪
3. 独立指出 `Q / R` 从配置到 solver 的路径
4. 独立指出 `optimized_input` 如何变成 `joint_velocity_command_interface_`
5. 独立完成一个很小的修改并说明理由

这一阶段完成后，新的 AI 才能认为用户真正接近本任务终点。

---

## 6. 每一轮学习任务的标准模板

新的 AI 每次带用户学习一个模块时，推荐按下面模板输出。

### 6.1 本轮目标

用 1 到 3 句话说明：

- 本轮看什么
- 为什么现在看它

### 6.2 关键文件

明确给出：

- 文件路径
- 建议先看哪一段

### 6.3 阅读提示

不是直接解释全部内容，而是给用户一些“读代码抓手”，例如：

- 先看类成员，再看构造，再看主函数
- 先看 `run()`，再回看 `enter()`
- 先找输入和输出，再补中间过程

### 6.4 理解检查问题

每轮给 2 到 4 个问题，优先用短答题，而不是开放式大作文。

问题要能区分：

- 用户只是看过
- 还是已经真的理解

### 6.5 进度记录

每轮结束后，把下面几项更新到这份文档：

- 当前已完成到哪个阶段
- 用户已掌握的点
- 用户还模糊的点
- 下一轮应该继续看什么
- 当前距离“独立完成 NMPC 修改与开发”还差哪几项能力

---

## 7. 进度记录区

这一部分需要随着后续学习不断更新。

### 7.1 当前起始状态

截至当前，用户已经具备的背景：

- 已经深度参与 `unitree_guide_controller` 的 NMPC 接入与调试
- 已经知道：
  - `StateNMPC`
  - `CtrlComponent`
  - `nmpc::LeggedInterface`
  - `go1_description/config/ocs2/task.info`
  的基本角色
- 已经能跟随逐步修改完成：
  - reference 更新验证
  - `Q/R` 第一轮调参
  - execution clamp 日志增强

这意味着用户已经不再是“从零开始”，而是已经具备：

- 一定的 NMPC 调试参与经验
- 一定的日志驱动验证意识
- 一定的配置与运行链路直觉

但用户当前**还没有系统建立整个仓库的全局理解图**，尤其仍需要系统梳理：

- controller 入口层
- FSM 整体关系
- gait 模块
- `Estimator / BalanceCtrl / Robot model`
- 输入链路
- 仿真/硬件接线
- 其他 controller 的定位

更重要的是，用户当前距离“独立做 NMPC 开发”还缺下面这些能力闭环：

1. 能从入口一路追到 NMPC runtime
2. 能把输入、状态、参考、policy、执行串成完整数据流
3. 能独立判断一个问题更像是：
   - reference 问题
   - 状态估计问题
   - cost / constraint 问题
   - execution 问题
4. 能在不依赖 AI 的前提下自己制定一轮最小验证方案

### 7.2 当前学习任务状态

状态：

- 尚未正式开始第一轮系统代码导读

下一轮建议从这里开始：

- 阶段 A：建立仓库地图
- 然后进入阶段 B：理解 `unitree_guide_controller` 入口与主干

但新的 AI 在每一轮都必须明确补一句：

- “这一轮学到的内容，和你以后独立改 NMPC 的关系是什么”

### 7.3 本轮进度更新（FSM 起步）

当前已完成阶段：

- 已从任务文档重新接手，并确认本任务主线仍然是“围绕独立修改 `unitree_guide_controller` 的 NMPC 建立系统理解”
- 已正式进入阶段 C 的起步部分：
  - 对齐 `UnitreeGuideController::update()` 中的 FSM 调度入口
  - 开始阅读并对比 `StateTrotting` 与 `StateNMPC`

本轮已确认的理解点：

- FSM 的统一生命周期接口来自 `controller_common/FSM/FSMState.h`
  - 所有状态都实现 `enter / run / exit / checkChange`
- `UnitreeGuideController::update()` 的主调度顺序已经明确：
  - 先更新 `robot_model`
  - 再更新 `wave_generator`
  - 再更新 `estimator`
  - 然后执行当前状态的 `run()`
  - 最后通过 `checkChange()` 判断是否进入切换流程
- `StateTrotting` 与 `StateNMPC` 都是 FSM 状态，但控制责任明显不同：
  - `StateTrotting` 是本地经典控制状态，自己完成“命令处理 -> 步态生成 -> 足端力/关节命令计算 -> torque/position/velocity/gain 下发”
  - `StateNMPC` 更像 NMPC 运行时调度状态，主要负责“命令转 reference -> observation 更新 -> 推进 MPC -> 取回 policy -> 下发 joint velocity”
- 当前 `StateNMPC` 的执行输出仍明显比 `StateTrotting` 更窄：
  - `StateTrotting` 同时写 `joint_torque / joint_position / joint_velocity / kp / kd`
  - `StateNMPC` 当前主要写 `joint_velocity_command_interface`

用户当前已掌握或已重新唤醒的点：

- 已重新回到 FSM 学习阶段
- 已重新提出关键问题：
  - `StateTrotting` 和 `StateNMPC` 的区别是什么

当前仍模糊、下一轮要继续补齐的点：

1. `StateNMPC` 中 reference、observation、policy 三者分别落在哪些函数
2. `CtrlComponent` 为什么既是传统 guide controller 组件容器，又是 NMPC 宿主容器
3. `StateTrotting` 的 gait / balance / robot model 链条与 `StateNMPC` 的 solver 链条在系统里如何并列存在
4. `checkChange()` 中不同 command 编号与状态切换关系还需要系统整理

下一轮建议继续阅读：

- `src/control/CtrlComponent.cpp`
- `src/nmpc/LeggedInterface.cpp`
- 必要时对照 `UnitreeGuideController_NMPC_Modification_Plan.md`

这一轮内容和后续独立改 NMPC 的关系：

- 这一轮先分清：
  - 什么代码是在“本地直接控制机器人”
  - 什么代码是在“为 NMPC 提供目标、状态和执行通道”
- 如果这层区分不清，后面做 NMPC 修改时就很容易把问题误判成 gait、估计器或执行层问题。

### 7.4 本轮进度更新（NMPC 数据流起步）

当前已完成阶段：

- 已基于用户回答，继续进入阶段 G 的起步部分
- 已开始把 `StateNMPC -> CtrlComponent -> LeggedInterface` 这条 NMPC 主链按数据流拆成：
  - reference
  - observation
  - policy
  - execution

本轮已确认的理解点：

- 用户已经能准确区分 `StateTrotting` 和 `StateNMPC` 的职责差异：
  - `StateTrotting` 更像完整本地控制器
  - `StateNMPC` 更像 NMPC 生命周期调度状态
- 用户已经确认 `StateNMPC` 当前最终下发的是关节速度命令
- `CtrlComponent` 中四段关键链路已初步对齐：
  - reference 更新：
    - `updateDesiredBodyVelocityCommand()`
    - `updateTargetTrajectoriesFromCommand()`
  - observation 更新：
    - `updateMeasuredRbdStateFromEstimator()`
  - solver 推进与 policy 获取：
    - `advanceMpcOnce()`
    - `evaluateCurrentPolicy()`
  - execution 下发：
    - `StateNMPC::applyNmpcJointVelocityCommand()`

用户当前已掌握或已重新唤醒的点：

- 已能从“控制逻辑完整性”角度解释 `Trotting` 与 `NMPC` 的差别
- 已开始具备把 NMPC 问题按链路拆分排查的意识

当前仍模糊、下一轮要继续补齐的点：

1. `updateTargetTrajectoriesFromCommand()` 里真正修改的是哪部分状态，哪些状态暂时没有被 reference 改动
2. `updateMeasuredRbdStateFromEstimator()` 是如何把 estimator + joint state interface 组合成 RBD state，再转成 centroidal state
3. `evaluateCurrentPolicy()` 中为什么要从 `optimized_input_` 里裁出 contact force 之后的那一段，才能得到 joint velocity command
4. `LeggedInterface` 在“问题定义层”里到底创建了哪些对象：
   - cost
   - dynamics
   - rollout
   - reference manager
   - initializer

这一轮内容和后续独立改 NMPC 的关系：

- 后续做 NMPC 修改时，最常见的不是“代码不会改”，而是“改错层”。
- 如果能稳定把问题分解到 `reference / observation / policy / execution` 四段，后面不管是调 `Q/R`、改 reference，还是验证执行映射，都会快很多。

### 7.5 本轮进度更新（reference / observation / policy 初步确认）

当前已完成阶段：

- 已通过提问确认用户对 `CtrlComponent` 中三段核心数据流的第一层理解：
  - reference 改了什么
  - observation 从哪里来
  - policy 为什么需要切片

本轮已确认的理解点：

- 用户已经能指出 `updateTargetTrajectoriesFromCommand()` 当前主要改动的是 base 的 `x / y / yaw`
- 用户已经能指出 `updateMeasuredRbdStateFromEstimator()` 中：
  - joint position / joint velocity 来自 state interfaces
  - base 相关状态主要来自 `Estimator`
- 用户已经理解 `optimized_input_` 不能整段直接当作 joint velocity 下发，必须按输入布局切片

当前仍需继续加固的点：

1. 需要从“知道改了哪些量”进一步升级到“知道为什么当前只改了这些量，以及其余状态为什么保持 observation 原值”
2. 需要从“知道要切片”进一步升级到“知道输入向量前半段是接触相关输入，后半段才是 actuated joint velocity”
3. 还需要建立更稳定的 NMPC 排障优先级：
   - `reference`
   - `observation`
   - `problem definition`
   - `execution`

这一轮内容和后续独立改 NMPC 的关系：

- 这一步的目标是让用户不只会“看日志”，而是能看懂“这条日志属于 NMPC 主链的哪一段”。
- 一旦这层能力建立，后面调 reference、查状态映射、查 policy 到执行层的错位都会更直接。

补充确认：

- 用户已经能判断：
  - 当 `target cmd vx/vy/yaw_rate` 在变化，但 `optimized_input_` 基本不变时，应优先怀疑 `reference` 链路
  - 当前关键检查点应是“参考轨迹是否真的被成功构造并送入 solver”
- 用户已经理解：
  - 当前 `updateTargetTrajectoriesFromCommand()` 采用的是“先拷贝当前 `observation_.state`，再局部修改少数 reference 状态量”的思路
  - 当前未被显式修改的 reference 状态并不是清零，而是保持当前观测值
- 用户已经能判断：
  - 如果以后希望 NMPC 明确跟踪某个机身高度，应优先修改 reference 构造层，而不是直接改执行层
  - 也即优先修改 `updateTargetTrajectoriesFromCommand()` 中 `x1` 的构造逻辑

### 7.6 本轮进度更新（LeggedInterface 与配置分层起步）

当前已完成阶段：

- 已开始进入 `LeggedInterface` 阅读
- 已开始区分：
  - FSM 调度层
  - NMPC runtime 宿主层
  - NMPC 问题定义层

本轮已确认的理解点：

- 用户已经能说明三层职责分工：
  - `StateNMPC` 更像面向 FSM 的调度壳
  - `CtrlComponent` 更像 runtime 生命周期宿主
  - `LeggedInterface` 负责更底层的 NMPC 接口准备与问题定义装配
- 用户已经能指出 `LeggedInterface` 负责创建的关键对象类型包括：
  - settings
  - optimal control problem
  - reference manager
  - initializer
  - rollout
  - dynamics
- 已确认当前 `LeggedInterface` 实际直接读取的是：
  - `task.info`
  - `reference.info`
- 已确认当前 `gait.info` 虽然存在于配置目录中，但在当前 `unitree_guide_controller` 这条 NMPC 主链代码里并没有真正接入使用
- 已确认当前 gait schedule 的加载来源其实是 `reference.info` 中的：
  - `initialModeSchedule`
  - `defaultModeSequenceTemplate`

当前仍需继续加固的点：

1. 需要把“文件名直觉”升级成“代码真实依赖”：
   - 不是看到 `gait.info` 就默认它已经参与当前求解链
2. 需要明确：
   - `task.info` 更偏问题定义、权重、模型和求解器设置
   - `reference.info` 更偏参考轨迹和模式/步态调度入口
   - `gait.info` 当前更像仓库内的备用或其他路径配置，而不是这条 guide-controller NMPC 主链的有效输入
3. 还需要继续追 `LeggedInterface::initialize()`，确认对象创建顺序与依赖关系

这一轮内容和后续独立改 NMPC 的关系：

- 后面如果你要改步态、参考、权重或求解器配置，第一件事不是凭文件名猜，而是先确认“当前控制链到底读没读这个文件”。
- 这是避免低效调参和错误修改范围的关键能力。

补充确认：

- 用户已经能明确说明：
  - 当前步态调度入口来自 `reference.info`
  - 不是来自 `gait.info`
- 用户已经能判断：
  - 如果修改 `gait.info` 后机器人行为完全不变，第一反应应是“该文件当前没有真正接入这条控制链”
- 用户已经能区分：
  - `task.info` 更偏问题定义、模型/求解器/权重配置
  - `reference.info` 更偏任务目标与参考轨迹/模式调度

### 7.7 本轮进度更新（LeggedInterface 装配视角）

当前已完成阶段：

- 已从“文件和对象列表”继续推进到“按装配依赖顺序理解 `LeggedInterface::initialize()`”

本轮已确认的理解点：

- 用户已经能判断：
  - `initialize()` 开头的文件路径、文件存在性、`joint_names / foot_names` 检查属于装配前检查
  - 不属于问题定义本体
- 用户已经能判断：
  - `loadBasicConfig()` 的主要作用不是简单确认文件能打开
  - 而是确认关键 section 是否齐全，能否支持后续装配
- 用户已经能区分：
  - `ReferenceManager` 更偏“管理 solver 跟踪什么”
  - `OptimalControlProblem / Dynamics / Cost / Rollout` 整体属于问题定义层
- 用户已经能把 `CtrlComponent` 与 `LeggedInterface` 的边界分开：
  - 前者更偏 runtime 生命周期
  - 后者更偏基础装配与问题定义

当前仍需继续加固的点：

1. `ReferenceManager` 与 `Initializer` 的角色还需要进一步区分：
   - 一个更偏目标管理
   - 一个更偏为求解提供初始化/起始策略支撑
2. 还需要把 `LeggedInterface::initialize()` 的完整对象创建顺序真正串成一条链

这一轮内容和后续独立改 NMPC 的关系：

- 当 NMPC 起不来时，后面排查不应只盯某个报错对象，而应能顺着装配依赖往前追到配置、模型和 reference 入口。

补充确认：

- 用户已经能判断：
  - 如果 `ReferenceManager` 创建失败，而模型基础件已成功建立，应优先排查 `reference` 配置/步态调度层
- 用户已经开始区分：
  - `LeggedInterface initialized` 更像“问题定义与基础接口已装好”
  - `setupMpcRuntime()` 更像“把这些基础件真正接成可运行的 MPC runtime，并接入控制主链”

### 7.8 本轮进度更新（setupMpcRuntime 与 observation 起步）

当前已完成阶段：

- 已从 `LeggedInterface` 继续推进到 `CtrlComponent::setupMpcRuntime()`
- 已开始区分：
  - problem definition ready
  - runtime ready
  - observation update

本轮已确认的理解点：

- 用户已经理解：
  - `LeggedInterface initialized` 不等于 `MPC runtime ready`
  - 前者表示问题定义层与基础对象已装好
  - 后者表示 solver runtime 已真正创建并可接入控制主链
- 用户已经能判断：
  - 如果 `OptimalControlProblem / Rollout / ReferenceManager / Initializer` 等前置对象未准备好，不应继续创建 `SqpMpc`
  - 原因是 solver 创建时就依赖这些问题定义层对象
- 用户已经开始理解 `MPC_MRT_Interface` 的角色：
  - 它不是 solver 本体
  - 而是控制器与 solver 之间的运行时桥梁
- 用户已经理解 runtime 状态量初始化的本质：
  - 不是简单缓存最新数据
  - 而是建立一套合法、维度正确、可启动的初始运行上下文
- 用户已经理解：
  - `measured_rbd_state_` 启动时先用 `initial_state` 顶上，是为了让 runtime 第一拍就有合法状态
  - 之后再由实时 observation 更新覆盖
- 用户已经能说明：
  - `updateMeasuredRbdStateFromEstimator()` 的核心职责是把 estimator 和 joint state interface 组合成 RBD state，再转换成 solver 需要的 centroidal state
- 用户已经能区分：
  - base 状态主要来自 `Estimator`
  - joint position / velocity 来自 state interfaces

当前仍需继续加固的点：

1. `evaluateCurrentPolicy()` 为什么要通过 MRT 重新评估当前 policy，而不是直接使用已有输入缓存
2. `optimized_input_` 的布局为什么不能整体直接下发，切片逻辑还需要继续巩固
3. `updateMeasuredRbdStateFromEstimator() -> advanceMpcOnce() -> evaluateCurrentPolicy()` 三步的时序关系还需要彻底串成闭环

这一轮内容和后续独立改 NMPC 的关系：

- 这一步已经把“问题定义层”和“运行时层”真正分开。
- 后面如果你修改代码或排查问题，就能先判断故障更像出在：
  - solver 还没真正 ready
  - observation 不合法
  - policy 没更新
  - execution 映射有问题
- 这会直接减少后续修改 NMPC 时的误判范围。

---

## 8. 新 AI 接手时的强约束

新的 AI 接手时，请严格遵守：

1. 不要一上来就继续做 NMPC 开发
   - 这是一个新的“系统学习任务”
2. 不要一次性灌输整仓库总结
   - 必须分阶段推进
3. 不要只讲，不提问
   - 必须通过问题确认用户理解程度
4. 不要只提问，不给提示
   - 必须提供阅读抓手和关键线索
5. 每完成一轮学习，都要更新本任务文档
6. 优先目标是帮助用户“独立完成 NMPC 修改与开发”，看懂项目是为这个目标服务
7. 如果某个模块与 NMPC 主线关系较弱，不要投入过深篇幅

---

## 9. 建议开场方式

新的 AI 接手这个任务后，建议第一轮这样开始：

1. 先告诉用户：
   - 这次虽然会先建立整个项目地图
   - 但最终目标不是泛读仓库，而是让你能独立做 NMPC 修改和开发
2. 先带用户看：
   - `quadruped_ros2_control` 顶层目录结构
3. 再问用户几个短问题，例如：
   - 你觉得 `commands`、`controllers`、`descriptions` 分别像什么角色？
   - 如果让你找“键盘命令进入 controller 的入口”，你会先去哪类目录？
4. 根据用户回答，再决定是否进入下一阶段

这样可以最大程度保持：

- 有节奏
- 有逻辑
- 以提问和提示为主
- 逐渐建立用户自己的理解框架
- 并始终向“独立完成 NMPC 开发”这个终点收束

---

## 10. 本任务的完成标准

新的 AI 不应以“已经讲完很多文件”为完成标准，而应以下面这些能力是否建立为完成标准：

1. 用户能独立说出 `unitree_guide_controller` 的整体控制主链
2. 用户能独立定位 NMPC 关键文件和关键配置
3. 用户能独立解释：
   - reference 从哪里来
   - observation 从哪里来
   - solver 在哪里建
   - policy 如何下发
4. 用户能独立提出并执行一轮最小 NMPC 修改方案
5. 用户能在 AI 只给少量提示的情况下，自己继续读懂后续 NMPC 代码

只有满足这些条件，才能认为这项任务真正完成。
