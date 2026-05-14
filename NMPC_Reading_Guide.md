# NMPC Source Reading Guide for `MPC_legged_control`

这份笔记只保留 `MPC/MPC_legged_control` 里和非线性模型预测控制（NMPC）实现思路直接相关的部分，目的是帮你快速抽出可迁移的设计，而不是把整个项目都读一遍。

## 1. 先建立整体图景

如果只看 NMPC 主线，可以把这套代码压缩成下面 4 层：

1. `legged_controllers/src/LeggedController.cpp`
   运行时入口，负责在控制循环里调用 MPC。
2. `legged_interface/src/LeggedInterface.cpp`
   把机器人问题组装成 OCS2 的最优控制问题。
3. `legged_interface/src/constraint/*.cpp`
   定义接触、摆腿、摩擦等约束。
4. `legged_controllers/config/<robot>/*.info`
   提供 MPC 参数、任务权重、步态和参考轨迹。

如果你的目标是“借鉴实现思路”，最重要的是理解：

- 状态 `x` 和输入 `u` 怎么定义
- 动力学模型怎么接进求解器
- 代价函数怎么组织
- 约束怎么按接触相位启停
- 在线控制循环中如何反复调用求解器

## 2. 阅读优先级

### 必看

- `MPC/MPC_legged_control/README.md`
- `MPC/MPC_legged_control/legged_interface/src/LeggedInterface.cpp`
- `MPC/MPC_legged_control/legged_controllers/src/LeggedController.cpp`
- `MPC/MPC_legged_control/legged_controllers/config/a1/task.info`
- `MPC/MPC_legged_control/legged_controllers/config/a1/reference.info`
- `MPC/MPC_legged_control/legged_interface/src/constraint/FrictionConeConstraint.cpp`
- `MPC/MPC_legged_control/legged_interface/src/constraint/ZeroForceConstraint.cpp`
- `MPC/MPC_legged_control/legged_interface/src/constraint/ZeroVelocityConstraintCppAd.cpp`
- `MPC/MPC_legged_control/legged_interface/src/constraint/NormalVelocityConstraintCppAd.cpp`
- `MPC/MPC_legged_control/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp`

### 选看

- `MPC/MPC_legged_control/legged_interface/src/LeggedRobotPreComputation.cpp`
- `MPC/MPC_legged_control/legged_interface/src/SwitchedModelReferenceManager.cpp`
- `MPC/MPC_legged_control/legged_controllers/src/TargetTrajectoriesPublisher.cpp`

### 可以先跳过

- `legged_wbc/`
- `legged_estimation/`
- `legged_hw/`
- `legged_gazebo/`
- `legged_examples/legged_unitree/legged_unitree_hw/`
- `qpoases_catkin/`

这些目录很重要，但它们更多是在解决“状态怎么测”“扭矩怎么落地”“仿真和实机怎么跑”，不是 NMPC 本体。

## 3. 第一阶段：先看 README，搞清楚问题定义

`README.md` 的价值不在于教你怎么编译，而在于它先给出了这套 NMPC 的抽象方式。

你要重点抓住两件事：

- 状态定义
  文中把状态写成质心动量、机身姿态、关节位置等组合量。
- 输入定义
  输入是接触力和关节速度，而不是直接把关节力矩作为 NMPC 的控制量。

这个选择很关键，因为它说明这套框架采用的是：

- 上层 NMPC 负责规划全身运动和接触力
- 下层 WBC 再把 NMPC 输出转换成关节力矩

如果你以后想迁移到自己的工程，需要先决定是否保留这个分层结构。  
如果你只想借鉴 NMPC 思路，这个分层是值得保留的，因为它把优化问题和执行问题拆开了。

## 4. 第二阶段：读 `LeggedInterface.cpp`，这是 NMPC 的建模核心

文件：

- `MPC/MPC_legged_control/legged_interface/src/LeggedInterface.cpp`

这个文件最值得精读的函数是：

### `LeggedInterface::LeggedInterface(...)`

这里主要做三件事：

- 检查 `taskFile / urdfFile / referenceFile` 是否存在
- 从 `taskFile` 读取模型、MPC、SQP、rollout 等设置
- 保存这些设置供后面组装最优控制问题使用

对你来说，这一段最值得借鉴的点是：

- 优化器参数、模型参数、参考参数全部外置到配置文件
- C++ 只负责组装，不把大量数值硬编码进源码

### `setupOptimalControlProblem(...)`

这是全文件最重要的函数。  
如果时间有限，就优先把这个函数完整看懂。

它的结构大致是：

1. `setupModel(...)`
   建立 Pinocchio 模型和 centroidal model 信息。
2. 读取 `initialState`
3. `setupReferenceManager(...)`
   把 gait 和摆腿参考接进来。
4. `problemPtr_ = std::make_unique<OptimalControlProblem>()`
   创建 OCS2 最优控制问题对象。
5. `problemPtr_->dynamicsPtr = ...`
   设置动力学模型。
6. `problemPtr_->costPtr->add(...)`
   加入代价项。
7. `problemPtr_->inequalityConstraintPtr->add(...)`
   加入不等式约束。
8. `problemPtr_->equalityConstraintPtr->add(...)`
   加入等式约束。
9. `problemPtr_->stateSoftConstraintPtr->add(...)`
   加入软约束。
10. `setupPreComputation(...)`
11. 构造 rollout 和 initializer

如果你以后要在自己的系统里重写一版 NMPC，这个函数的结构几乎可以直接当模板。

### `setupModel(...)`

这里的重点是：

- 用 URDF + Pinocchio 建立刚体模型
- 生成 `centroidalModelInfo_`

这意味着这套 NMPC 并不是直接拿全刚体状态做控制量，而是把问题整理成质心动力学相关的形式。  
这是一种常见且很实用的折中：比全刚体优化更轻，但比纯运动学更保真。

### `setupReferenceManager(...)`

这里把 gait schedule 和 swing trajectory planner 组织起来。  
对 NMPC 来说，这一层的作用是：

- 决定某个时刻哪条腿是支撑腿
- 决定摆腿时足端参考高度或法向速度要求

也就是说，步态切换不是写死在控制器主循环里，而是作为参考管理器输入到优化问题。

## 5. 第三阶段：读约束实现，理解“接触相位如何进入 NMPC”

这个项目的 NMPC 思路最值得借鉴的一点，是它把不同接触状态下的约束切换得很清楚。

### `FrictionConeConstraint.cpp`

文件：

- `MPC/MPC_legged_control/legged_interface/src/constraint/FrictionConeConstraint.cpp`

这个约束在足端接触地面时激活：

- `isActive(time)` 返回当前腿是否接触

它做的事是：

- 取出该腿接触力
- 变换到局部坐标
- 构造摩擦锥约束值
- 提供一阶和二阶导数给优化器

这部分非常值得借鉴，因为它体现了一个成熟 NMPC 工程实现的做法：

- 不只是定义约束值
- 还明确给出线性化和二阶近似

如果你后续自己写求解器接口，这里要重点学习的是：

- 约束函数的数学形式如何落到代码
- 导数如何围绕输入向量中某一段接触力分量展开

### `ZeroForceConstraint.cpp`

文件：

- `MPC/MPC_legged_control/legged_interface/src/constraint/ZeroForceConstraint.cpp`

这个约束在摆腿相激活：

- `isActive(time)` 返回“非接触时为真”

它表达的物理意思很直接：

- 脚离地时，这条腿的接触力必须为零

这类约束非常适合迁移，因为它简单、清晰、物理意义强。  
如果你以后做自己的四足或双足 NMPC，这通常都是第一批应该保留的约束。

### `ZeroVelocityConstraintCppAd.cpp`

文件：

- `MPC/MPC_legged_control/legged_interface/src/constraint/ZeroVelocityConstraintCppAd.cpp`

这个约束在支撑相激活，要求支撑足速度为零。  
核心逻辑非常短，但意义很大：

- 接触腿不允许在地面上乱滑
- 通过末端线性约束来表达足端速度限制

这里最值得借鉴的是它的组织方式：

- 约束类不直接自己做全部运动学推导
- 而是把工作委托给 `EndEffectorLinearConstraint`

这种“业务约束 + 底层运动学工具”分层方式很适合复用。

### `NormalVelocityConstraintCppAd.cpp`

文件：

- `MPC/MPC_legged_control/legged_interface/src/constraint/NormalVelocityConstraintCppAd.cpp`

这个约束在摆腿时激活，用来约束足端法向速度。  
它的特殊点在于：

- 约束参数不是固定死的
- 会从 `LeggedRobotPreComputation` 动态取配置

这说明这套 NMPC 不是只做静态约束，而是在优化前根据当前参考、接触状态等预计算一些局部线性约束配置。

这点很值得借鉴，因为它比“每次在约束内部临时现算一切”更工程化。

### `SwingTrajectoryPlanner.cpp`

文件：

- `MPC/MPC_legged_control/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp`

这部分很重要，但容易被误读。  
它不是求解器本身，而是给求解器提供“摆腿阶段应该满足什么足端高度/速度参考”。

这个类主要做的事：

- 根据 `modeSchedule` 提取每条腿在各阶段的接触标志
- 找到每段 swing 的起止时刻
- 为足端高度生成三次样条
- 提供某时刻应满足的足端高度和法向速度约束

如果你借鉴 NMPC 实现思路，务必注意这一点：

- gait 不是单纯影响 cost
- gait 还会改变约束结构本身

## 6. 第四阶段：读 `LeggedController.cpp`，理解在线求解流程

文件：

- `MPC/MPC_legged_control/legged_controllers/src/LeggedController.cpp`

如果 `LeggedInterface.cpp` 解决的是“要解什么问题”，那么这个文件解决的是“怎么在实时循环里解它”。

重点看下面几个函数。

### `init(...)`

这里完成系统装配：

- 读取 `urdfFile / taskFile / referenceFile`
- 调用 `setupLeggedInterface(...)`
- 调用 `setupMpc()`
- 调用 `setupMrt()`
- 初始化状态估计
- 初始化 WBC

如果你只关心 NMPC，需要重点盯住的不是硬件接口部分，而是：

- `setupLeggedInterface(...)`
- `setupMpc()`
- `setupMrt()`

### `starting(...)`

这里做的是在线 MPC 很常见的一步：

- 构造初始观测
- 先把当前状态和初始参考送进求解器
- 等到第一份可用 policy 出来之后再进入正式控制

这段流程很有借鉴价值，因为它解决的是一个工程上很实际的问题：

- 控制器启动时不能直接假定 MPC 已经有结果

### `update(...)`

这是运行时主循环，和 NMPC 直接相关的主线可以压缩成：

1. 更新状态估计
2. `setCurrentObservation(currentObservation_)`
3. `updatePolicy()`
4. `evaluatePolicy(...)`
5. 取出当前时刻对应的优化状态和输入

如果只关心 NMPC，到这一步其实已经够了。  
后面的 WBC、力矩映射和可视化都可以暂时弱化。

这段代码最值得借鉴的实现思路是：

- MPC 在线运行在后台持续更新 policy
- 控制循环每个周期只评估“当前时刻该执行的那一点”

这比每个控制周期都从头完整求解一次更工程化，也更接近高频控制里的常见架构。

### `setupMpc()`

这里是求解器真正初始化的地方。  
重点关注：

- `SqpMpc` 的构造方式
- reference manager 和 gait receiver 怎么接入
- 如何把 ROS 通信与 MPC 同步起来

如果你未来不打算继续用 OCS2，也依然值得看，因为这里展示了一个很通用的工程结构：

- 求解器
- 参考输入模块
- 观测输入模块
- policy 输出模块

## 7. 第五阶段：回头对照配置文件

建议至少选一个机器人型号看完整套配置。  
例如：

- `MPC/MPC_legged_control/legged_controllers/config/a1/task.info`
- `MPC/MPC_legged_control/legged_controllers/config/a1/reference.info`
- `MPC/MPC_legged_control/legged_controllers/config/a1/gait.info`

这里最值得找的是：

- `model_settings`
- `mpc`
- `sqp`
- `rollout`
- `swing_trajectory_config`
- `initialState`
- `initialModeSchedule`
- `defaultModeSequenceTemplate`

阅读方式建议是：

1. 先在 `LeggedInterface.cpp` 里看到某个配置键名
2. 再回到 `task.info/reference.info` 查具体数值

这样你会很快知道：

- 哪些是结构设计
- 哪些只是参数选择

## 8. 如果你只想迁移“NMPC 思路”，真正该提炼什么

不要试图整体复刻这个仓库。  
更值得提炼的是下面这些可迁移设计。

### A. 把问题定义和实时控制循环分开

- `LeggedInterface` 负责定义优化问题
- `LeggedController` 负责运行时更新和调用

这是很好的架构边界，建议保留。

### B. 配置外置

把下面这些东西放到配置文件，而不是硬编码：

- 预测时域
- SQP 参数
- 权重矩阵
- gait 模板
- swing 高度与速度参数
- 初始状态

### C. 约束按接触相位动态启停

这是四足 NMPC 里最应该借鉴的点之一：

- 支撑相启用摩擦锥和零足端速度
- 摆动相启用零接触力和法向摆腿约束

### D. 使用预计算层

`LeggedRobotPreComputation` 这类设计很实用。  
它可以把一些和当前时刻、当前参考相关，但又不想在每个约束内部重复现算的东西提前整理好。

### E. 在线上只做 policy evaluation

不是每次控制周期都完整求解一次，而是：

- 后台推进优化
- 前台读取最新 policy 并在当前时刻评估

这个思路对实时性很重要。

## 9. 一个最省时间的阅读顺序

如果你想在最短时间内抓住 NMPC 主干，建议直接按下面顺序读：

1. `README.md`
   先搞清楚状态、输入、整体方法。
2. `legged_interface/src/LeggedInterface.cpp`
   只盯 `setupOptimalControlProblem()`、`setupModel()`、`setupReferenceManager()`
3. `legged_interface/src/constraint/ZeroForceConstraint.cpp`
   先看最简单的约束切换。
4. `legged_interface/src/constraint/ZeroVelocityConstraintCppAd.cpp`
   看支撑相足端速度约束。
5. `legged_interface/src/constraint/NormalVelocityConstraintCppAd.cpp`
   看摆腿相法向约束。
6. `legged_interface/src/constraint/FrictionConeConstraint.cpp`
   看接触力不等式约束和导数实现。
7. `legged_interface/src/constraint/SwingTrajectoryPlanner.cpp`
   看 gait 如何转成时变足端参考。
8. `legged_controllers/src/LeggedController.cpp`
   只看 `init()`、`starting()`、`update()`、`setupMpc()`
9. `config/a1/task.info` 和 `reference.info`
   回头把数值配置和源码对起来。

## 10. 结合你当前工程时，最值得对照的点

你当前工作区里已经有自己的控制器，例如 `LegPdController.cpp`。  
如果你后面想把这里的思路迁移到自己的 ROS 2 工程，建议优先对照下面几个问题：

- 你自己的状态量是否已经足够支撑 NMPC 预测
- 你是否也打算采用“上层规划接触力，下层做力矩分配”的分层结构
- 你当前控制循环频率是否允许后台优化加前台评估的结构
- 你是否需要先做一个简化版，只保留 `ZeroForce + ZeroVelocity + FrictionCone`

对大多数迁移工作来说，先做“简化约束集 + 简化参考管理”会更稳，而不是一开始就完整搬所有约束。

## 11. 一句话总结

如果只借鉴这套代码的 NMPC 实现思路，请优先抓住这条主线：

`LeggedInterface` 定义问题，`constraint/` 定义接触时变约束，`LeggedController` 负责在线调用，`task.info/reference.info` 提供外置参数。

只要把这四部分读透，你就已经拿到了这套仓库里和 NMPC 最相关、最值得迁移的核心设计。
