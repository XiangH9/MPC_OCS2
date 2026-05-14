# MPC/WBC 代码迁移执行清单

## 1. 这份文档的作用

这份文档是路线图的执行版。

它主要回答四个非常实际的问题：

1. 现有哪些文件短期内应该保留。
2. 现有哪些文件未来大概率要重构或替换。
3. 整个升级工作应该按什么顺序推进。
4. 每一个阶段改完之后，应该怎么判断是否真的变好了。

这份文档的目标不是继续把当前简化版 trotting 调到极致。

它的目标是帮助我们把当前项目逐步迁移到：

- 更强的状态估计
- 基于 MPC 的运动规划
- 基于 WBC 的全身控制

## 2. 当前仓库中各部分的大致角色

### 2.1 目前仍然适合作为稳定基础设施保留的文件

这些文件短期内仍然非常有价值，应该尽量保留：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/UnitreeGuideController.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/control/CtrlComponent.h`
- `src/quadruped_ros2_control/libraries/controller_common/include/controller_common/CtrlInterfaces.h`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/launch/gazebo_classic.launch.py`

保留它们的原因：

- 它们提供了 ROS 2 controller 的整体壳子
- 它们提供了 command/state interface 绑定
- 它们提供了控制器生命周期与 FSM 入口
- 它们提供了 Gazebo 仿真的可运行路径

简单说：

- 这些文件更适合作为“集成骨架”
- 不应该把它们当成当前的主要算法瓶颈

### 2.2 算法上最关键、后续最可能重写的文件

下面这些文件，是真正会决定未来控制质量的部分：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/FSM/StateTrotting.h`

原因：

- `Estimator.cpp` 决定控制器看到的状态到底准不准。
- `StateTrotting.cpp` 现在承载了 locomotion 的核心逻辑。
- `StateTrotting.h` 后续很可能随着模块拆分而变化。

### 2.3 未来应该“变薄”的文件

这类文件不是说写错了，而是当前职责过重。

- `StateTrotting.cpp`

原因：

- 它现在同时承担了命令解释、目标生成、步态处理、站立逻辑、力矩生成和调试输出
- 这种结构适合 baseline，不适合做现代高质量运动控制系统

未来应该朝这个方向变化：

- `StateTrotting` 主要负责协调
- 真正的控制智能放在 planner / controller 模块里

## 3. 整体迁移原则

后续修改应当始终遵循三个原则。

### 原则 1：不要一次替换所有东西

不要在一个阶段里同时大改：

- 估计器
- planner
- controller

否则很容易出现：

- 出问题时完全不知道是哪里炸了
- 改进和退化无法归因

### 原则 2：先建评估体系，再做大替换

在真正替换核心模块之前，必须先能稳定测量：

- base 位姿误差
- base 速度误差
- 姿态误差
- 原地漂移
- 命令跟踪误差
- 控制输出平滑性

### 原则 3：始终保持当前系统可运行

虽然当前 trotting 很弱，但它仍然是一个有价值的 baseline。

所以每个阶段都应尽量保证：

- Gazebo launch 还能跑
- 键盘控制链路还能跑
- 日志还能采集

## 4. 推荐的升级顺序

更合理的顺序通常是：

1. 先搭评估与日志体系。
2. 再彻底看清估计器质量。
3. 然后升级估计器。
4. 然后重构 locomotion 架构。
5. 再引入 MPC。
6. 再引入 WBC。
7. 最后统一调参与对比 baseline。

这个顺序的好处是：

- 每一步都能知道自己到底改对了什么
- 不会把问题混在一起

## 5. 分阶段代码迁移计划

## Phase 0：冻结 baseline

### 目标

保留当前项目作为可重复对比的 baseline。

### 要做什么

- 保持当前 launch 路径可运行
- 保持当前调试日志可用
- 避免直接在 baseline 分支上做大量一次性实验

### 推荐的 git 使用方式

后续所有实验最好都在下面这个仓库里单独开分支：

- `src/quadruped_ros2_control`

分支命名示例：

- `debug/estimator-eval`
- `debug/trotting-analysis`
- `feature/contact-estimator`
- `feature/mpc-centroidal`
- `feature/wbc-qp`

### 这一阶段的完成标准

- 仿真能正常启动
- 控制器能正常加载
- baseline trotting 行为能稳定复现
- baseline 日志可以重复采集

## Phase 1：建立评估基础设施

### 目标

在替换核心模块之前，先建立客观评估方法。

### 可能需要动到的代码区域

主要可能是：

- 工作区根目录下新增工具脚本
- 或新增 utilities 目录
- 在下面这些文件里少量加日志：
  - `Estimator.cpp`
  - `StateTrotting.cpp`

### 这一阶段应该记录什么

估计器相关：

- estimated base position
- estimated base velocity
- estimated orientation
- IMU 原始数据
- contact state

控制器相关：

- 目标 body position `pcd_`
- 实际 body position
- 目标速度
- 实际速度
- torque 输出
- wave/gait 状态

仿真真值：

- Gazebo 里的 base pose / velocity ground truth

### 这一阶段结束后应该能回答的问题

- 当前失稳主要来自估计器，还是主要来自控制器，还是两者都有问题？
- 漂移发生在接触切换前，还是切换后？
- 控制发散时，估计是否仍然合理？

### 完成标准

- 有一套可重复运行的日志采集流程
- baseline standing 与 trotting 日志可以导出
- 在仿真中能做 truth-vs-estimate 对比

## Phase 2：升级估计器

### 目标

在高级控制接管之前，先把 base 状态估计做强。

### 当前文件

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`

### 它目前的职责

目前它主要融合：

- IMU
- 足端运动学
- 接触信息

来估计：

- base 位置
- base 速度
- base 姿态

### 这一阶段为什么重要

如果下面这些量不准：

- base velocity
- contact trust
- orientation
- standing pose

那 MPC 和 WBC 上来之后也只会基于错误状态做更复杂的错误控制。

### 推荐重构方向

第一步先尽量保持接口不变，只升级内部实现。

可选方向：

- 用 Error-State EKF 替换当前估计核心
- 或用 Invariant EKF 替换当前估计核心

但尽量保持以下输出接口风格稳定：

- `getPosition()`
- `getVelocity()`
- `getRotation()`
- `getGyroGlobal()`
- `getYaw()`

这样可以让系统在迁移期仍然保持可跑。

### 完成标准

- standing 漂移明显减小
- 速度估计更平滑、更可信
- 估计值与仿真真值误差明显下降
- 接触切换时估计不会大跳

## Phase 3：拆分 locomotion 架构

### 目标

不再让 `StateTrotting.cpp` 承担几乎所有运动控制职责。

### 当前文件

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`

### 当前问题

这个文件现在同时处理：

- 用户命令解释
- 机身目标速度生成
- 机身目标位置积分
- gait 生成
- stance/swing 处理
- 全身力控制
- torque 生成
- debug 输出

### 重构方向

概念上应该拆成：

1. command/reference manager
2. gait/contact scheduler
3. planner
4. controller

不一定一天之内拆完，但方向必须清楚。

### 可能新增的模块

未来可以考虑新增类似这些文件：

- `control/ReferenceManager.*`
- `gait/ContactScheduler.*`
- `control/MpcPlanner.*`
- `control/WholeBodyController.*`

### 完成标准

- `StateTrotting` 主要负责 orchestration
- 参考生成和力矩生成不再强耦合
- 控制核心逻辑不再硬塞在一个 FSM 文件里

## Phase 4：引入 MPC

### 目标

用预测优化替换当前启发式机身力控制思路。

### 当前将来会被替代的逻辑

当前 `StateTrotting.cpp` 里面大致是这样一条链：

- 计算 `pos_error_`
- 计算 `vel_error_`
- 生成期望机身加速度
- 生成期望角加速度
- 调 balance/force allocator

这一部分以后应该逐步交给 MPC。

### 推荐的第一版 MPC

先从较简化的 predictive model 入手：

- centroidal model
- 或 single-rigid-body predictive model

即便长期目标会比这更复杂，也建议先这样做。

原因：

- 更容易和当前 baseline 接轨
- 开发复杂度可控
- 仍然能明显强于当前启发式力分配

### MPC 理想输出

MPC 最好能输出：

- 未来时域内的 body trajectory
- 未来时域内的 contact force
- 未来时域内的姿态/角动量目标

### 完成标准

- 求解器能在控制周期内稳定运行
- 低速 walking 比 baseline 更稳
- contact force 输出更平滑
- 轨迹跟踪质量优于 baseline

## Phase 5：引入 WBC

### 目标

用真正的全身控制替换当前简化 torque 生成逻辑。

### 为什么需要 WBC

MPC 通常给的是较高层目标，例如：

- body motion target
- contact force target

而 WBC 负责把这些目标真正落到关节层，同时满足：

- 接触约束
- swing foot 跟踪
- 姿态任务
- posture 任务
- 关节/力矩/摩擦约束

### 未来代码方向

应当新增专门的 whole-body control 模块，而不是继续在 `StateTrotting.cpp` 里直接生成所有 torque。

可考虑未来新增：

- `control/WholeBodyController.*`

### WBC 未来要处理的典型任务

- base orientation task
- base height task
- base velocity task
- swing foot position/velocity task
- posture task
- contact consistency constraints

### 完成标准

- stance 支撑能力明显优于 baseline
- swing 轨迹跟踪更干净
- torque 输出更平滑、更物理一致
- contact 切换时冲击更小

## Phase 6：强化动力学模型层

### 目标

让底层模型接口足以支撑真正的浮基优化控制。

### 当前限制

当前模型能力足以支撑 baseline torque mapping，
但不足以优雅支撑研究级 WBC。

### 未来方向

未来模型层最好能更方便地提供：

- full floating-base dynamics
- mass matrix
- nonlinear terms
- contact Jacobians
- task Jacobians

这一阶段如果有需要，可以考虑接入更强的刚体动力学后端。

### 完成标准

- controller 不再只依赖最小化的临时动力学接口
- WBC 和 MPC 可以稳定消费模型信息

## 6. 迁移过程中，哪些东西要尽量保持稳定

### 稳定的外部控制壳子

下面这些尽量不要轻易动坏：

- `UnitreeGuideController`
- ROS 2 lifecycle 行为
- command/state interface 绑定

### 稳定的测试路径

尽量一直保持可用：

- 当前 Gazebo launch
- 当前 keyboard input
- 当前仿真 robot description

### 稳定的估计器/控制器输出接口

即使估计器内部升级，也尽量保持上层调用风格稳定，例如：

- base position
- base velocity
- rotation
- yaw

这样可以减少系统级联改动带来的混乱。

## 7. 现在不值得过度优化的事情

当前阶段，不建议花太多时间的事情包括：

- 无限细调当前 trotting gains
- 反复改 zero-command hacks
- 反复做只影响状态机的临时补丁
- 试图把当前启发式控制直接修成最终控制器

这些工作只有在下面情况下才值得继续做：

- 帮助我们看清失败机理
- 帮助我们建立更好的日志
- 帮助我们维持 baseline 可运行

否则就属于低价值投入。

## 8. 建议补知识的顺序

这是推荐的学习优先级。

### Level 1：必须先打牢的基础

- 刚体运动学
- 雅可比矩阵
- 逆运动学
- 浮基动力学
- 质心动力学
- 接触约束

### Level 2：状态估计

- IMU 融合
- EKF / Error-State EKF
- Invariant EKF
- 接触辅助状态估计
- 腿式机器人中的可观性

### Level 3：优化与 MPC

- LQR
- 线性 MPC
- 非线性 MPC 基础
- QP / SQP / NLP
- 滚动时域优化
- 实时求解器约束

### Level 4：Whole-Body Control

- operational space control
- task-space control
- null-space projection
- 层级控制
- QP-based WBC
- 接触一致逆动力学

### Level 5：腿式机器人 locomotion

- gait scheduling
- foothold planning
- swing trajectory generation
- disturbance recovery
- 接触切换下的混合系统动力学

## 9. 现在最推荐的实际下一步

当前最推荐的动作是：

1. 保持 baseline 分支始终可跑。
2. 建立 Gazebo 中 truth-vs-estimate 的误差评估工具。
3. 建立 standing 和低速 walking 的标准测试集。
4. 明确第一个真正替换的是：
   - 估计器，还是
   - 预测控制器。
5. 在
   - `src/quadruped_ros2_control`
   里建立独立实验分支。

## 10. 最终原则

当前这套框架真正有价值的地方是：

- 已经有完整 ROS 2 控制链路
- 已经有可运行的仿真链路
- 已经有新模块可以接入的位置

它的价值并不在于：

- 当前 trotting 算法已经很接近高质量 locomotion 控制器

因此，最正确的策略是：

- 保留框架
- 保留可复现性
- 保留可观测性
- 分阶段替换薄弱控制核心

这才是从当前 baseline 走向更强 MPC + WBC 四足控制架构的最短路径。

## 11. 第一阶段实操清单：先把“看清楚问题”这件事做好

这一阶段不追求换控制器，而是先把实验能力搭起来。

### 11.1 先做统一日志

建议至少统一记录下面三类信息：

#### A. 估计器输出

建议记录：

- 时间戳
- base position estimate
- base velocity estimate
- base orientation estimate
- IMU quaternion
- gyro
- acc
- contact state

当前最直接的入口文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`

#### B. 控制器内部量

建议记录：

- `pcd_`
- `pos_body_`
- `vel_target_`
- `vel_body_`
- `yaw_cmd_`
- `dd_pcd`
- `d_wbd`
- 每条腿的 torque
- `wave_status`

当前最直接的入口文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/BalanceCtrl.cpp`

#### C. Gazebo ground truth

建议记录：

- base 世界坐标位置
- base 世界坐标速度
- base 姿态
- base 角速度

这部分可以后面用脚本从 Gazebo / ROS topic 中提取。

### 11.2 先不要追求“日志特别漂亮”

第一步目标不是做一个华丽的数据平台，而是先满足三点：

- 能稳定采到
- 能多次复现实验
- 能导出后对比

只要这三点先成立，后面再做更高级的数据整理都不迟。

## 12. 第一阶段标准测试项

后续所有估计器和控制器升级，都建议固定跑同一组测试。

### 12.1 Standing 测试

至少做两组：

- 原地站立 30s
- 原地站立 60s

观察点：

- 位置漂移
- 高度漂移
- roll/pitch/yaw 漂移
- 是否出现慢性累积偏差

### 12.2 低速移动测试

建议做：

- 极小前进速度
- 极小侧移速度
- 极小转向速度

观察点：

- 起步是否平顺
- 停止后是否能重新稳住
- 估计误差是否明显放大

### 12.3 起停测试

建议做：

- 从静止到低速再回到静止

观察点：

- 起步瞬间有没有明显冲击
- 停下后是否持续漂移
- 接触切换时系统有没有突然发散

### 12.4 扰动测试

建议做：

- 仿真中对 base 加一个侧向或前向小推力

观察点：

- 恢复时间
- 恢复过程中姿态误差峰值
- 是否直接失稳

## 13. 第一阶段建议优先画的图

如果后续要系统化做实验，建议先把这些图画出来。

### 13.1 估计器误差图

建议优先画：

- `x_est - x_truth`
- `y_est - y_truth`
- `z_est - z_truth`
- `vx_est - vx_truth`
- `vy_est - vy_truth`
- `vz_est - vz_truth`
- `yaw_est - yaw_truth`

作用：

- 看静止漂移
- 看慢走时累计误差
- 看接触切换时是否跳变

### 13.2 控制器跟踪图

建议优先画：

- `pcd_(x/y/z)` 与 `pos_body_(x/y/z)`
- `vel_target_(x/y/z)` 与 `vel_body_(x/y/z)`
- `yaw_cmd_` 与实际 yaw

作用：

- 看控制器是不是在追不合理目标
- 看 tracking error 是不是持续扩大

### 13.3 控制输出图

建议优先画：

- `dd_pcd`
- `d_wbd`
- 各关节 torque

作用：

- 看输出是否打满
- 看失稳前是否出现突刺
- 看是否存在越来越强的发散趋势

## 14. 第一阶段完成后，必须能回答的几个问题

第一阶段结束后，不一定要把狗跑稳，但一定要能回答下面这些问题：

1. 在零命令站立时，估计器是否已经漂移明显？
2. 在零命令站立时，控制器输出是否持续增长？
3. 失稳之前，先发散的是估计误差还是控制输出？
4. 步态切换时，估计器是否会发生跳变？
5. `trotting` 的问题主要是：
   - 估计器不准
   - 控制器太弱
   - 还是两者同时存在明显短板？

如果这些问题回答不清，后面直接上 MPC/WBC 只会把系统复杂度拉高，但不一定真的更好。

## 15. 第二阶段开始前的建议决策

在第一阶段日志和图表完成后，建议先做一个明确决策：

### 选项 A：先升级估计器

适合这种情况：

- truth-vs-estimate 误差已经很大
- 接触切换时估计明显跳变
- 控制器虽然弱，但估计本身已经不可信

### 选项 B：先升级控制器架构

适合这种情况：

- 估计基本还能用
- 但控制输出很明显发散
- tracking error 明显扩大
- 站立和低速运动都压不住

这一步建议明确写进实验记录里，不要只是脑子里有个大概印象。
