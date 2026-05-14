# Trotting 向 MPC/WBC 升级路线图

## 1. 当前已经确认的结论

到目前为止，我们已经可以比较明确地得出下面这些结论：

- `fixedStand` 状态下，机器人在仿真中可以稳定站立。
- `trotting` 状态下，即使不给速度命令，机器人也站不稳。
- `trotting` 状态下，一旦给速度命令，系统会更容易失稳。
- 因此，当前的主要问题已经不是 launch、键盘输入、控制器是否加载成功，或者简单的 FSM 切换条件。
- 当前的主要问题是 `trotting` 这套控制本身的质量不够高。

换句话说，现在这套 `unitree_guide_controller` 更适合被看作：

- 一个可以运行的基础框架
- 一个方便理解控制链路的教学/验证平台

而不应该被看作：

- 一个值得长期围绕参数打磨的最终运动控制器

## 2. 我们已经确认了什么

### 2.1 启动链路和控制器链路是正常的

下面这些链路已经确认打通：

- 启动文件：
  - `src/quadruped_ros2_control/controllers/unitree_guide_controller/launch/gazebo_classic.launch.py`
- 已加载控制器：
  - `leg_pd_controller`
  - `imu_sensor_broadcaster`
  - `joint_state_broadcaster`
  - `unitree_guide_controller`

所以现在的问题不是：

- `controller_manager` 没启动
- 控制器没激活
- 加载了错误的控制器

### 2.2 命令通路是通的

当前命令路径已经梳理清楚：

- `keyboard_input` 发布 `control_input`
- `UnitreeGuideController` 接收用户命令
- FSM 切换到 `trotting`
- `StateTrotting` 生成目标和力矩
- 命令通过 `leg_pd_controller` 下发到关节

所以现在的问题也不是：

- 键盘命令没有送到控制器
- 命令在中途丢了

### 2.3 当前 trotting 控制器结构上偏弱

从代码结构和日志表现来看，目前 trotting 这条链路大致是：

- 状态估计
- 简化的命令生成
- 简化的机身目标生成
- 单刚体风格的平衡控制
- 足端力分配
- 关节力矩映射

这套结构适合做：

- 基础验证
- 教学理解
- baseline 对比

但不太适合做：

- 高质量动态运动
- 抗扰恢复
- 接触切换鲁棒控制
- 高水平全身协调控制

## 3. 为什么不应该继续在当前 trotting 逻辑上过度投入

当前 trotting 代码的价值主要在于：

- 帮我们理解整个控制链路
- 帮我们建立日志和评估方法
- 帮我们确认问题边界
- 作为后续新控制器接入的外壳

当前 trotting 代码不适合作为长期优化对象的主要原因有：

- 动力学建模过于简化，主体还是单刚体思想
- 估计器较基础
- 状态机、目标生成、控制逻辑耦合过重
- 静止支撑和动态步行没有清晰分层
- 没有形成现代优化控制框架

因此，继续在这套 trotting 逻辑上做大量细碎参数调整，性价比已经不高。

## 4. 后续升级总方向

长期目标应该是：

- 更强的状态估计
- 更清晰的参考轨迹生成
- Model Predictive Control（MPC）
- Whole-Body Control（WBC）

更理想的控制架构应该是：

1. 用户命令 / 行为层
2. 步态与接触调度层
3. 参考轨迹生成层
4. MPC 负责未来时域内的机身/接触力规划
5. WBC 负责把高层目标变成关节层输出
6. 底层执行层负责发给仿真或硬件

而不是让一个 `StateTrotting.cpp` 同时承担几乎所有职责。

## 5. 现有代码里，后面主要会改哪些地方

### 5.1 状态估计器

当前关键文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`

它的重要性在于：

- 后面的所有控制都依赖 base 位姿、速度、姿态估计
- 如果估计器漂移，再强的控制器也会基于错误状态做决策

未来可能的升级方向：

- 更强的接触辅助状态估计
- Error-State EKF
- Invariant EKF
- 更稳的接触可信度建模
- 必要时融合额外传感器

### 5.2 Trotting 状态本身

当前关键文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`

它的重要性在于：

- 这里是当前 locomotion 逻辑的核心
- 目前把命令解释、目标生成、步态处理、控制输出都揉在一起了

未来的方向应该是：

- 让这一层变薄
- 更多承担“模式协调”的角色
- 不再把完整的运动控制核心硬写在这里

### 5.3 机器人模型与动力学接口

相关区域包括：

- `robot_model_`
- 足端力到关节力矩的映射
- 运动学/动力学接口

为什么重要：

- WBC 需要更完整的浮基动力学支持
- MPC/WBC 需要的模型能力明显高于当前最小化接口

未来方向：

- 引入更强的刚体动力学能力
- 提供质量矩阵、非线性项、接触雅可比、任务雅可比等

### 5.4 平衡控制/力分配模块

当前 trotting 的核心输出仍然依赖简化的 balance/force allocation 思路。

未来方向应该是：

- 用优化控制替代手工式力分配
- 从“启发式平衡”升级到“带约束最优控制”

## 6. 后续判断“变好了”的标准是什么

在替换估计器和控制器之前，必须先建立评估标准。

### 6.1 怎么评估估计器

在 Gazebo 里，我们天然有一个优势：

- 可以拿到 ground truth

所以估计器应该重点比较：

- 估计 base 位置 vs Gazebo 真值
- 估计 base 速度 vs Gazebo 真值
- 估计姿态 vs Gazebo 真值
- 接触切换时估计是否跳变

重点指标包括：

- 静止站立时的位置漂移
- 慢走时的位置/速度漂移
- yaw 漂移
- 速度估计质量
- stance/swing 切换时是否有大突变

### 6.2 怎么评估控制器

控制器不能只看“站没站住”，而应该看：

- 静态稳定性
- 机身高度保持能力
- 机身 roll/pitch 稳定性
- 速度跟踪精度
- 抗扰能力
- 力矩输出是否平滑
- 接触力是否合理

建议做这些标准测试：

- 原地站立 30s / 60s
- 很小速度命令下的前进测试
- 起步-停止测试
- 转向测试
- 横移测试
- 外力扰动测试

## 7. 推荐的开发策略

### 7.1 不要把主要精力继续放在当前 trotting 参数打磨上

仍然值得做的事情：

- 保留必要日志
- 能够复现实验现象
- 维持 baseline 可运行

不值得继续投入太多精力的事情：

- 无休止地调当前 trotting 参数
- 反复做低价值 FSM 小补丁
- 反复针对表面症状打补丁

### 7.2 把当前项目当作“过渡平台”

当前项目最合适的用途是：

- baseline 控制器
- 仿真测试平台
- 日志采集平台
- 后续新估计器/控制器的接入壳子

### 7.3 一次只替换一个大模块

更合理的顺序通常是：

1. 先建立评估工具
2. 然后升级估计器，或者升级控制器
3. 但不要两者同时大换血
4. 每做一步都和 baseline 对比

这样才能知道改进到底来自哪里。

## 8. 未来比较合理的系统结构

一个比较实际的目标架构可以是：

### 8.1 行为与命令层

- 键盘/手柄/更高层输入
- 步态模式切换
- 行为层意图管理

### 8.2 步态与接触调度层

- stance/swing 时序
- gait timing
- contact schedule

### 8.3 参考轨迹管理层

- body velocity target
- body pose target
- foothold reference
- swing foot trajectory

### 8.4 MPC 层

先从较简化的模型入手：

- centroidal model 或 single-rigid-body predictive model
- 滚动时域优化
- 规划未来接触力
- 规划未来机身运动

### 8.5 WBC 层

WBC 负责统一处理：

- base 姿态任务
- base 高度任务
- swing foot 跟踪任务
- posture 任务
- 接触一致性约束
- 关节/力矩/摩擦约束

### 8.6 底层执行层

- 把目标加速度/力矩/位置转换成底层命令
- 通过仿真或硬件接口下发

## 9. 未来应该补哪些知识

### 9.1 机器人动力学基础

必须补：

- 刚体运动学
- 雅可比矩阵
- 逆运动学
- 浮基动力学
- 质心动力学
- 接触约束建模

### 9.2 状态估计

必须补：

- IMU 融合
- EKF / Error-State EKF
- Invariant EKF
- 接触辅助估计
- 腿式机器人中的可观性问题

### 9.3 优化与 MPC

必须补：

- LQR
- 基础 MPC
- 非线性 MPC 基础
- QP / SQP / NLP
- 滚动时域优化
- 实时求解器的计算代价

### 9.4 Whole-Body Control

必须补：

- operational space control
- task-space control
- null-space 方法
- 层级控制
- QP-based WBC
- 接触一致的逆动力学

### 9.5 四足机器人专项知识

必须补：

- gait scheduling
- foothold planning
- swing foot trajectory generation
- disturbance recovery
- 接触切换下的混合动力学

## 10. 现在最值得做的下一步

当前最实用的下一步应当是：

1. 保持当前 baseline 在 Gazebo 中可运行。
2. 建立 ground truth 对比工具。
3. 建立 standing 和低速 walking 的标准日志与误差曲线。
4. 明确第一项真正替换的是：
   - 估计器，还是
   - 控制器。
5. 在 `src/quadruped_ros2_control` 仓库里用独立分支做实验。

## 11. 最核心的思维方式

当前这套代码最有价值的地方在于：

- 它已经提供了一个完整的 ROS 2 控制壳子
- 已经有仿真可运行链路
- 已经有键盘输入到关节输出的完整通道
- 已经能让我们把新模块逐步接进去

它最没必要被过度神化的地方在于：

- 当前 trotting 算法本身并不接近高质量 locomotion controller

所以最正确的策略是：

- 保留框架
- 保留可复现性
- 保留可观测性
- 逐步替换掉薄弱的控制核心

我们的目标不是把当前这份 trotting 代码修成最终产品。

我们的目标是利用这套项目，走向更强的 MPC + WBC 控制架构。

## 12. 后续最关键的文件

当前最值得重点理解和后续改造的文件有：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/FSM/StateTrotting.h`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/UnitreeGuideController.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/control/CtrlComponent.h`
- `src/quadruped_ros2_control/libraries/controller_common/include/controller_common/CtrlInterfaces.h`

这些文件未必是未来 MPC/WBC 的最终归宿，但它们一定是当前理解系统、后续植入新架构的起点。

## 13. 第一阶段最值得补齐的实验能力

如果后面要真正升级估计器、MPC 和 WBC，那么现在最值得优先补齐的，不是继续调 trotting 参数，而是下面这些实验能力。

### 13.1 统一日志能力

至少应该能稳定记录三类数据：

- 估计器输出
- 控制器内部目标与输出
- Gazebo ground truth

如果这三类数据不能放在同一时间轴上，后面很多结论都会停留在“看起来像”。

### 13.2 Ground truth 对比能力

在 Gazebo 环境里，一个非常大的优势是：

- 仿真真值是可拿到的

所以现在最值得做的事情之一，就是把：

- `Estimator.cpp` 输出的 base position / velocity / orientation

和：

- Gazebo 真值

放到一起比较。

只要这一层打通，后面很多“到底是估计炸了还是控制炸了”的问题，就能少走很多弯路。

### 13.3 标准测试场景

后续所有控制器升级，至少都应该在同一组标准场景下对比：

- 原地站立 30s
- 原地站立 60s
- 极小前向速度
- 起步-停止
- 小幅转向
- 小幅横移
- 人工施加扰动

如果没有统一测试场景，不同实验之间就很难真正可比。

## 14. 现在最应该优先看的误差曲线

后续你做实验时，最先应该盯的不是复杂优化量，而是最基础、最说明问题的量。

### 14.1 估计器侧

优先看这些：

- `position_est - position_truth`
- `velocity_est - velocity_truth`
- `yaw_est - yaw_truth`
- `roll/pitch_est - roll/pitch_truth`
- contact state 与误差变化的对应关系

你真正想回答的是：

- 静止时估计有没有慢漂
- 步态切换时估计有没有跳
- 失稳前估计是否已经明显发散

### 14.2 控制器侧

优先看这些：

- `pcd_ - pos_body_`
- `vel_target_ - vel_body_`
- `yaw_cmd_ - yaw_actual`
- `dd_pcd`
- `d_wbd`
- joint torque

你真正想回答的是：

- 控制器是不是一直在追一个不合理的目标
- 控制器输出是不是越来越大
- 失稳前是姿态控制先发散，还是位置控制先发散

## 15. 关于“这套框架值不值得继续改”的最终判断

现在对这个项目的正确态度应该是：

- 值得继续“用”
- 不值得继续“深修”

“值得继续用”的意思是：

- 它适合作为后续估计器、MPC、WBC 的接入平台
- 它适合作为 baseline
- 它适合作为实验验证环境

“不值得继续深修”的意思是：

- 不应该把大量时间继续耗在当前简化 trotting 控制细节上
- 不应该指望靠少量参数优化把它变成高性能 locomotion controller

所以后面的工作重点应该逐渐转向：

- 建立评估体系
- 升级估计器
- 重构控制架构
- 引入 MPC
- 引入 WBC
