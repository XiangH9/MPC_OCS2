# Unitree Guide Controller NMPC Modification Plan

这份文档用于给后续开发者或新的 AI 直接交接当前 `unitree_guide_controller` 的 NMPC 改造进度、设计约束和后续计划。

工作目录：

- 控制器：
  - `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller`
- 主要参考仓库：
  - `/home/xiangh9/xhros2/MPC/MPC_legged_control`
- 仅作为 ROS2 接线思路参考，不直接依赖：
  - `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller`

---

## 1. 当前总体策略

当前已经明确采用以下策略，请后续继续保持：

### 1.1 不直接依赖其他 controller 包

可以参考：

- `MPC_legged_control`
- `ocs2_quadruped_controller`

但**不要**直接在 `unitree_guide_controller` 中 include 其他 controller 包的头文件，尤其不要形成这种编译依赖：

- `#include <ocs2_quadruped_controller/...>`

目标是让 `unitree_guide_controller` 最终具备更好的可移植性，不要求使用者额外下载整个其他控制器仓库。

### 1.2 先做 MPC，后做 WBC

当前阶段的重点是：

- `LeggedInterface`
- 配置读取
- 状态准备
- MPC runtime 入口
- 后续的 `MPC / MRT / policy evaluation`

当前阶段明确**不优先处理**：

- `WBC`
- 全身力控制
- torque 级完整闭环

也就是说，路线已经切换为：

**MPC first, WBC later**

补充说明：

- 当前已经不是“只打通 solver 壳子”的阶段；
- 当前 `unitree_guide_controller` 已经具备：
  - `setCurrentObservation()`
  - 后台 `advanceMpc()`
  - 前台 `updatePolicy()`
  - 前台 `evaluatePolicy()`
  - 基于 `planned_mode` 的执行相位切换
- 但当前仍然**没有**参考仓库那种完整 `WBC` 闭环，因此不能简单把参考仓库中 “`optimizedState / optimizedInput` -> WBC -> hybrid command” 的最终执行方式直接照搬过来。

### 1.3 每一步都要求可验证

每一步修改都遵循：

- 先搭一层最小骨架
- 编译通过
- 启动通过
- 日志可验证
- 再进入下一层

不要一次性大规模迁移整套 OCS2 / WBC 逻辑。

### 1.4 参考实现优先用于控制逻辑对标

后续分析和改造 `unitree_guide_controller` 的 NMPC 时，必须优先对照参考仓库：

- `/home/xiangh9/xhros2/MPC/MPC_legged_control`

工作原则：

- 不直接复制整个参考工程，也不直接引入其 controller 包编译依赖；
- 但凡涉及控制主链设计、执行层结构、状态估计链、NMPC 输出解释方式、力分配/WBC 职责边界，必须先对照参考实现的运行逻辑，再判断当前自有实现缺了什么；
- 不能把“日志已经显示 `advanceMpc()` / `evaluatePolicy()` 成功”直接等同于“机器人已经具备稳定站立能力”；
- 必须区分：
  - solver 主链是否打通；
  - reference trajectory 是否正确；
  - `optimizedState / optimizedInput` 是否被正确解释；
  - 执行层是否具备足够的 torque / hybrid 支撑能力；
- 当 `unitree_guide_controller` 与 `MPC_legged_control` 的行为差异会影响站立稳定性时，应优先把这种差异记录到任务文档，再决定修改顺序。

当前已确认的重要参考结论：

- `MPC_legged_control` 的控制逻辑不是“NMPC 直接输出 joint velocity 后下发即可稳定站立”；
- 它采用的是：
  - 状态估计；
  - NMPC 求解 `optimizedState / optimizedInput`；
  - WBC/力分配；
  - 最终下发 `posDes + velDes + torque(ff)` 的 hybrid 命令；
- 因此，后续分析 `unitree_guide_controller` 时，必须默认检查：
  - 是否缺少对 `optimizedState` 的使用；
  - 是否缺少对 `optimizedInput` 中接触力部分的执行层解释；
  - 是否缺少 WBC / 力矩前馈 / 约束一致化；
  - 是否只是做了“最小求解链打通”，但尚未具备“稳定执行链”。

### 1.5 当前已经确定的执行主逻辑

截至 `2026-05-12 13:48:49 CST`，当前应继续保持以下主逻辑，不要再回退到旧路线：

- `CtrlComponent / OCS2` 负责：
  - 当前观测更新；
  - 后台 `advanceMpc()`；
  - 前台 `updatePolicy() + evaluatePolicy()`；
  - 输出 `planned_mode`；
- `StateNMPC` 负责：
  - 按 `planned_mode` 解析 `solver_contact` 与 `leg_roles`；
  - `stance` 腿执行 `support torque + NMPC stance hold`；
  - `swing` 腿执行 `reference foot target -> IK -> joint tracking`；
- `UnifiedGaitScheduler` 当前只作为：
  - 足端 reference 提供者；
  - 局部时序/相位参考；
  - 不再是 OCS2 mode 的输入真源。

明确禁止继续恢复以下旧路线：

- 前台每拍手工向 OCS2 `ReferenceManager` 写入 mode schedule；
- 运行时 `resetMpcNode()` 式 recovery 主链；
- 让 `UnifiedGaitScheduler` 直接主导 NMPC mode / execution mode；
- 在当前没有完整 WBC 的前提下，让 `swing` 腿直接跟裸 `NMPC joint q/qd`。

### 1.6 当前已确认的 OCS2 主链约束

基于本地源码与运行日志，当前已经确认：

- 正确主链必须是：
  - `setCurrentObservation()`
  - `advanceMpc()`
  - `updatePolicy()`
  - `evaluatePolicy()`
- 旧 policy 一旦过期，而后台又没续出新 policy，就会落入 OCS2 标准失败路径；
- 因此：
  - 不要再设计额外 runtime recovery 去掩盖这个问题；
  - 优先保证后台线程稳定、fresh policy 持续进入、`planned_mode` 正常推进。

---

## 2. 当前已完成进度

下面是截至当前已经完成并验证通过的内容。

### 2.1 FSM 已接入 `StateNMPC`

已完成：

- 在 `controller_common/common/enumClass.h` 中加入 `FSMStateName::NMPC`
- 新增：
  - `include/unitree_guide_controller/FSM/StateNMPC.h`
  - `src/FSM/StateNMPC.cpp`
- 在 `UnitreeGuideController` 中注册 `StateNMPC`
- 在部分已有状态中增加跳转逻辑

当前已验证：

- 可以正常切换到 `StateNMPC`
- 日志中已看到：
  - `Switched from fixed stand to nmpc`

当前状态：

- `StateNMPC` 已经是真实 FSM 状态，不是概念占位
- 当前已能真实进入 NMPC runtime，并驱动后续 `MPC / MRT / policy evaluation`

### 2.2 `CtrlComponent` 已升级为 NMPC 宿主容器

已完成：

- `CtrlComponent` 从轻量结构升级为类
- 已接入：
  - `setupNmpcConfig(...)`
  - `setupLeggedInterface()`
  - `isNmpcInterfaceReady()`
  - `setupMpcObjects()`
  - `setupMpcRuntime()`
  - `isMpcRuntimeReady()`

当前 `CtrlComponent` 已经管理的内容包括：

- 现有 guide 链相关对象：
  - `robot_model_`
  - `estimator_`
  - `balance_ctrl_`
  - `wave_generator_`
- NMPC 配置路径：
  - `robot_pkg_`
  - `urdf_file_`
  - `task_file_`
  - `reference_file_`
  - `gait_file_`
- joint / foot names
- MPC runtime 占位状态：
  - `mpc_time_`
  - `mpc_state_`
  - `mpc_input_`
  - `mpc_mode_`
  - `measured_rbd_state_`
  - `optimized_state_`
  - `optimized_input_`
  - `planned_mode_`
  - `policy_available_`
  - `mpc_objects_initialized_`
  - `mpc_runtime_ready_`
  - `mpc_running_`
  - `controller_running_`
  - `mpc_thread_`
  - `observation_`
  - `rbd_conversions_`

注意：

- 当前 runtime 字段已不再只是壳子
- 已成功创建：
  - `SqpMpc`
  - `MPC_MRT_Interface`
- 已能进行：
  - `setCurrentObservation`
  - `advanceMpc()`
  - `updatePolicy()`
  - `evaluatePolicy(...)`

### 2.3 `robot_pkg` 配置已经接通

已完成：

- 在 `go1_description/config/gazebo.yaml` 中为 `unitree_guide_controller` 增加：
  - `robot_pkg: "go1_description"`

因此当前 `CtrlComponent::setupNmpcConfig()` 已能正确生成：

- `urdf_file_`
- `task_file_`
- `reference_file_`
- `gait_file_`

当前已验证：

- 启动时相关路径日志能正确输出

### 2.4 已建立自有 `nmpc::LeggedInterface`

已完成：

- 新建：
  - `include/unitree_guide_controller/nmpc/LeggedInterface.h`
  - `src/nmpc/LeggedInterface.cpp`

这个类现在是**自有接口层入口**，不是对其他 controller 包的直接引用。

当前 `LeggedInterface` 已具备：

- 保存 `task_file / urdf_file / reference_file`
- 保存 `joint_names / foot_names`
- 基础初始化
- 文件存在性检查
- 读取 `.info` 配置
- 检查关键 section 是否存在
- 提供 `BasicNmpcConfig`

当前重点配置项检查包括：

- task.info:
  - `legged_robot_interface.verbose`
  - `model_settings`
  - `mpc`
  - `sqp`
  - `rollout`
  - `swing_trajectory_config`
  - `initialState`
- reference.info:
  - `initialModeSchedule`
  - `defaultModeSequenceTemplate`

当前已验证：

- `LeggedInterface` 初始化成功日志已出现
- `StateNMPC` 已能读取 `getBasicConfig()`

### 2.5 `StateNMPC` 已联通 `LeggedInterface`

已完成：

- `StateNMPC::enter()` 已能：
  - 检查 `isNmpcInterfaceReady()`
  - 读取 `legged_interface_->getBasicConfig()`
  - 打印配置摘要
  - 调用 `setupMpcRuntime()`
  - 根据 runtime 状态打印当前阶段

当前已形成的链路：

`FSM -> StateNMPC::enter() -> CtrlComponent -> LeggedInterface -> setupMpcRuntime()`

这条链路已经真实存在并可验证。

当前补充状态：

- `StateNMPC::run()` 已能低频调用：
  - `updateMeasuredRbdStateFromEstimator()`
  - `advanceMpcOnce()`
  - `evaluateCurrentPolicy()`
- 当前日志已能看到：
  - `MPC runtime is ready.`
  - `advanceMpcOnce() succeeded.`
  - `Latest NMPC policy cached successfully.`

### 2.5.1 `advanceMpcOnce()` 已不再代表当前真实实现

这是一条过时描述，仅保留作早期历史记录。

当前真实实现已经切换为更接近参考仓库的结构：

- 后台线程持续调用 `advanceMpc()`；
- 前台控制周期中调用：
  - `pushCurrentObservationToMpc()`
  - `evaluateCurrentPolicy()`
- 当前不再以“前台单步 `advanceMpcOnce()` 成功”作为运行主逻辑。

### 2.6 OCS2 本地环境已安装并完成最小兼容修补

已完成：

- 采用 `manumerous/ocs2_ros2` 作为当前本地 OCS2 来源
- 源码已放入：
  - `/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2`
- 已在当前工作区完成并验证以下关键包：
  - `ocs2_core`
  - `ocs2_mpc`
  - `ocs2_sqp`
  - `ocs2_pinocchio_interface`
  - `ocs2_centroidal_model`
  - `ocs2_legged_robot`
- 已验证本地存在：
  - `install/ocs2_core/share/ocs2_core/cmake/ocs2_coreConfig.cmake`
  - `install/ocs2_legged_robot/share/ocs2_legged_robot/cmake/ocs2_legged_robotConfig.cmake`
- 已验证 `unitree_guide_controller` 可以在当前工作区成功编译通过

这意味着最早阻塞 `unitree_guide_controller` 的问题：

- `find_package(ocs2_core)` 失败

已经解决。

### 2.6.1 本次安装中补齐的关键依赖

通过 `rosdep` 和系统包安装，已补齐或确认可用：

- `ros-humble-pinocchio`
- `ros-humble-hpp-fcl`
- `ros-humble-grid-map-ros`
- `libglpk-dev`

此前系统中已存在并确认可用：

- `libeigen3-dev`
- `libboost-all-dev`
- `liburdfdom-dev`
- `liburdfdom-headers-dev`
- `liboctomap-dev`
- `libassimp-dev`
- `libtinyxml2-dev`
- `libgoogle-glog-dev`
- `libgflags-dev`

### 2.6.2 为使 `ocs2_legged_robot` 在 ROS2 Humble 下可用而做的最小兼容修改

当前 `manumerous/ocs2_ros2` 中的 `ocs2_legged_robot` 仍保留部分 ROS1 / 旧接口残留，因此做了最小兼容修补。

1. `ocs2_legged_robot/CMakeLists.txt`

文件：

- `/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_robotic_examples/ocs2_legged_robot/CMakeLists.txt`

处理内容：

- 注释掉了基于 ROS1 `catkin_EXPORTED_TARGETS` 的这段依赖声明：

```cmake
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
```

原因：

- 当前为 ROS2 / ament 环境，`catkin_EXPORTED_TARGETS` 为空；
- 不注释会导致：
  - `add_dependencies called with incorrect number of arguments`

2. `SwitchedModelReferenceManager` 接口签名适配

文件：

- `include/ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h`
- `src/reference_manager/SwitchedModelReferenceManager.cpp`

处理内容：

- 将 `modifyReferences(...)` 从旧签名：
  - `(initTime, finalTime, initState, targetTrajectories, modeSchedule)`
- 适配为新签名：
  - `(initTime, finalTime, initState, initMode, targetTrajectories, modeSchedule)`

原因：

- 当前 `ocs2_oc::ReferenceManager` 基类接口已经新增 `size_t initMode` 参数；
- 不适配会导致：
  - `marked override, but does not override`

3. 暂时移除 `ocs2_ipm` 相关依赖

文件：

- `include/ocs2_legged_robot/LeggedRobotInterface.h`
- `src/LeggedRobotInterface.cpp`

处理内容：

- 删除 `#include <ocs2_ipm/IpmSettings.h>`
- 删除 `ipm::Settings ipmSettings_`
- 删除 `ipmSettings()` getter
- 删除 `ipm::loadSettings(...)`

原因：

- 当前工作区中的 `ocs2_ipm` 仍是 ROS1 `catkin` 风格包；
- 它不会生成 ROS2 下可被 `find_package(ocs2_ipm)` 发现的配置文件；
- 当前 `unitree_guide_controller` 接入路线优先使用 `SqpMpc`，因此先裁掉 `ipm` 依赖，保证最小可用链路跑通。

### 2.6.3 为使 `unitree_guide_controller` 接入当前 OCS2 版本而做的兼容修改

1. 修正 OCS2 头文件大小写

文件：

- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/control/CtrlComponent.h`

处理内容：

- 将：
  - `#include <ocs2_mpc/MPC_MRT_interface.h>`
- 修正为：
  - `#include <ocs2_mpc/MPC_MRT_Interface.h>`

原因：

- 当前 OCS2 仓库中的真实头文件名为 `MPC_MRT_Interface.h`；
- Linux 文件系统区分大小写，旧写法会直接导致找不到头文件。

2. 显式补充 Boost 依赖

文件：

- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/CMakeLists.txt`

处理内容：

- 增加：
  - `find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)`
- 并对 `unitree_guide_controller` 显式链接：
  - `Boost::system`
  - `Boost::filesystem`
  - `Boost::log`
  - `Boost::log_setup`

原因：

- 当前 OCS2 依赖链会向上传递 Boost imported targets；
- 若本包未显式 `find_package(Boost ...)`，CMake 生成阶段会报：
  - `Target ... links to target Boost::system but the target was not found`

### 2.6.4 当前验证结论

已完成验证：

- `ocs2_legged_robot` 编译通过
- `unitree_guide_controller` 编译通过

这说明当前工作区已经具备以下条件：

- `unitree_guide_controller` 可在编译层面接入 OCS2 基础能力
- `StateNMPC -> CtrlComponent -> LeggedInterface -> OCS2 package dependency` 这条链路的底层依赖已不再缺失

### 2.6.5 当前仍需注意的点

1. 本次兼容修改的目标是先让当前 NMPC 开发链路在 ROS2 Humble 下最小可用，不是完整清理整个 `ocs2_ros2` 仓库中的所有 ROS1 遗留问题。

2. `ocs2_ipm` 当前未纳入可用链路。
当前控制器接入路线优先使用：

- `ocs2_sqp`
- `SqpMpc`

后续如确实需要 `ipm` 路线，再单独处理 `ocs2_ipm` 的 ROS2 化。

3. 当前构建环境中仍会出现与 conda/虚拟环境相关的 runtime search path 警告，例如 `libgomp.so.1` 潜在冲突。
这些警告当前未阻塞编译，但后续若出现运行时动态库异常，应优先在干净 shell 中复现并排查。

### 2.7 自有 `LeggedInterface` 已升级为 settings 桥接层

已完成：

- `unitree_guide_controller::nmpc::LeggedInterface` 不再只是配置 section 检查壳子
- 当前已能真实加载并持有：
  - `ocs2::legged_robot::ModelSettings`
  - `ocs2::mpc::Settings`
  - `ocs2::sqp::Settings`
  - `ocs2::rollout::Settings`
- 已在自有接口层中将控制器提供的：
  - `joint_names_`
  - `foot_names_`
  注入到 `model_settings_`

当前已验证日志包括：

- `model_settings jointNames size: 12`
- `model_settings contactNames3DoF size: 4`
- `OCS2 settings loaded successfully.`
- `LeggedInterface initialized successfully.`

这一步很重要，因为它验证了当前路线应继续坚持：

- 保留自有 `unitree_guide_controller::nmpc::LeggedInterface`
- 逐步在自有接口层中完成 OCS2 所需对象构建
- 只参考 `ocs2_quadruped_controller` 的实现思路
- 不直接依赖其头文件和编译接口

### 2.7.1 已验证过但当前明确放弃的路线

当前已经验证过一条不可继续推进的路线：

- 在自有 `LeggedInterface::initialize()` 中直接构造
  - `ocs2::legged_robot::LeggedRobotInterface`

运行结果表明，这条路线在当前架构下不合适。

实际暴露的问题是：

- 原版 `ocs2::legged_robot::LeggedRobotInterface` 在建模阶段依赖 `model_settings_.jointNames` 与 `contactNames3DoF`
- 而这些名称并不是单靠 `task.info` 就能完整提供
- 参考实现 `ocs2_quadruped_controller::LeggedInterface` 的做法是：
  - 先 `loadModelSettings(...)`
  - 再通过代码把 `joint_names` 和 `foot_names` 注入 `model_settings_`
- 直接构造原版 `LeggedRobotInterface` 会绕开这一步
- 运行时会导致模型关节自由度为零，最终出现：
  - `Loading empty matrix "defaultJointState" is not allowed.`

因此，当前已经明确：

- 不继续直接桥接原版 `ocs2::legged_robot::LeggedRobotInterface`
- 改为在自有接口层内部逐步构建：
  - `PinocchioInterface`
  - `CentroidalModelInfo`
  - `initialState`
  - `ReferenceManager`
  - `OptimalControlProblem`
  - `Initializer`
  - `SqpMpc`

### 2.7.2 当前运行时验证结论（旧阶段）

以下内容是此前“solver 尚未接入”阶段的旧验证结论，仅保留作历史记录。

当时在仿真中已重新验证：

- 控制器配置阶段：
  - 自有 `LeggedInterface` 能成功初始化
  - OCS2 settings 桥接层能正常工作
- 状态切换阶段：
  - `Switched from fixed stand to nmpc`
  - `StateNMPC enter`

### 2.8 当前主链运行状态（新阶段）

这是当前最重要的最新结论。

已确认：

- MPC 后台线程可以持续产出 fresh policy；
- 前台可以持续：
  - `updatePolicy()`
  - `evaluatePolicy()`
- `policyFinalTime` 能持续往前推进；
- `reuse_count` 保持在较小范围；
- `planned_mode` 能正常在 `15 / 6 / 9` 等模式间切换；
- 不再出现此前那种“policy 过期后永久卡死”的主链失效问题。

当前因此已经确认：

- `MPC / MRT / evaluatePolicy` 主链已经基本打通；
- 后续重点不再是 solver 主链，而是执行层质量与稳定性。

### 2.9 当前已完成的执行层方向修正

截至当前，已经完成并应继续保留的执行层修正如下：

1. `solver mode` 主导执行相位

- `StateNMPC` 中已由 `planned_mode -> solver_contact -> leg_roles`
- `stance / swing` 划分不再由 `UnifiedGaitScheduler` 直接决定

2. 不再向 OCS2 手工灌 mode schedule

- 已移除前台把 `UnifiedGaitScheduler` 的 mode schedule 写入 `ReferenceManager` 的路径
- 这条路线已确认不符合当前参考实现主链

3. 停用 runtime recovery 主链

- 已移除/停用：
  - `recoverExpiredMpcPolicy()`
  - `buildSafeRecoveryModeSchedule()`
  - `resetMpcNode()` 式恢复思路

4. `support target` 的 `x / y / yaw` 已改为当前测量 + 短时预瞄

- 这一步已经通过日志验证有效；
- 主要收益是：
  - `target_py` 不再持续积分漂移；
  - `pos_error_y` 明显减小；
  - `yaw target` 与测量值更一致。

5. `support target` 的 `z / roll / pitch` 当前策略

- 经过实验，手工向 `z / roll / pitch` 注入 nominal 恢复偏置会恶化控制；
- 当前已回退为：
  - `target_pz = current_pz`
  - `target_roll = current_roll`
  - `target_pitch = current_pitch`
- 当前默认原则是：
  - 在没有完整 WBC 的前提下，不再额外手工制造 `z / roll / pitch` 恢复器

6. `swing` 腿执行链当前结论

- 已验证“直接让 swing 腿跟裸 `NMPC joint q/qd`”会明显恶化行走效果；
- 当前已回退回：
  - `reference foot target -> IK -> joint tracking`
- 当前保留的正确分工是：
  - `stance`：`support torque + NMPC stance hold`
  - `swing`：`reference IK`

### 2.10 当前最重要的运行结论

当前已经确认两件事：

1. 主链对了

- solver 主链、线程、policy 生命周期、mode source 已基本摆正

2. 行走还不够好，但问题已经收缩到执行层

当前主要矛盾已经不是：

- `advanceMpc()` 是否跑
- `updatePolicy()` 是否成功
- `mode schedule` 是否写对

而是：

- `support torque`
- `stance hold`
- `swing reference IK`
- 三者之间的执行层协调是否足够稳定
  - `NMPC interface is ready`
- runtime 阶段：
  - `setupMpcRuntime()` 已再次回到一致的 shell 状态
  - 当前日志为：
    - `MPC runtime shell prepared.`
    - `Real SQP MPC creation will be connected after self-owned OCS2 model setup is ready.`

这说明当时系统处于一个新的、更高质量的稳定中间状态：

- 编译通过
- 启动通过
- 自有接口层 ready
- `StateNMPC` ready
- 真实 solver 尚未接入，但当时已不再处于错误路线中

### 2.8 自有 `LeggedInterface` 已完成最小建模对象准备

已完成：

- 在自有 `unitree_guide_controller::nmpc::LeggedInterface` 中新增并成功构建：
  - `ocs2::PinocchioInterface`
  - `ocs2::CentroidalModelInfo`
  - `initialState`
- 当前这些对象已经不再只是“头文件占位”，而是运行时真实创建成功

实现方式：

- 继续沿用自有接口层路线，不再直接构造原版 `ocs2::legged_robot::LeggedRobotInterface`
- 在自有 `LeggedInterface` 中：
  - 先加载 `ModelSettings / mpc::Settings / sqp::Settings / rollout::Settings`
  - 再注入：
    - `model_settings_.jointNames = joint_names_`
    - `model_settings_.contactNames3DoF = foot_names_`
  - 然后调用：
    - `centroidal_model::createPinocchioInterface(...)`
    - `centroidal_model::createCentroidalModelInfo(...)`
    - `loadEigenMatrix(task_file_, "initialState", initial_state_)`

本阶段补充了运行时 getter：

- `getPinocchioInterface()`
- `getCentroidalModelInfo()`
- `getInitialState()`

### 2.8.1 当前已验证日志

当前在仿真中已验证可以稳定看到：

- `PinocchioInterface created successfully.`
- `CentroidalModelInfo stateDim: 24`
- `CentroidalModelInfo inputDim: 24`
- `initialState size: 24`
- `LeggedInterface initialized successfully.`

这说明：

- Go1 当前 OCS2 质心模型维度已经真实建立完成
- 当前自有接口层已具备进入下一阶段的最小建模前提
- 后续再接 solver 时，不需要再回头解决：
  - `defaultJointState`
  - `jointNames`
  - `contactNames3DoF`
  - `initialState`
  这些基础建模问题

### 2.8.2 本阶段踩过并已解决的问题

1. `defaultJointState` 报空矩阵

实际原因不是 `reference.info` 没写值，而是：

- 原先尝试直接构造原版 `ocs2::legged_robot::LeggedRobotInterface`
- 该路径没有在当前架构下正确注入 `jointNames / contactNames3DoF`
- 导致 Pinocchio 模型关节自由度错误
- 最终 `defaultJointState` 的目标向量长度为 0

该问题已通过“回到自有接口层并在 settings 后手动注入 joint/foot names”路线解决。

2. `PinocchioInterface::getModel().nq` 编译报 incomplete type

原因：

- 在 `.cpp` 中访问了 `getModel().nq`
- 但没有包含 `pinocchio::ModelTpl` 的完整定义头

处理方式：

- 在 `src/nmpc/LeggedInterface.cpp` 中补：
  - `#include <pinocchio/multibody/model.hpp>`

该问题已解决。

### 2.9 自有 `LeggedInterface` 已接入 `ReferenceManager / Initializer / OptimalControlProblem / Rollout`

已完成：

- 在自有 `unitree_guide_controller::nmpc::LeggedInterface` 中新增并成功构建：
  - `ocs2::legged_robot::SwitchedModelReferenceManager`
  - `ocs2::legged_robot::LeggedRobotInitializer`
  - `ocs2::OptimalControlProblem`
  - `ocs2::TimeTriggeredRollout`
- 已新增对应 getter / readiness 接口：
  - `hasReferenceManager()`
  - `hasInitializer()`
  - `hasOptimalControlProblem()`
  - `hasRollout()`
  - `getReferenceManagerPtr()`
  - `getInitializer()`
  - `getOptimalControlProblem()`
  - `getRollout()`

实现路线：

- 继续坚持自有接口层路线
- 使用 `reference.info` 中的：
  - `initialModeSchedule`
  - `defaultModeSequenceTemplate`
  构造 `GaitSchedule`
- 使用 `task.info` 中的：
  - `swing_trajectory_config`
  构造 `SwingTrajectoryPlanner`
- 再组装：
  - `SwitchedModelReferenceManager`
  - `LeggedRobotInitializer`
  - 最小 `OptimalControlProblem` 骨架
  - `TimeTriggeredRollout`

当前验证结论：

- 上述对象都已在 `LeggedInterface::initialize()` 中成功创建
- 当前已不再停留在“solver 前壳子”阶段

### 2.10 `CtrlComponent` 已成功创建真实 `SqpMpc / MPC_MRT_Interface`

已完成：

- `CtrlComponent::setupMpcRuntime()` 不再只是 runtime shell
- 当前已真实创建：
  - `ocs2::SqpMpc`
  - `ocs2::MPC_MRT_Interface`
- 已将 `LeggedInterface` 中准备好的：
  - `MpcSettings`
  - `SqpSettings`
  - `OptimalControlProblem`
  - `Initializer`
  - `ReferenceManager`
  接入 runtime

当前已验证日志包括：

- `SqpMpc created successfully.`
- `MPC_MRT_Interface created successfully.`
- `MPC runtime is ready.`
- `StateNMPC] MPC runtime is ready.`

注意：

- 当前运行中仍可能看到：
  - `Failed to set threads priority ...`
- 这属于线程优先级权限警告，不阻塞当前 NMPC 主链验证

### 2.11 已打通最小 policy 级运行链路

已完成：

- 当前 `CtrlComponent` 已具备：
  - `advanceMpcOnce()`
  - `evaluateCurrentPolicy()`
  - `hasLatestPolicy()`
- 当前已成功按顺序执行：
  - `setCurrentObservation(...)`
  - `advanceMpc()`
  - `updatePolicy()`
  - `evaluatePolicy(...)`
- 当前已能读取并缓存：
  - `optimized_state_`
  - `optimized_input_`
  - `planned_mode_`

当前已验证日志包括：

- `advanceMpcOnce() succeeded.`
- `Latest NMPC policy cached successfully.`
- `optimized state dim: 24`
- `optimized input dim: 24`
- `planned mode: 15`
- `StateNMPC] latest NMPC policy is available.`

当前观察：

- 目前多次评估中：
  - `optimized input[0]: 0`
  - `planned mode: 15`
- 这在当前“最小问题骨架 + 初始静态状态”阶段是可接受的中间结果
- 当前重点是主链打通，不是立刻追求最优轨迹质量

### 2.12 已将 estimator 的 base 状态接入 `measured_rbd_state_`

已完成：

- `CtrlComponent` 中新增：
  - `ocs2::CentroidalModelRbdConversions`
  - `updateMeasuredRbdStateFromEstimator()`
- 当前 `StateNMPC::run()` 已在低频调用中先执行：
  - `updateMeasuredRbdStateFromEstimator()`
  再执行：
  - `advanceMpcOnce()`
  - `evaluateCurrentPolicy()`

当前实现方式：

- 使用 `Estimator` 提供的：
  - `getPosition()`
  - `getRotation()`
  - `getVelocity()`
  - `getGyroGlobal()`
  构造 base 相关 `rbdState`
- 再使用：
  - `CentroidalModelRbdConversions::computeCentroidalStateFromRbdModel(...)`
  转换成 24 维 `measured_rbd_state_`

当前已验证日志包括：

- `measured_rbd_state_ updated from estimator.`
- `measured_rbd_state_ dim: 24`
- `measured_rbd_state_[0]: ...`

这说明：

- `Estimator -> RBD state -> centroidal state -> MPC observation`
  这条链路已真实存在
- 当前 `measured_rbd_state_` 已经不再总是等于 `initial_state_`

### 2.12.1 当前仍然存在的输入侧占位项

此前这里记录过“joint q/dq 仍是占位”的历史状态。该项现已完成更新，保留本节用于纠正旧交接信息。

当前最新状态：

- `joint positions`
  - 已从 `joint_position_state_interface_` 真实读取
- `joint velocities`
  - 已从 `joint_velocity_state_interface_` 真实读取

这意味着当前 `measured_rbd_state_` 已更新为：

- base 部分：真实 estimator 数据
- joint 部分：真实 state interface 数据

因此，后续最高优先级已经不再是“补 joint q/dq 接线”，而是：

- 提升 `evaluatePolicy(...)` 输出质量
- 提升执行层的可观测动作质量
- 继续完善最小执行策略，而不是回头重做基础状态接线

### 2.13 OCS2 状态结构已确认，后续不要再拍脑袋写索引

当前已确认：

- `CentroidalModelInfo.stateDim = generalizedCoordinatesNum + 6`
- 对 Go1：
  - `generalizedCoordinatesNum = 18`
  - `stateDim = 24`
- 24 维 centroidal state 的布局为：
  - `0:6` `normalized momentum`
  - `6:12` `base pose`
  - `12:24` `joint angles`

同时已确认 `CentroidalModelRbdConversions::computeCentroidalStateFromRbdModel(...)` 所需 `rbdState` 布局为：

- `0:3` base orientation (ZYX Euler)
- `3:6` base position
- `6:18` joint positions
- `18:21` base angular velocity (world frame)
- `21:24` base linear velocity (world frame)
- `24:36` joint velocities

这一步很重要，因为它明确了后续接 estimator / joint state 时的正确映射方式。

### 2.14 零输入下已实现最小稳定站立

已完成并验证：

- 当前在 `StateNMPC` 中采用阶段性控制策略：
  - 零速度/零转向命令时，优先使用 `applySupportTorque()` 作为站立稳定器；
  - 非零运动命令时，才切换到 NMPC hybrid 执行链。
- 在该策略下，仿真中已观察到：
  - `planned mode: 15` 持续稳定；
  - `optimizedState / optimizedInput` 持续稳定输出；
  - `candidate joint_pos` 与实测 joint position 基本一致；
  - base height 不再出现此前那种快速下沉；
  - support torque 已出现稳定非零输出；
  - 机器人在零输入场景下已具备“基本稳定站立”的能力。

当前日志特征包括：

- `planned mode: 15`
- `optimized state dim: 24`
- `optimized input dim: 24`
- `support target z: ...`
- `measured z: ...`
- `pos_error z` 已缩小到约 1 cm 量级
- `support torque[0]` 已不再长期接近 0

当前结论：

- 这说明当前系统已经从“进入 NMPC 后无法稳定站立”推进到“零输入下可基本稳定站立”；
- 当前阶段的稳定站立主要依赖：
  - NMPC 后端稳定求解；
  - `optimizedState` / `optimizedInput` 正常更新；
  - `applySupportTorque()` 作为零输入站立主稳定器；
- 当前尚不能把此状态描述为“NMPC hybrid 已可独立承担站立控制”；
- 当前更准确的表述应为：
  - `zero-command standing works with support-torque-led stabilization`

后续注意事项：

- 当前 base height 仍存在约 1 cm 量级误差，后续可继续细调；
- 后续进入非零速度命令测试前，不要撤掉当前零输入站立 fallback；
- 若后续 AI 接手，应先保持当前“零输入稳站”能力，再推进运动控制阶段。

### 2.15 已复现“有时可稳站、有时明显失稳”，当前瓶颈已收敛到 support torque 姿态目标不完整

在 `2.14` 的基础上，又做了多轮重复实验，用来排除偶然性。结论如下：

- 当前系统确实出现过“零输入下基本稳定站立”的运行窗口；
- 但同一套代码在不同初始姿态/初始进入时机下，仍会稳定复现另一类失稳现象；
- 因此，当前不能把状态描述为“零输入站立已经鲁棒解决”；
- 更准确的描述应为：
  - `zero-command standing is partially achieved but not yet robust`

已复现的两类典型工况：

1. 较好工况

- `planned mode: 15` 持续稳定；
- `optimizedState / optimizedInput` 输出平滑；
- `candidate joint_pos` 与实测 joint position 基本一致；
- `support target z` 与 `measured z` 误差可收敛到约 1 cm 量级；
- 机身姿态接近直立；
- support torque 日志可见稳定非零输出；
- 机器人外观上接近“基本站稳”。

2. 较差工况

- 同样是零输入、同样 `planned mode: 15`；
- `optimizedState / optimizedInput` 仍然连续、平滑，说明 solver 主链没有坏；
- 但 estimator 日志会稳定复现：
  - `pitch ≈ -0.34 rad`
  - `measured z ≈ 0.30`
  - `support target z ≈ 0.346`
  - `pos_error z ≈ 0.044 ~ 0.046`
- 也就是：
  - 机身长时间保持明显前倾；
  - base height 长时间低于目标高度，并继续缓慢下沉；
  - `support torque[0]` 多次接近 `0` 或极小值；
- 这说明当前失稳并不是：
  - gait mode 错；
  - solver 没算；
  - `optimizedState / optimizedInput` 乱跳；
- 更像是：
  - support torque 这一层没有形成完整、鲁棒的站姿恢复能力。

这一轮重复实验后，当前最重要的新判断是：

- 当前零输入 fallback 虽然已经优于早期“完全站不住”的状态；
- 但它还不是完整 stand controller；
- 问题重点已经从：
  - `NMPC state/input` 是否接上
  - `planned mode` 是否正确
  - `optimizedState` 是否被读取
- 收敛到：
  - `applySupportTorque()` 是否真正建立了完整的姿态恢复目标；
  - 尤其是是否只锁了 `yaw`，但没有显式锁定并恢复 `roll / pitch`。

当前最值得后续 AI 直接接手验证的怀疑点：

- `StateNMPC::applySupportTorque()` 中的期望姿态目前主要由 `support_yaw_target_` 构造；
- 当前实现大概率只构造了绕 `z` 轴的目标旋转（例如 `Rd = rotz(...)`）；
- 这意味着在零输入站立时，控制器并没有显式地保持“进入 NMPC 时的完整机身姿态”；
- 一旦进入时存在较大 `pitch` 偏差，控制器可能没有正确的 `roll / pitch` 参考把机身拉回直立；
- 这与重复实验中长期维持 `pitch ≈ -0.34 rad` 的现象高度一致。

因此，当前阶段的最高优先级已经更新为：

1. 不要再优先怀疑：
   - `planned mode: 15`
   - `optimizedState / optimizedInput`
   - yaw/roll 索引顺序
2. 优先检查并改造：
   - `StateNMPC.h` 中是否只有 `support_yaw_target_`
   - `StateNMPC::enter()` 是否只初始化了 `yaw target`
   - `StateNMPC::applySupportTorque()` 是否只以 `yaw` 构造期望姿态
3. 后续正确方向是：
   - 将零输入站立目标从“位置 + yaw”升级成“位置 + 完整 `roll/pitch/yaw` 姿态目标”
   - 先锁住进入 `NMPC` 时的完整姿态，再验证是否能显著改善前倾失稳工况

这一步的重要性高于：

- 继续微调 `Q/R`
- 提前扩展 gait/mode schedule
- 提前接更复杂的 WBC

因为当前多轮日志已经表明：

- solver 主链基本稳定；
- 真正不稳定的是“站立执行层的姿态恢复能力”。

---

## 3. 当前代码结构的真实状态

当前 `unitree_guide_controller` 中，NMPC 相关实现大致分布如下：

### 3.1 状态层

- `src/FSM/StateNMPC.cpp`
- `include/unitree_guide_controller/FSM/StateNMPC.h`

职责：

- 作为 NMPC 状态入口
- 做 readiness check
- 调用 `CtrlComponent` 的 runtime 初始化入口
- 在运行阶段承担最小执行层职责：
  - 高频调用 `applyNmpcJointVelocityCommand()`
  - 将最近一次有效 `nmpc_joint_velocity_cmd_` 写入 velocity command interface

### 3.2 组件层

- `src/control/CtrlComponent.cpp`
- `include/unitree_guide_controller/control/CtrlComponent.h`

职责：

- 管理路径
- 管理 joint / foot names
- 持有 `LeggedInterface`
- 持有：
  - `SqpMpc`
  - `MPC_MRT_Interface`
  - `SystemObservation`
  - `CentroidalModelRbdConversions`
- 维护：
  - `measured_rbd_state_`
  - `optimized_state_`
  - `optimized_input_`
  - `planned_mode_`
  - `nmpc_joint_velocity_cmd_`
- 提供：
  - runtime ready 判定
  - `advanceMpcOnce()`
  - `evaluateCurrentPolicy()`
  - `updateMeasuredRbdStateFromEstimator()`
  - `getNmpcJointVelocityCommand()`
  - `hasNmpcJointVelocityCommand()`

### 3.3 自有 NMPC 接口层

- `src/nmpc/LeggedInterface.cpp`
- `include/unitree_guide_controller/nmpc/LeggedInterface.h`

职责：

- 作为自有 NMPC 问题定义层入口
- 当前阶段负责：
  - 文件与 section 校验
  - 读取 OCS2 settings
  - 注入 `joint_names / foot_names`
  - 创建最小建模对象：
    - `PinocchioInterface`
    - `CentroidalModelInfo`
    - `initialState`
  - 创建：
    - `ReferenceManager`
    - `Initializer`
    - `OptimalControlProblem`
    - `Rollout`
  - 作为后续 `SqpMpc` 与更完整问题定义扩展的桥接层

---

## 4. 当前明确不做的事情

为了防止后续 AI 或开发者误判节奏，这里明确列出当前阶段**不要提前做**的事情。

### 4.1 不直接接 WBC

当前不要优先处理：

- `WeightedWbc`
- `WbcBase`
- torque 输出
- 全身控制约束层

原因：

- 先把 MPC 主链打通更重要
- 一旦 MPC、估计器、WBC 同时改，问题很难定位

### 4.2 不直接依赖 `ocs2_quadruped_controller`

当前不要做：

- 在 `unitree_guide_controller` 中 include `ocs2_quadruped_controller` 的头文件
- 把另一个 controller 包作为你的编译必要依赖

### 4.3 不一下子完整迁移 `MPC_legged_control`

当前不要尝试一次性完整复制：

- `legged_interface`
- `constraint/*`
- `precomputation`
- `reference manager`
- `wbc`

应继续保持：

- 先迁移最小功能
- 每一步都能编译和验证

---

## 5. 当前计划的阶段划分

### 阶段 A：FSM 与配置入口

状态：

- 已完成

内容：

- `StateNMPC`
- `robot_pkg`
- 路径生成

### 阶段 B：自有接口层骨架

状态：

- 已完成增强版并进入最小建模阶段

内容：

- 自有 `nmpc::LeggedInterface`
- 文件与配置结构校验
- OCS2 settings 加载
- `joint_names / foot_names` 注入
- `PinocchioInterface`
- `CentroidalModelInfo`
- `initialState`

### 阶段 C：状态层与接口层打通

状态：

- 已完成

内容：

- `StateNMPC` 能读取 `LeggedInterface`
- 能调用 `setupMpcRuntime()`

### 阶段 D：MPC runtime 壳子

状态：

- 已完成，但已不再是当前阶段重点

内容：

- `CtrlComponent` 中存在 runtime 入口函数和状态位
- 这一步已经作为历史阶段完成
- 后续不应再把当前状态描述为“只剩 shell”

### 阶段 E：真实 MPC 对象接入

状态：

- 已完成

内容：

- 不再直接桥接原版 `ocs2::legged_robot::LeggedRobotInterface`
- 改为继续在自有 `nmpc::LeggedInterface` 内逐步补齐：
  - `ReferenceManager`
  - `OptimalControlProblem`
  - `Initializer`
- 在以上对象 ready 后，`CtrlComponent::setupMpcRuntime()` 已成功创建：
  - `MPC_BASE`
  - `MPC_MRT_Interface`
  - `SqpMpc`

### 阶段 F：policy 级运行

状态：

- 已完成最小可用链路

内容：

- `setCurrentObservation`
- `updatePolicy()`
- `evaluatePolicy(...)`
- 观察 `optimized_state / optimized_input / planned_mode`

当前补充说明：

- 已能稳定读到：
  - `optimized_state`
  - `optimized_input`
  - `planned_mode`
- `optimized_input` 中的关节速度段已可进一步提取为执行层候选命令
- 当前 policy 质量仍受“问题定义仍是最小骨架”限制

### 阶段 G：简化执行层

状态：

- 已完成第一版最小可用接入

内容：

- 不接 WBC，先做简化 velocity 级执行
- `CtrlComponent::evaluateCurrentPolicy()` 已从 `optimized_input` 中提取：
  - `nmpc_joint_velocity_cmd_`
- `StateNMPC::applyNmpcJointVelocityCommand()` 已将：
  - `nmpc_joint_velocity_cmd_`
  写入：
  - `joint_velocity_command_interface_`
- 当前执行结构已经升级为：
  - 低频 `updateMeasuredRbdStateFromEstimator()`
  - 低频 `advanceMpcOnce() / evaluateCurrentPolicy()`
  - 高频保持下发最近一次有效 `joint velocity command`
- 已加入第一版执行保护：
  - `std::clamp(cmd, -2.0, 2.0)`

### 阶段 G.1：键盘速度命令已接入 NMPC reference

状态：

- 已完成并通过运行验证

内容：

- `StateNMPC::run()` 已读取：
  - `ctrl_interfaces_.control_inputs_.lx`
  - `ctrl_interfaces_.control_inputs_.ly`
  - `ctrl_interfaces_.control_inputs_.rx`
- 并通过：
  - `CtrlComponent::updateDesiredBodyVelocityCommand(vx, vy, yaw_rate)`
  - `CtrlComponent::updateTargetTrajectoriesFromCommand()`
  将键盘命令转为运行时 `TargetTrajectories`
- 当前已能在日志中稳定看到：
  - `target cmd vx: 0.4, vy: -0.2, yaw_rate: -0`

验证结论：

- `键盘 /control_input -> NMPC reference` 这条链路已经真实打通
- 后续在 `baseTrackingCost` 接入后重新运行观察表明：
  - `optimized input[0]` 已不再长期保持 0
  - `candidate joint_vel[0]` 已不再长期保持 0
  - `applied joint_vel_cmd[0]` 已不再长期保持 0
- 这说明：
  - reference 已经不只是“接进去”
  - solver 也已经开始对键盘目标产生真实响应

### 阶段 G.2：`baseTrackingCost` 已接入并验证

状态：

- 已完成
- 已通过编译和运行验证

背景结论：

- 已通过对照 `ocs2_legged_robot::LeggedRobotInterface`
  和 `ocs2_quadruped_controller::LeggedInterface`
  确认当前自有 `LeggedInterface::setupOptimalControlProblemSkeleton()` 只有：
  - `dynamicsPtr`
  - `rollout`
- 在此阶段之前**没有任何 cost 项**
- 这就是为什么 reference 已经更新，但 solver 仍持续输出 0 附近控制量的核心原因

已完成的内容：

- `ocs2_legged_robot/cost/LeggedRobotQuadraticTrackingCost.h`
- `initializeInputCostWeight(...)`
- `getBaseTrackingCost(...)`
- 在 `setupOptimalControlProblemSkeleton()` 中加入：
  - `problem->costPtr->add("baseTrackingCost", ...)`

当前最新代码状态：

- `baseTrackingCost` 相关函数声明已加入：
  - `include/unitree_guide_controller/nmpc/LeggedInterface.h`
- `baseTrackingCost` 相关实现已开始加入：
  - `src/nmpc/LeggedInterface.cpp`
- 命名空间与类型别名问题已经收尾
- 当前启动时已能看到：
  - `WARNING: Loaded at least one default value in matrix: "Q"`
  - `WARNING: Loaded at least one default value in matrix: "R"`
  - `baseTrackingCost added successfully.`

运行验证结论：

- 之前“reference 已更新但输出仍接近 0”的核心原因已经定位并修复
- 当前 `OptimalControlProblem` 已不再只是：
  - `dynamicsPtr`
  - `rollout`
- 而是已经真正包含至少一项 tracking cost
- 在此之后，`optimized_input` 与 `joint velocity command` 已开始出现非零响应

### 阶段 G.2.1 当前剩余问题不再是编译，而是调参与稳定性

截至 2026-04-28，当前新的重点是：

- `Q`
- `R`
- 执行层稳定性
- 步态/约束/参考定义是否足够支持稳定移动

当前已经确认的现象：

- 即使 `target cmd vx` 为 0，solver 也可能输出非零控制量
- 当 `target cmd vx` 从 `0 -> 0.02 -> 0.4` 变化时，
  `optimized_input` 和 `candidate joint_vel` 已会跟着变化
- 这说明系统已经从“纯静态无响应”进入“有响应但尚未调优稳定”的阶段

因此下一位 AI 不要回头再修 tracking cost 编译问题，而是直接做下面这件事：

1. 检查 `task.info` 中 `Q` / `R` 是否合理
2. 对 `Q` / `R` 做最小调参
3. 重新验证：
   - `optimized input[0]`
   - `candidate joint_vel[0]`
   - `joint_vel_cmd[0] before clamp`
   - `joint_vel_cmd[0] after clamp`
4. 再决定是否需要补：
   - `joint_position_command_interface_`
   - `joint_kp_command_interface_`
   - `joint_kd_command_interface_`

### 阶段 H：WBC 接入

状态：

- 明确延后

内容：

- 完整闭环
- torque 输出

---

## 6. 下一步具体工作计划

下面是当前最推荐的下一步。注意：旧版本文档里写的“先补 joint q/dq 接线”已经完成，新的 AI 不要回头重复做这一步。

### 下一步主目标

**当前主目标不再是继续围绕 `Q/R` 和单一执行增益做小修小补，也不再是在 `StateNMPC` 内部继续堆叠基于 `planned_mode` 的零散分支，而是最大程度复用现有 `gait/` 与 `StateTrotting` 的步态执行链路，把 `StateNMPC` 重构为“统一 gait source + swing/stance 分腿执行”的 locomotion 结构。**

当前已经确认：

- `reference -> MPC -> planned_mode` 这条链路已经打通
- `planned_mode` 已能在 `15 / 6 / 9` 之间切换
- `support contact` 也已开始跟随 mode 切换
- 零命令稳站问题已经通过 `support_torque_only + force_full_contact` 思路基本分离

但当前真正阻碍“走起来”的核心，不再是：

- `Q/R` 是否再细调一点
- `joint velocity clamp` 是否再小一点

而是：

**当前执行层没有真正做到“摆动腿”和“支撑腿”使用不同控制律；同时，步态/相位/摆动足轨迹这几个本应独立的模块，还没有被工程化复用进 `StateNMPC`。**

### 下一步建议子任务

1. 明确接受当前新的执行层目标结构
   - `cn = 0` 摆动腿：
     - `Swing Trajectory`
     - `IK + PD`
     - 输出 `tau_swing`
   - `cn = 1` 支撑腿：
     - `BalanceCtrl / MPC force distribution`
     - `foot force f`
     - `tau_stance = J^T f`
   - 最终按腿合成控制输出
2. 不再把 `support_torque + nmpc_hybrid` 的简单叠加视为最终 locomotion 架构
   - 这套结构对“先站稳”有帮助
   - 但对“真正走起来”是不够的
3. 优先统一 gait source，不要在 `StateNMPC` 内长期直接以 `planned_mode` 作为唯一腿角色来源
   - `planned_mode` 仍然是上层 NMPC 的模式输出，必须保留
   - 但下层 swing/stance 执行不应继续散落为：
     - `planned_mode -> contact -> leg role`
     的局部硬编码
   - 更合理的工程方向是：
     - 复用 `gait/WaveGenerator`
     - 复用 `gait/GaitGenerator`
     - 让 `StateNMPC` 通过统一 gait 层获取：
       - `contact`
       - `phase`
       - swing foot target
4. 支撑腿执行链优先复用当前已有能力
   - `BalanceCtrl::calF(...)`
   - `J^T f`
   - 只作用于当前 `contact = 1` 的腿
5. 摆动腿执行链优先复用 `StateTrotting` 中已有思路，并尽量直接复用 `gait/` 目录已有模块
   - 摆动轨迹生成
   - 摆动相位 `phase`
   - 落脚点计算 `FeetEndCalc`
   - 足端目标跟踪
   - IK / 关节参考
   - `PD` 增益只作用于当前 `contact = 0` 的腿
6. 零命令稳站和非零命令 locomotion 明确区分
   - `zero_command = true`
     - 允许继续使用稳站 fallback
     - 不应因为 gait template 自动切换而强制进入对角支撑
   - `zero_command = false`
     - 才进入真正的 locomotion 执行链
7. `Q/R` 调参与增益调参仍然重要，但降级为第二优先级
   - 只有在 swing / stance 分工成立之后，调参才更有意义
   - 否则只是在错误架构上做局部优化
8. 工程实现上应避免把所有 gait / swing 逻辑继续杂糅进 `StateNMPC.cpp`
   - `StateNMPC` 应逐步收敛为：
     - NMPC 观测与参考更新
     - gait 执行调度
     - stance / swing 控制结果汇合
   - 不应继续膨胀成：
     - 自带步态器
     - 自带相位器
     - 自带轨迹器
     - 自带所有落脚点逻辑

### 6.1 截至当前的新增验证结论（2026-04-28）

这部分是旧版文档没有同步进去、但当前非常关键的新进展。

#### 6.1.1 `TargetTrajectories` 的 `x1` 更新逻辑已经完成方向验证

已完成并验证：

- 在 `CtrlComponent::updateTargetTrajectoriesFromCommand()` 中修正了 `base_pose` 索引使用
- 当前已确认 `getBasePose(x, model_info)` 对应的 6 维布局应按：
  - `0`: `px`
  - `1`: `py`
  - `2`: `pz`
  - `3`: `yaw`
  - `4`: `pitch`
  - `5`: `roll`

本轮修正后的关键逻辑是：

- `yaw = base_pose(3)`
- `base_pose(0) += vx_world * dt`
- `base_pose(1) += vy_world * dt`
- `base_pose(3) += yaw_rate * dt`

当前已经通过日志完成以下验证：

1. 零指令验证
   - `target cmd vx: 0, vy: 0, yaw_rate: 0`
   - `delta base: dpx=0, dpy=0, dyaw=0`
2. 纯前进验证
   - 当 `vx = 0.4` 且 `dt = 0.3` 时
   - 已稳定观察到 `dpx ≈ 0.12`
3. 前进 + 侧移验证
   - 当 `vx = 0.4, vy = -0.2` 时
   - 已稳定观察到 `dpx ≈ 0.12, dpy ≈ -0.06`
4. 纯转向验证
   - 当 `yaw_rate = -0.5` 且 `dt = 0.3` 时
   - 已稳定观察到 `dyaw = -0.15`

这说明：

- `keyboard / control_input -> desired body velocity -> TargetTrajectories -> x1`
  这条 reference 链已经完成方向与量级验证
- 后续新的 AI **不要再优先怀疑 reference 方向写错**
- 当前主要矛盾已经转移到：
  - `Q / R`
  - 执行层质量
  - gait / constraints / mode schedule

#### 6.1.2 执行层日志已升级为 clamp 前后双日志

已完成：

- `StateNMPC::applyNmpcJointVelocityCommand()` 不再只打印缓存值
- 当前已新增：
  - `joint_vel_cmd[0] before clamp`
  - `joint_vel_cmd[0] after clamp`

这一步很重要，因为此前日志里的：

- `applied joint_vel_cmd[0]`

实际上并不能区分：

- solver 原始输出过大
- 还是执行层真实下发值过大

现在新的 AI 接手后，应该优先使用：

- `candidate joint_vel[0]`
- `joint_vel_cmd[0] before clamp`
- `joint_vel_cmd[0] after clamp`

来判断当前问题究竟在：

- `Q / R` 过激进
- 还是 execution clamp 触发过多

### 6.2 截至当前的最新结构性结论（2026-05-07）

这部分比前面所有“继续调执行层细节”的结论都更重要。后续新的 AI 或开发者接手时，应优先以这里为准。

#### 6.2.1 `gait.info` 目前并未真正进入运行链，当前 gait template 实际来自 `reference.info`

已确认：

- `CtrlComponent` 虽然持有 `gait_file_`
- 但当前自有 `LeggedInterface` 构造与 `loadGaitSchedule()` 仍只从：
  - `reference.info`
  读取：
  - `initialModeSchedule`
  - `defaultModeSequenceTemplate`

因此此前 `planned_mode` 长期固定为 `15` 的根因不是 solver 本身，而是：

- `reference.info` 中 `defaultModeSequenceTemplate` 被配置成纯 `STANCE`

后续为验证 gait 切换，已经临时将 `reference.info` 改成 `standing_trot` 风格模板，因此当前才会看到：

- `planned_mode = 15 / 6 / 9` 周期切换

这个结论说明：

- gait schedule 本身已经开始生效
- 当前问题不再是“为什么 mode 不切”

#### 6.2.2 当前主要失败点不在上层 MPC，而在下层执行层分工错误

当前已经验证到：

- `planned_mode` 会切
- `support contact` 会切
- 零命令下可通过稳站逻辑保持基本站立

但一旦给速度命令，机器人仍会垮掉。其核心原因不是：

- 键盘输入比例略大
- `Q/R` 还不够精细
- `candidate joint_vel` 数值还有波动

而是：

**执行层没有真正按照每条腿当前是 swing 还是 stance 去分配控制职责；同时，`StateNMPC` 还没有复用现有成熟的步态模块，而是在内部逐步堆积 gait 相关逻辑。**

当前代码更接近：

- 全身都参与 `support torque`
- 全身又都接受 `nmpc hybrid joint command`

而不是标准 locomotion 结构中的：

- 摆动腿只走 `Swing Trajectory + IK + PD`
- 支撑腿只走 `MPC / force distribution + J^T f`

#### 6.2.3 当前整体思路需要从“在现有叠加结构上补丁”转向“统一 gait source + 执行层按腿重构”

当前新的总体判断如下：

1. 现有 `support_torque + nmpc_hybrid` 叠加结构适合用于：
   - 打通 NMPC 链路
   - 实现零命令稳站 fallback
2. 但这套结构不应再被当作最终 locomotion 架构继续硬补
3. 如果目标是“让狗走起来”，执行层应明确迁移到：
   - 统一 gait source
   - `stance leg -> BalanceCtrl / J^T f`
   - `swing leg -> swing trajectory / IK / PD`
4. 从工程设计看，不推荐继续在 `StateNMPC` 内部长期直接围绕 `planned_mode` 自己实现完整 gait 执行层
5. 更推荐的方向是：
   - 让 `planned_mode` 继续作为 NMPC 上层模式输出
   - 让 `gait/` 目录中的模块承担：
     - 相位
     - 接触
     - 落脚点
     - 摆动足轨迹
   - `StateNMPC` 只负责：
     - 读 NMPC
     - 调 gait
     - 汇合 swing / stance 控制

因此，从本文档这一刻起，后续工作主线调整为：

- 优先完成统一 gait source 下的 swing / stance 分腿执行框架
- 次优先级才是：
  - `Q/R`
  - `kp/kd`
  - clamp
  - 其它执行细节微调

#### 6.2.4 后续修改时的判断准则

后续每一项修改都应先问自己下面这个问题：

**这项修改是在让“摆动腿更像摆动腿、支撑腿更像支撑腿，并且复用了现有 gait 模块”，还是只是在 `StateNMPC` 里继续堆局部补丁？**

如果答案是后者，那么优先级应下降。

换句话说，后续路线应优先复用：

- `gait/WaveGenerator` 中成熟的相位与接触逻辑
- `gait/GaitGenerator` 中成熟的摆动足轨迹逻辑
- `gait/FeetEndCalc` 中成熟的落脚点计算逻辑
- `StateTrotting` 中成熟的 swing leg 处理思路
- `StateNMPC` / `BalanceCtrl` 中已经跑通的 stance leg 力分配思路

最终把两者组合成新的 NMPC locomotion 执行层，而不是继续把所有 gait / swing 功能都塞进 `StateNMPC` 自己实现。

#### 6.1.3 `R` 已做过第一轮 A/B 调整

当前 `go1_description/config/ocs2/task.info` 中：

- `R` 的 foot velocity relative to base 那 12 个对角项
- 已从早期的 `5000.0`
- 降到 `1000.0`
- 后又回调到当前的 `2000.0`

当前文件中的真实状态应为：

- `(12,12)` 到 `(23,23)` 均为 `2000.0`

这意味着：

- 新的 AI 不要再把这一步描述成“还没开始调 `R`”
- 当前已经处于：
  - `5000 -> 1000 -> 2000`
  的第一轮调参中间态

#### 6.1.4 当前对 `R = 2000` 的最新观察

基于最近一轮运行日志，当前已经看到：

- 零输入时，`joint_vel_cmd[0] before clamp` 仍可能非零
  - 例如 `1.26174`
- 但当前多组日志中：
  - `before clamp == after clamp`
  - 尚未看到第 0 个关节速度长期撞上 `±2.0` clamp
- 在有输入时：
  - `vx / vy / yaw_rate` 对 `delta base` 的响应方向正常
  - `candidate joint_vel[0]` 仍有明显变化

当前阶段的判断是：

- `R = 2000` 相比 `1000` 更接近“可继续观察”的中间值
- 但系统是否已经足够稳定，仍需继续通过：
  - 零输入抖动
  - 小输入连续性
  - 机器人实体动作趋势
  这三类现象综合判断

### 再下一步主目标

在 velocity 执行已经稳定后，再进入：

- 简化执行层
- 或补最小 tracking cost / constraints 以提升策略质量

### 重要提醒

- 不要一次性把过多 OCS2 / ROS / WBC 依赖全部引入
- 继续保持“自有接口层先建模，再进入 solver / execution refinement”
- 当前已经验证通过的结构不要轻易回退：
  - `CtrlInterfaces` 已绑定到 `CtrlComponent`
  - `joint q/dq` 已从真实 state interface 读入
  - `nmpc_joint_velocity_cmd_` 已能写入 velocity command interface

---

## 7. 关键文件清单

### 已稳定的关键文件

- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateNMPC.cpp`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/FSM/StateNMPC.h`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/CtrlComponent.cpp`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/control/CtrlComponent.h`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/nmpc/LeggedInterface.cpp`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/nmpc/LeggedInterface.h`

### 已改过的配置文件

- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/descriptions/unitree/go1_description/config/gazebo.yaml`

### 下一步高优先级会继续改的文件

- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/descriptions/unitree/go1_description/config/ocs2/task.info`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/nmpc/LeggedInterface.cpp`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/nmpc/LeggedInterface.h`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateNMPC.cpp`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/FSM/StateNMPC.h`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/control/CtrlComponent.h`
- `/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/CtrlComponent.cpp`

---

## 8. 当前可观测的验证现象

如果当前工程状态正常，应该已经可以看到：

- 启动控制器时：
  - NMPC 路径打印正常
  - `LeggedInterface` 基础初始化正常
  - `model_settings jointNames size: 12`
  - `model_settings contactNames3DoF size: 4`
  - `OCS2 settings loaded successfully.`
  - `PinocchioInterface created successfully.`
  - `CentroidalModelInfo stateDim: 24`
  - `CentroidalModelInfo inputDim: 24`
  - `initialState size: 24`
- 切换到 `StateNMPC` 时：
  - `NMPC interface is ready`
  - 配置摘要输出
  - `setupMpcRuntime()` 被调用
  - `SqpMpc created successfully.`
  - `MPC_MRT_Interface created successfully.`
  - `MPC runtime is ready.`
  - `advanceMpcOnce() succeeded.`
  - `Latest NMPC policy cached successfully.`
  - `optimized state dim: 24`
  - `optimized input dim: 24`
  - `planned mode: 15`
  - `candidate joint_vel dim: 12`
  - `cached nmpc_joint_velocity_cmd dim: 12`
  - `latest NMPC policy is available.`
  - `measured_rbd_state_ updated from estimator.`
  - `measured_rbd_state_ dim: 24`
  - `applying joint velocity command, dim: 12`
  - `joint_vel_cmd[0] before clamp: ...`
  - `joint_vel_cmd[0] after clamp: ...`
  - `target cmd vx: 0.4, vy: -0.2, yaw_rate: -0`
  - `x0 base: px=..., py=..., yaw=...`
  - `x1 base: px=..., py=..., yaw=...`
  - `delta base: dpx=..., dpy=..., dyaw=...`
  - `WARNING: Loaded at least one default value in matrix: "Q"`
  - `WARNING: Loaded at least one default value in matrix: "R"`
  - `baseTrackingCost added successfully.`
  - `optimized input[0]: 21.8665`
  - `candidate joint_vel[0]: 0.151121`
  - `joint_vel_cmd[0] before clamp: 0.151121`
  - `joint_vel_cmd[0] after clamp: 0.151121`

### 8.1 当前应优先检查的运行现象

新的 AI 接手后，优先按下面顺序看日志，不要一上来就猜代码。

1. 先看 reference 是否正确
   - 看：
     - `target cmd vx / vy / yaw_rate`
     - `x0 base`
     - `x1 base`
     - `delta base`
   - 目标是确认：
     - 零输入时 `delta` 接近 0
     - `vx = 0.4` 时 `dpx ≈ 0.12`
     - `vy = -0.2` 时 `dpy ≈ -0.06`
     - `yaw_rate = -0.5` 时 `dyaw = -0.15`
2. 再看 solver 输出是否合理
   - 看：
     - `optimized input[0]`
     - `candidate joint_vel[0]`
3. 最后看 execution 是否被 clamp
   - 看：
     - `joint_vel_cmd[0] before clamp`
     - `joint_vel_cmd[0] after clamp`
   - 如果经常出现：
     - `before clamp` 很大
     - `after clamp` 长期等于 `2.0` 或 `-2.0`
   - 则优先怀疑：
     - `R` 太小
     - solver 输出过激进

如果后续 AI 接手时完全看不到 `baseTrackingCost added successfully.`，应优先回查：

- `src/nmpc/LeggedInterface.cpp`
- `include/unitree_guide_controller/nmpc/LeggedInterface.h`
- `task.info` 中 `Q` / `R`

如果后续 AI 接手时看不到这些现象，应优先回查：

- `gazebo.yaml` 中 `robot_pkg`
- `CtrlComponent::setupNmpcConfig()`
- `CtrlComponent::setupLeggedInterface()`
- `StateNMPC::enter()`
- `CtrlComponent::setupMpcRuntime()`
- `CtrlComponent::updateMeasuredRbdStateFromEstimator()`
- `CtrlComponent::evaluateCurrentPolicy()`
- `StateNMPC::run()`
- `StateNMPC::applyNmpcJointVelocityCommand()`

---

## 9. 交接结论

截至当前，项目已经完成了：

- `StateNMPC` 状态接入
- `CtrlComponent` 作为 NMPC 宿主容器的基础升级
- `robot_pkg` 和 OCS2 配置路径接入
- 自有 `nmpc::LeggedInterface` 骨架建立
- 基础配置校验
- `PinocchioInterface / CentroidalModelInfo / initialState` 接入
- `ReferenceManager / Initializer / OptimalControlProblem / Rollout` 接入
- `SqpMpc / MPC_MRT_Interface` 创建成功
- `setCurrentObservation / advanceMpc / updatePolicy / evaluatePolicy` 打通
- `optimized_state / optimized_input / planned_mode` 可持续读取
- `Estimator` 的 base 状态已接入 `measured_rbd_state_`
- joint q/dq 已从真实 state interface 接入 `measured_rbd_state_`
- `optimized_input` 中的关节速度段已提取为 `nmpc_joint_velocity_cmd_`
- `nmpc_joint_velocity_cmd_` 已能写入 `joint_velocity_command_interface_`
- 已完成“低频求解 + 高频命令保持下发”的第一版执行结构
- 已加入第一版速度限幅保护
- 键盘速度命令已能进入 `TargetTrajectories`
- `baseTrackingCost` 已完成接入
- solver 已开始对键盘 reference 产生非零控制响应
- `TargetTrajectories` 的 `x1` 更新逻辑已经通过 `vx / vy / yaw_rate` 三通道方向验证
- `StateNMPC` 已能打印 clamp 前后的真实 joint velocity 下发值
- `task.info` 中 `R` 的 12 个 foot velocity 权重已完成：
  - `5000 -> 1000 -> 2000`
  的第一轮 A/B 调整
- `CtrlComponent` 中已加入 runtime 时钟推进：
  - `advanceMpcClock(double dt)`
- `CtrlComponent` 中已加入最新 policy 到 runtime 缓存的同步：
  - `syncMpcRuntimeFromLatestPolicy()`
- 已验证：
  - `observation_.time` 不再停留在 `0`
  - 会按控制周期低频推进，例如 `7.5 -> 7.6 -> 7.7`
- `StateNMPC::run()` 的关键时序已调整为：
  - 先刷新测量状态
  - 再更新 reference
  - 再低频推进 MPC / 评估 policy
- `updateTargetTrajectoriesFromCommand()` 已从 2 点 reference 升级为多点 short-horizon reference
  - 当前为 4 点
  - `dt = 0.3`
  - 因此末点通常对应约 `0.9s` horizon
- 已验证多点 short-horizon reference 的日志现象：
  - `dpx ≈ 0.36`
  - `dyaw ≈ -0.45` 或对应当前较小转向命令的合理量级
- `StateNMPC` 的执行层已从纯 velocity 下发升级为最小混合执行：
  - `position + velocity + kp + kd`
- `StateNMPC` 中已接入第一版支撑 torque：
  - 复用 `BalanceCtrl::calF(...)`
  - 复用 `robot_model_->getTorque(...)`
  - 当前暂时将四条腿都视为接触腿
- 已验证：
  - `support torque[0]` 非零
  - 说明力分配链条本身已经进入 `StateNMPC`
- `StateNMPC -> CtrlComponent -> LeggedInterface -> setupMpcRuntime() -> policy evaluation` 链路打通

当前**还没有完成**：

- `Q / R` 的有效调参与完整配置
- 更高质量的 tracking cost / constraints / reference 定义
- 更稳定、更可解释的 joint velocity policy 输出
- `StateNMPC` 中真正可支撑机身的执行层
- 接触相/摆动相区分后的支撑 torque 策略
- position / kp / kd 与 velocity 的协同执行策略
- WBC

后续正确方向是：

**继续在 `unitree_guide_controller` 内部推进自有 NMPC 主链，不再回头重做基础接线，而是围绕“问题定义质量 + 执行质量”继续推进：当前优先级已经从“让 solver 跑起来”转移到“让执行层先支撑住机身，再让 NMPC 输出真正产生运动效果”。**

### 9.1 当前最新阶段判断（2026-05-07）

当前新的关键事实如下：

1. solver 主链已不是主要矛盾
   - `advanceMpcOnce()` 持续成功
   - `evaluateCurrentPolicy()` 持续成功
   - `optimized_input / candidate joint_vel / mpc_input_` 持续更新
2. runtime 时钟与 reference 时序已基本自洽
   - `observation_.time` 持续推进
   - 多点 short-horizon reference 的 `delta base` 符合理论量级
3. 当前主要矛盾已转移到执行层
   - 即使已补 `position + velocity + kp + kd`
   - 且已接入第一版 `BalanceCtrl` 支撑 torque
   - 机器人在 `StateNMPC` 中仍然“站不住 / 易趴下”

因此新的 AI 接手时，**不要再优先回头调时钟推进、reference 两点/四点逻辑、或怀疑 solver 是否在运行**。这些都已经过了最早的验证阶段。

### 9.2 当前关于“站不住”的最重要结论

截至当前，最需要同步给新的 AI 的结论是：

**机器人进入 `StateNMPC` 后仍站不住，当前最高概率原因不是“NMPC 没有输出”，而是“执行层虽然开始接入力分配，但还没有形成真正闭环的支撑控制能力”。**

支撑这个判断的已知事实：

- `support torque[0]` 已非零
- 说明 `BalanceCtrl -> getTorque -> torque interface` 这条链已经被接进 `StateNMPC`
- 但与此同时：
  - `candidate joint_vel[0]` 量级仍很小
  - `planned mode` 长期保持单一
  - 当前支撑 torque 仍是“所有腿全接触”的粗糙近似
  - 没有真实接触相/摆动相区分
  - 没有完整 WBC 闭环

因此当前更合理的判断是：

- **第一版支撑 torque 已进入，但还不足以让机身稳定支撑**
- 这不是“完全没接入力分配”
- 也不是“单纯调 `Q/R` 就能解决”的问题

### 9.3 新的 AI 接手后，不要重复做的事情

新的 AI 接手后，请不要优先重复下面这些动作：

1. 不要重新回头验证：
   - `x0 / x1 / delta base` 的方向是否正确
   - `observation_.time` 是否还在 `0`
   - `optimized_input` 是否完全不更新
2. 不要把当前问题重新误判成：
   - “NMPC 没输出”
   - “reference 没接进去”
   - “只是少了 velocity interface”
3. 不要直接跳去完整 WBC 迁移
   - 当前更适合继续基于已有 `BalanceCtrl` 链条做最小增强

### 9.4 新的 AI 当前接手后最推荐的排查方向

当前最推荐的新一轮排查顺序是：

1. 先检查 `StateNMPC` 中支撑 torque 的目标定义是否合理
   - 当前 `support_body_pos_target_`
   - `support_body_vel_target_`
   - `support_yaw_target_`
   是否过于粗糙
2. 检查当前“全腿接触”的假设是否过强或不合理
   - 是否导致力分配与实际姿态不匹配
3. 检查 torque 支撑与 `position + velocity + kp + kd` 是否互相打架
   - 尤其是 `kp/kd` 是否仍偏硬
4. 只有在执行层支撑逻辑更合理之后，再继续评估：
   - gait / mode schedule
   - 更细的 `Q/R`
   - 更完整的接触约束

---

## 10. 建议接手模式

如果新的 AI 接手，请保持下面这种工作模式，不要切回“大段抽象讨论但不给具体修改位置”的节奏。

### 10.1 沟通模式

- 用户希望：
  - 先解释修改逻辑
  - 但一旦进入实现阶段，就必须给出**具体文件、具体位置、具体代码**
- 用户通常自己动手改代码，因此新的 AI 应优先提供：
  - 改哪个文件
  - 放在哪一段附近
  - 替换成什么代码
  - 改完后用什么命令编译
- 不要只给高层方案而不落到代码
- 用户当前更希望：
  - AI 解释“为什么先做这一步”
  - 然后再告诉他“具体改哪里”
  - 由用户自己完成修改
- 因此新的 AI 应默认采用：
  - 先读代码
  - 再给最小下一步
  - 每一步都附理由
  - 每一步改完都要求看日志再进入下一步

### 10.2 当前最适合的推进节奏

1. 先通过编译
   - 当前第一阻塞优先级永远高于“继续想新功能”
2. 编译通过后再做 runtime 验证
3. 每一步只新增一层最小功能：
   - 先 cost
   - 再输出响应
   - 再执行质量
   - 最后才是稳定行走 / WBC

### 10.3 当前接手第一步

新的 AI 接手后，**第一件事不是重新分析整个架构**，而是：

- 打开：
  - `src/FSM/StateNMPC.cpp`
  - `include/unitree_guide_controller/FSM/StateNMPC.h`
  - `src/control/CtrlComponent.cpp`
- 先确认当前代码里仍存在：
  - `x0 base / x1 base / delta base` 日志
  - `support target z / measured z / pos_error z` 日志
  - estimator `base rpy` 日志
  - `candidate joint_pos / candidate joint_vel` 日志
- 再确认当前 `StateNMPC` 里：
  - 零输入时走的是 `applySupportTorque()`
  - 非零输入时才走 NMPC hybrid 执行链
  - `support` 侧当前是否只锁 `yaw`，没有显式锁 `roll / pitch`
- 然后重新编译并运行验证：
  - `colcon build --packages-select go1_description unitree_guide_controller --symlink-install --event-handlers console_direct+`
- 重点观察：
  - `planned mode: 15`
  - `target cmd vx / vy / yaw_rate`
  - `estimator base rpy`
  - `support target z / measured z / pos_error z`
  - `optimized input[0]`
  - `candidate joint_pos[0]`
  - `candidate joint_vel[0]`

如果此时仍稳定复现：

- `pitch ≈ -0.34 rad`
- `measured z` 明显低于 `support target z`
- `optimizedState / optimizedInput` 却仍然平滑

则默认进入下一步：

- 优先改造 `support` 姿态目标，使其显式锁住 `roll / pitch / yaw`
- 而不是再回头重复怀疑 solver / mode / reference trajectory

### 10.4 当前推荐的带用户推进方式

新的 AI 接手时，应保持和本轮一致的推进方式：

1. 先说明当前判断
   - 例如：
     - “reference 已完成验证，当前重点转向 `Q/R` 与 execution”
2. 每次只给一个最小修改
   - 不要一次布置很多文件
3. 每次都给：
   - 具体文件
   - 具体函数或配置段
   - 具体替换代码或参数值
4. 每次都解释：
   - 为什么先做这一改
   - 改完后应该看哪几行日志
5. 用户改完后，再基于日志进入下一步

### 10.5 当前最现实的后续路线

当前后续路线已经较此前进一步收敛，建议按下面顺序推进：

1. 先把 `support torque` 的姿态目标补完整
   - 从“只控 `yaw`”升级成“控完整 `roll / pitch / yaw`”
2. 再补低频诊断日志
   - `d_wbd`
   - 每条腿足端 `fz`
   - 不止一个关节的 torque 摘要
3. 重新验证零输入站立是否从“有时能站住”提升为“进入条件变化时也能稳住”
4. 只有在完整姿态目标仍无明显改善时，再继续考虑：
   - 更复杂的约束/步态问题
   - position / kp / kd 协同执行
   - 更后续的 WBC / torque feedforward 执行层

如果用户问“大方向上还剩什么才能让狗动起来”，新的 AI 应明确回答：

1. 先把零输入站立做成鲁棒能力，而不是只在部分初始姿态下能成立
2. 然后判断当前 hybrid 执行层是否足以驱动 Gazebo 里的 Go1
3. 如果不够，再补最小执行配套：
   - `joint_position_command_interface_`
   - `joint_kp_command_interface_`
   - `joint_kd_command_interface_`
4. 再继续看 gait / constraints / mode schedule 是否真的支持迈步
5. 最后才进入更完整的 tracking problem 或 WBC

---

## 11. 当前控制框架与 NMPC 数学背景

这一节用于给后续 AI 或开发者提供统一背景。它不是“下一步怎么改”的清单，而是总结：

- 当前系统到底在跑什么控制链路
- 当前 NMPC 的状态、输入、模型分别是什么
- 当前系统为什么“能站稳但还不能自然走起来”

后续所有分析都应优先以这里为准。

### 11.1 当前整体控制框架

截至当前，`unitree_guide_controller` 中的 `StateNMPC` 并不是一个“单一输出、单一执行器”的纯 NMPC 落地结构，而是一个**过渡性的混合执行框架**。

当前流程可以概括为：

1. 用户输入速度命令
2. `CtrlComponent::updateTargetTrajectoriesFromCommand()` 生成 `TargetTrajectories`
3. `CtrlComponent` 调用 `MPC_MRT_Interface::evaluatePolicy(...)`
4. 求解器返回：
   - `optimized_state`
   - `optimized_input`
   - `planned_mode`
5. `StateNMPC::run()` 根据当前工况执行两条下层链路：
   - `applySupportTorque(...)`
   - `applyNmpcJointHybridCommand()`
6. 最终把 torque / joint position / joint velocity / kp / kd 写入控制接口
7. 机器人状态由 estimator 更新，再反馈给 MPC

因此，当前系统并不是“NMPC 直接输出唯一控制量，然后原封不动执行”，而是：

- 一条 `support torque` 支撑稳定链
- 一条 `joint hybrid command` 关节执行链

共同参与当前控制。

### 11.2 当前 `support torque` 链的角色

`applySupportTorque(...)` 的本质是：

- 根据机身位置/姿态误差构造期望机身加速度与角加速度
- 调用 `BalanceCtrl::calF(...)`
- 解出满足接触约束的足端力
- 再通过 `J^T f` 转为关节力矩

对应代码位置：

- [StateNMPC.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateNMPC.cpp)
- [BalanceCtrl.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/BalanceCtrl.cpp)

`BalanceCtrl::calF(...)` 的输入不是求解器直接给出的控制量，而是：

- 期望线加速度 `ddPcd`
- 期望角加速度 `dWbd`
- 当前足端相对机身位置
- 当前接触模式 `contact`

然后在 QP 中求解接触力。

因此当前 `support torque` 更像：

- 机身稳定器
- 支撑腿力分配器

它非常适合：

- 零命令稳站
- `STANCE`
- full contact 支撑

但它并不是完整 locomotion 执行层。

### 11.3 当前 `nmpc hybrid` 链的角色

`CtrlComponent::evaluateCurrentPolicy()` 调用：

```cpp
mpc_mrt_interface_->evaluatePolicy(
    observation_.time,
    observation_.state,
    optimized_state,
    optimized_input,
    planned_mode);
```

然后做两件事：

1. 从 `optimized_state` 中截取关节角，缓存到：
   - `nmpc_joint_position_cmd_`
2. 从 `optimized_input` 中截取关节速度段，缓存到：
   - `nmpc_joint_velocity_cmd_`

对应代码：

- [CtrlComponent.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/CtrlComponent.cpp:447)

随后 `StateNMPC::applyNmpcJointHybridCommand()` 会把：

- `joint_pos_cmd`
- `joint_vel_cmd`
- `kp/kd`

写入关节接口。

因此当前 `hybrid` 链本质上是：

- 关节参考执行器

而不是“直接执行 MPC 求出的接触力”的纯支撑腿控制器。

### 11.4 当前 NMPC 采用的模型类型

当前自有 `LeggedInterface` 明确创建的是：

- `CentroidalModelInfo`
- `LeggedRobotDynamicsAD`

对应代码：

- [LeggedInterface.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/nmpc/LeggedInterface.cpp:334)
- [LeggedInterface.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/nmpc/LeggedInterface.cpp:168)

同时 `task.info` 中：

```info
centroidalModelType 0
```

而 OCS2 定义：

- `0 = FullCentroidalDynamics`
- `1 = SingleRigidBodyDynamics`

对应源码：

- [CentroidalModelInfo.h](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/include/ocs2_centroidal_model/CentroidalModelInfo.h:47)

因此当前运行的模型不是 SRBD，而是：

**Full Centroidal Dynamics NMPC**

### 11.5 当前 NMPC 的状态定义

OCS2 源码中给出的状态定义为：

\[
x =
\begin{bmatrix}
h_{\mathrm{lin}}/m \\
h_{\mathrm{ang}}/m \\
p_b \\
\theta_{zyx} \\
q_j
\end{bmatrix}
\]

即：

- 质心线动量除以总质量
- 质心角动量除以总质量
- base 位置
- base 的 ZYX 欧拉角
- 关节角

依据：

- [PinocchioCentroidalDynamicsAD.h](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/include/ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h:35)
- [CentroidalModelPinocchioMapping.h](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/include/ocs2_centroidal_model/CentroidalModelPinocchioMapping.h:35)

对当前 Go1 配置：

- `generalizedCoordinatesNum = 18`
- `actuatedDofNum = 12`
- `stateDim = generalizedCoordinatesNum + 6 = 24`

依据：

- [FactoryFunctions.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/FactoryFunctions.cpp:95)

这和运行日志中的：

- `optimized state dim: 24`

一致。

### 11.6 当前 NMPC 的输入定义

OCS2 源码给出的输入定义为：

\[
u =
\begin{bmatrix}
f_{\mathrm{contacts}} \\
w_{\mathrm{contacts}} \\
\dot q_j
\end{bmatrix}
\]

即：

- 接触力
- 接触力矩（如果是 6DoF 接触）
- 关节速度

依据：

- [PinocchioCentroidalDynamicsAD.h](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/include/ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h:35)

当前机器人只有 4 个 3DoF 足接触，没有 6DoF 接触，因此实际退化为：

\[
u =
\begin{bmatrix}
f_{FL} \\
f_{FR} \\
f_{RL} \\
f_{RR} \\
\dot q_j
\end{bmatrix}
\in \mathbb{R}^{24}
\]

其中：

- 每条足端力 \(f_i \in \mathbb{R}^3\)
- 关节速度 \(\dot q_j \in \mathbb{R}^{12}\)

输入维度关系：

\[
\text{inputDim} = n_{\text{act}} + 3n_{3\text{DoF}} + 6n_{6\text{DoF}}
\]

依据：

- [FactoryFunctions.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/FactoryFunctions.cpp:96)

当前配置下：

\[
\text{inputDim} = 12 + 3 \times 4 = 24
\]

这和日志中的：

- `optimized input dim: 24`

一致。

### 11.7 当前 NMPC 连续动力学方程

OCS2 中实际使用的是：

\[
\dot x = f(x,u)
\]

在 [PinocchioCentroidalDynamicsAD.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamicsAD.cpp:79) 里，状态导数被分成两部分：

\[
\dot x =
\begin{bmatrix}
\dot{\bar h} \\
\dot q
\end{bmatrix}
\]

其中 \(\bar h = h/m\)。

#### 11.7.1 动量动力学

对 3DoF 接触足，`getNormalizedCentroidalMomentumRate(...)` 的实现为：

\[
\dot h =
\begin{bmatrix}
mg \\
0
\end{bmatrix}
+
\sum_i
\begin{bmatrix}
f_i \\
r_i \times f_i
\end{bmatrix}
\]

归一化后：

\[
\dot{\bar h}_{\mathrm{lin}} = g + \frac{1}{m}\sum_i f_i
\]

\[
\dot{\bar h}_{\mathrm{ang}} = \frac{1}{m}\sum_i r_i \times f_i
\]

其中：

- \(g = [0,0,-9.81]^T\)
- \(r_i\)：从 CoM 指向第 \(i\) 个接触点的位置
- \(f_i\)：第 \(i\) 个接触力

依据：

- [ModelHelperFunctions.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/ModelHelperFunctions.cpp:167)

#### 11.7.2 广义坐标运动学

状态导数的后半部分由：

```cpp
mappingCppAd.getPinocchioJointVelocity(state, input)
```

生成，见：

- [PinocchioCentroidalDynamicsAD.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamicsAD.cpp:90)

对于 `FullCentroidalDynamics`，源码给出：

\[
v_b = A_b^{-1}(h - A_j \dot q_j)
\]

然后：

\[
v =
\begin{bmatrix}
v_b \\
\dot q_j
\end{bmatrix}
\]

其中：

- \(A(q)\)：centroidal momentum matrix
- \(A_b\)：其前 6 列，对应浮动基
- \(A_j\)：其后部，对应关节部分

依据：

- [CentroidalModelPinocchioMapping.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/CentroidalModelPinocchioMapping.cpp:73)

因此，当前模型可以理解为：

\[
\dot x =
\begin{bmatrix}
g + \frac{1}{m}\sum_i f_i \\
\frac{1}{m}\sum_i r_i \times f_i \\
v_b(h,q,\dot q_j) \\
\dot q_j
\end{bmatrix}
\]

这里的最后两块更严格地说是：

- base 线速度
- base 欧拉角导数
- 关节速度

共同组成 generalized coordinates 的时间导数。

### 11.8 当前求解器到底“输出什么”

这一点必须分成两层理解。

#### 11.8.1 从优化问题本身看

当前求解器优化的输入变量是：

\[
u^\star =
\begin{bmatrix}
f^\star \\
\dot q_j^\star
\end{bmatrix}
\]

即：

- 接触力
- 关节速度

不是“机体加速度”。

机体加速度是动力学作用后的结果，不是当前 OCS2 问题中的直接输入变量。

#### 11.8.2 从当前控制器的落地执行看

当前控制器并没有把 `optimized_input` 中的接触力段直接完整下发到底层执行，而是：

1. `support torque` 链自己再根据机身误差调用 `BalanceCtrl::calF(...)` 求支撑腿力
2. `hybrid` 链从：
   - `optimized_state` 中取关节角
   - `optimized_input` 中取关节速度
   再写入 joint command / gains

因此，当前真实执行量是：

- 一部分是 `J^T f` 形成的关节力矩
- 一部分是 joint position / joint velocity / PD gains

这也是为什么当前系统被定义为“过渡性的混合执行框架”。

### 11.9 当前为什么说它是“非线性 MPC”

原因有两个，都有直接源码依据：

1. 求解器创建的是：
   - `ocs2::SqpMpc`
   - 见 [CtrlComponent.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/CtrlComponent.cpp:204)
2. 动力学使用：
   - `LeggedRobotDynamicsAD`
   - 其内部通过 `CppAdInterface` 对系统流形
     \(\dot x = f(x,u)\)
     做一阶自动微分线性化
   - 见 [PinocchioCentroidalDynamicsAD.cpp](/home/xiangh9/xhros2/uni_ws/src/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamicsAD.cpp:35)

因此当前系统不是线性 MPC，而是：

**基于非线性 centroidal dynamics 的 SQP 型 NMPC**

### 11.10 当前模型“是否简化”

答案是：

- 相对于完整刚体 torque-level NMPC，它是**简化模型**
- 但相对于更粗糙的单刚体模型，它又不是最简版

更准确地说，当前模型是：

- `FullCentroidalDynamics`
- 保留：
  - centroidal momentum
  - 浮动基位姿
  - 关节位形
  - 接触力
  - 关节速度输入
- 但没有把“完整关节力矩级刚体动力学”直接作为优化模型

因此最佳表述是：

**当前使用的是非线性的全质心动力学降阶模型，而不是完整的全刚体力矩级 NMPC。**

### 11.11 这一背景对后续架构设计的直接启发

基于上面的状态、输入和动力学定义，当前最自然的执行层分工应该是：

1. 支撑腿：
   - 以接触力 \(f_i\) 为核心物理量
   - 对应：
     - `MPC / force distribution`
     - `J^T f`
2. 摆动腿：
   - 不应继续复用同一套支撑腿控制律
   - 更自然地走：
     - `Swing Trajectory`
     - `IK`
     - `PD`

这与当前文档前面已经确认的新主线是一致的：

- 不能继续把所有腿都塞进同一种执行律
- 应逐步从“support torque + hybrid 叠加”迁移到：
  - `stance leg -> force control`
  - `swing leg -> trajectory tracking`

### 11.12 后续 AI 接手时必须牢记的几点

1. 当前 `optimized_input` 并不等于“直接可落地的唯一控制量”
2. 当前求解器内部已经包含接触力变量
3. 当前失败点主要不在“没有 force”
4. 当前失败点主要在：
   - 执行层没有按腿区分 swing / stance
   - 求解器输出与落地执行结构尚未真正对齐
5. 所以后续修改优先级应是：
   - 先整理执行层职责
   - 再调 `Q/R`
   - 再调细节增益与 clamp

### 11.13 `StateTrotting` 与 `gait/` 目录中的现成摆动腿链路

在决定后续路线之前，已经专门调研了：

- [StateTrotting.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp)
- [WaveGenerator.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/gait/WaveGenerator.cpp)
- [GaitGenerator.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/gait/GaitGenerator.cpp)
- [FeetEndCalc.cpp](/home/xiangh9/xhros2/uni_ws/src/quadruped_ros2_control/controllers/unitree_guide_controller/src/gait/FeetEndCalc.cpp)

调研结论是：

**当前项目中其实已经存在一条相对完整、工程分层清晰的摆动腿控制链路。**

#### 11.13.1 `WaveGenerator` 的职责

`WaveGenerator` 提供：

- 每条腿当前是否接触地面：`contact_(i)`
- 每条腿当前相位：`phase_(i)`
- 步态状态：
  - `STANCE_ALL`
  - `SWING_ALL`
  - `WAVE_ALL`

它不是简单的“接触开关”，而是：

- 接触判定
- 相位推进
- 状态切换平滑

三者的统一 gait source。

#### 11.13.2 `FeetEndCalc` 的职责

`FeetEndCalc` 不负责生成整条摆动轨迹，而负责：

- 根据当前机身速度
- 目标机身速度
- 目标转向角速度
- 摆动相位

估计摆动腿下一步的**落脚终点**。

也就是说，它的角色是：

- foothold / next-step endpoint 估计器

不是轨迹器。

#### 11.13.3 `GaitGenerator` 的职责

`GaitGenerator` 以：

- `WaveGenerator` 给出的 `contact / phase`
- `FeetEndCalc` 给出的落脚终点

为输入，输出：

- 四条腿的目标足端位置
- 四条腿的目标足端速度

对于支撑腿：

- 足端目标固定在原地
- 目标速度为 0

对于摆动腿：

- 采用摆线轨迹
- 生成平滑的 `feet_pos` 和 `feet_vel`

因此它的本质是：

- swing foot trajectory generator

#### 11.13.4 `StateTrotting` 的摆动腿控制方式

`StateTrotting` 的摆动腿逻辑不是“简单关节跟踪”，而是完整的：

1. `WaveGenerator` 判定接触与相位
2. `GaitGenerator` 生成摆动足目标位置/速度
3. `calcTau()` 中：
   - 支撑腿使用 `BalanceCtrl::calF(...)` 的力分配结果
   - 摆动腿则用足端空间 PD：
     \[
     f_{swing} = K_p (p_{foot}^{des} - p_{foot}) + K_d (\dot p_{foot}^{des} - \dot p_{foot})
     \]
4. `calcQQd()` 中：
   - 目标足端位置通过 `robot_model_->getQ(...)` 做 IK
   - 目标足端速度通过 `robot_model_->getQd(...)` 求关节速度
5. `calcGain()` 中：
   - 摆动腿和支撑腿使用不同关节增益

这条链路说明：

**`StateTrotting` 已经具备比当前 `StateNMPC` 更完整、更清晰的 swing leg 执行层。**

#### 11.13.5 这次调研带来的工程结论

这次调研后，后续路线发生了明确调整：

1. 不再建议在 `StateNMPC` 内继续把：
   - gait 相位
   - 接触判定
   - 摆动足目标
   - 落脚点计算
   全部重新发明一遍
2. 最大程度复用现有：
   - `WaveGenerator`
   - `FeetEndCalc`
   - `GaitGenerator`
   - `StateTrotting` 的 swing 执行思想
3. `StateNMPC` 应逐步收敛为：
   - NMPC 观测与参考更新
   - gait 执行调度
   - stance / swing 控制汇合点
4. 不建议长期保留“在 `StateNMPC` 里直接以 `planned_mode` 为唯一 gait 执行源”的做法
   - `planned_mode` 仍然是上层 NMPC 的重要输出
   - 但 gait / phase / swing trajectory 更适合继续由 `gait/` 模块承担

#### 11.13.6 当前推荐的集成方向

基于这次调研，当前推荐的工程路线是：

1. `StateNMPC` 继续保留：
   - `planned_mode`
   - NMPC policy 读取
   - stance 力分配主链
2. 逐步引入统一 gait source：
   - 优先复用 `WaveGenerator`
   - 进一步复用 `GaitGenerator`
3. 摆动腿目标生成尽量走：
   - `phase`
   - `FeetEndCalc`
   - `cycloid swing trajectory`
   - `IK + PD`
4. 支撑腿继续走：
   - `BalanceCtrl`
   - `J^T f`

最终目标不是把 `StateTrotting` 和 `StateNMPC` 粗暴拼在一起，而是：

**复用 `gait/` 目录中的成熟步态模块，形成一个层次更清楚的 NMPC locomotion 执行框架。**

### 11.14 Current vs Target Architecture

#### 11.14.1 Current As-Is

当前 `StateNMPC` 仍处于过渡性架构阶段，实际运行链路可以概括为：

1. 键盘/上层命令进入 `CtrlComponent::updateTargetTrajectoriesFromCommand()`
2. NMPC/OCS2 读取当前 observation 与 target，输出：
   - `optimized_state`
   - `optimized_input`
   - `planned_mode`
3. `StateNMPC` 负责：
   - 读取 NMPC policy
   - 当前已直接接入：
     - `wave_generator_`
     - `gait_generator_`
   - 在迁移阶段同时持有：
     - planner side mode information
     - execution side gait source
   - 调度支撑腿与摆动腿执行链
4. 支撑腿当前主要走：
   - `applySupportTorque(...)`
   - `BalanceCtrl`
   - `J^T f`
5. 摆动腿当前主要走：
   - `applySwingLegControl(...)`
   - `gait_generator_` 生成的足端目标
   - `robot_model_->getQ(...) / getQd(...)`
   - joint-space `q / qd / kp / kd`

这一版已经不只是“按腿角色分流”的第一步，而是已经完成了：

- `WaveGenerator` 接入 `StateNMPC`
- `GaitGenerator` 接入 `StateNMPC`
- swing 腿从“纯 NMPC joint-space fallback”
  迁移到“gait foot target -> IK -> joint tracking”

当前真正尚未完成的，不再是“是否接 gait”，而是：

- gait source 统一
- `planned_mode` 的职责降级
- 执行层结构继续从过渡实现收敛

#### 11.14.2 Target To-Be

目标架构不应再让 `StateNMPC` 自己堆叠 gait 细节，而应收敛为清晰的分层结构：

1. `CtrlComponent + NMPC`
   - 负责 observation、reference、policy、mode schedule
2. `gait/` 子系统
   - 负责 contact / phase / foothold / swing trajectory
3. `StateNMPC`
   - 负责调度
   - 负责汇合 stance / swing 两条执行链
   - 不再承担完整 gait 生成职责
4. `stance execution`
   - 负责支撑腿力分配与 `J^T f`
5. `swing execution`
   - 负责 swing foot target
   - `IK + PD`
   - 或进一步升级为更完整的足端空间控制

从工程边界上看，目标不是“让 `StateNMPC` 更聪明”，而是“让各层职责更单一、更可替换”。

### 11.15 Module Responsibilities and Boundaries

#### 11.15.1 NMPC / OCS2

职责：

- 维护优化问题与求解器
- 接收 observation 与 `TargetTrajectories`
- 输出：
  - `optimized_state`
  - `optimized_input`
  - `planned_mode`

不建议承担：

- 完整 gait 相位管理
- swing foot trajectory 细节
- 末端执行器级别的完整摆动腿控制

#### 11.15.2 `gait/` 子系统

职责：

- 统一管理 gait/contact/phase 信息
- 统一生成摆动足轨迹
- 统一估计落脚点

推荐复用模块：

- `WaveGenerator`
- `FeetEndCalc`
- `GaitGenerator`

不建议让这些逻辑长期散落在 `StateNMPC.cpp` 的局部辅助函数中。

#### 11.15.3 `StateNMPC`

职责：

- 管理 NMPC 状态机入口
- 接收求解器输出
- 调度：
  - stance leg control
  - swing leg control
- 汇总控制命令并下发到接口

不建议长期承担：

- 独立完整 gait source
- 独立完整 swing trajectory generator
- 大量与 `StateTrotting` 重复的步态细节

#### 11.15.4 stance execution

职责：

- 只服务当前 `STANCE` 腿
- 根据机身误差与接触状态做力分配
- 将足端力映射为关节力矩：
  - `J^T f`

当前可复用链路：

- `BalanceCtrl`
- `applySupportTorque(...)`

#### 11.15.5 swing execution

职责：

- 只服务当前 `SWING` 腿
- 使用统一 gait source 生成：
  - swing phase
  - foothold / target foot position
  - swing foot velocity
- 将足端目标转为：
  - `q_des`
  - `qd_des`
  - `kp/kd`

推荐直接借鉴：

- `StateTrotting::calcQQd()`
- `robot_model_->getQ(...)`
- `robot_model_->getQd(...)`

### 11.16 Transition State and Technical Debt

当前最大问题已经不再是单纯参数，而是过渡性实现仍然存在以下技术债：

1. gait source 尚未统一
   - `planned_mode` 已经可用
   - `WaveGenerator / GaitGenerator` 已经正式接入 `StateNMPC` 主链
   - 但当前仍处于 `planned_mode` 与 `wave_generator_` 双源并存阶段
2. swing 腿还没有完全迁移到足端目标驱动
   - 当前虽然已经按腿分流
   - 且已开始使用 gait foot target + IK
   - 但整体还处于第一版迁移状态，尚未完成稳定化
3. `StateNMPC` 仍承担过多中间层逻辑
   - 如果继续在其中直接堆 `planned_mode -> contact -> phase -> swing target`
     会持续恶化文件层次
4. 当前实现中存在“为跑通而保留”的过渡逻辑
   - 应明确标记为 temporary，而不是继续扩大其职责范围

因此，后续工作主线应优先是“消除结构性技术债”，而不是先继续做局部 gain 调参。

### 11.17 Data Flow View

从运行时数据流角度，当前推荐用下面这条链理解系统：

1. User input
2. `TargetTrajectories`
3. NMPC / OCS2
4. 输出：
   - `optimized_state`
   - `optimized_input`
   - `planned_mode`
5. gait dispatcher
   - 统一生成：
     - contact
     - phase
     - leg roles
     - swing foot targets
6. execution split
   - stance leg:
     - `BalanceCtrl`
     - `J^T f`
   - swing leg:
     - `IK + PD`
7. robot
8. estimator
9. feedback to：
   - NMPC
   - gait subsystem

当前与目标之间的差距主要集中在第 5 步：`gait dispatcher` 还没有从 `StateNMPC` 中完全抽象出来。

### 11.18 Reuse Strategy

后续实现应遵循以下复用优先级：

#### 11.18.1 Direct Reuse

优先直接复用：

- `WaveGenerator`
- `FeetEndCalc`
- `GaitGenerator`
- `QuadrupedRobot::getQ(...)`
- `QuadrupedRobot::getQd(...)`
- `StateTrotting` 中 swing leg 的控制分层思路

#### 11.18.2 Keep but Reposition Responsibilities

保留但调整职责定位：

- `StateNMPC`
  - 从“局部实现越来越多 gait 逻辑”
    收敛为“执行调度器”
- `BalanceCtrl`
  - 继续作为 stance 主链核心

#### 11.18.3 Transitional Logic to Gradually Retire

以下逻辑可以短期保留，但不应继续扩展：

- `StateNMPC` 内部大量基于 `planned_mode` 的局部 gait 派生逻辑
- 纯 joint-space 的 swing fallback
- “先在 `StateNMPC` 里补一个最小 gait 实现再说”的继续扩张

### 11.19 Recommended Next Implementation Order

基于当前工程化判断，推荐后续实现顺序如下：

1. 先完成 gait source 统一
   - 明确 `planned_mode` 是 NMPC 模式输出与监视信号
   - 明确 `wave_generator_->contact_ / phase_` 是执行层 gait source
   - 明确 `gait/` 是 gait / phase / swing trajectory 的主要承担者
2. 将 `StateNMPC` 中与 gait 强相关的新增逻辑控制在最小范围
   - 优先通过复用外部模块完成
3. swing 目标生成链已经完成第一版接入
   - `WaveGenerator`
   - `FeetEndCalc`
   - `GaitGenerator`
   - `getQ / getQd`
   后续重点转为稳定化与统一参数源，而不是是否接入
4. 继续保留 stance 主链稳定性
   - 不在 swing 改造阶段同时大改 `BalanceCtrl`
5. 在 gait source 统一后，再继续做：
   - `planned_mode` 与 `wave contact` 的一致性监视
   - gait 参数与 NMPC template 的节奏对齐

这意味着接下来的 gait 修改，不应以“继续往 `StateNMPC` 里塞局部逻辑”为方向，而应以“把 `gait/` 目录提升为正式依赖模块”为方向。

### 11.20 Latest Progress Snapshot (2026-05-08)

截至当前，已经完成的关键进度如下：

1. `StateNMPC` 已正式接入：
   - `wave_generator_`
   - `gait_generator_`
2. 非零命令下，`StateNMPC` 已调用：
   - `gait_generator_.setGait(...)`
   - `gait_generator_.generate(...)`
3. 摆动腿控制链已经从：
   - `NMPC joint-space q/qd`
   迁移为：
   - `gait foot target`
   - `world -> body` 目标转换
   - `robot_model_->getQ(...)`
   - `robot_model_->getQd(...)`
   - `joint position / velocity / swing gains`
4. 当前日志已经可以稳定观测：
   - `gait target foot`
   - `gait target foot vel`
   - `first swing foot target body`
   - `first swing joint q/qd from gait IK`
   - `gait` 与 `NMPC joint-space` 的对照

这说明当前阶段的核心变化是：

- gait 接入已经从计划变成了可运行实现
- 当前最重要的问题已经从“如何把 gait 接进来”
  转移为“如何把执行层统一到单一 gait source”

### 11.21 Immediate Next Step

当前最优先的下一步是：

1. 让执行层 gait source 统一到：
   - `wave_generator_->contact_`
   - `wave_generator_->phase_`
2. 将 `planned_mode` 明确降级为：
   - 上层 NMPC 模式输出
   - 监视/一致性检查信号
3. 在 `run()` 中增加低频对照日志：
   - `planned-mode contact`
   - `wave contact`
   - `gait source consistency`

完成这一轮后，系统结构会更清楚：

- `planned_mode` 负责 planner-side information
- `wave_generator_` 负责 execution-side gait source
- `gait_generator_` 负责 swing trajectory generation

### 11.22 Unified Gait Scheduler Direction (2026-05-08)

后续必须明确收敛到一个统一的 gait scheduler，而不是长期维持：

- planner side:
  - `planned_mode`
- execution side:
  - `wave_generator_->contact_ / phase_ / status_`

双源并存。

统一原则如下：

1. 执行层唯一 gait 真相源
   - 第一阶段以 `WaveGenerator` 为 execution-side single source of truth
   - `StateNMPC`、`GaitGenerator`、stance/swing dispatch 都消费同一份：
     - `status`
     - `contact`
     - `phase`
2. `planned_mode` 降级为 planner-side monitor
   - 当前只用于：
     - 日志
     - consistency check
     - 后续与上层 schedule 对齐
   - 不再直接驱动本地腿角色/接触执行决策
3. 最终目标不是“永远靠本地 WaveGenerator”
   - 最终应形成一个更上层的 unified gait scheduler
   - 同时向：
     - NMPC 提供 horizon 上的 `mode schedule`
     - execution layer 提供当前周期的 `status/contact/phase`
4. 第一阶段最小落地
   - 先在 `StateNMPC` 中显式引入 execution gait scheduler state
   - 把分散在 `run()` 中的：
     - zero-command override
     - wave status 切换
     - contact/phase 读取
     - leg role 派生
     收敛为一个统一更新步骤
   - 这是过渡层，不是最终上层 scheduler 终态

### 11.23 Unified Gait Scheduler Phase-1 Implementation Notes

第一阶段代码实施的目标不是马上重写 OCS2 的 mode schedule，而是先把执行层 gait 调度做成明确边界：

1. `StateNMPC` 内维护一个 `execution_gait_state`
   - `requested_status`
   - `active_status`
   - `contact`
   - `phase`
   - `leg_roles`
   - `zero_command_override`
2. 每个控制周期内，`StateNMPC` 先同步这份 execution gait state
   - 然后 stance/swing 控制、gait 日志、consistency monitor 都只读这份状态
3. 这样做的收益
   - 避免 gait 决策散落在 `run()`、`applySupportTorque()`、`applySwingLegControl()` 各处
   - 为后续把 scheduler 上移到 `CtrlComponent` 或更上层模块提供清晰接口
   - 为后续把 planner-side `mode schedule` 和 execution-side wave state 对齐创造过渡层

下一阶段才考虑：

- 让 unified scheduler 同时生成：
  - OCS2/LeggedInterface 需要的 horizon mode schedule
  - execution layer 需要的当前 `contact/phase`

### 11.24 How Gait Information Enters NMPC

后续实现 unified gait scheduler 时，必须先统一对“步态信息如何参与 NMPC 求解”的理解。

#### 11.24.1 Gait Information Does Not Only Affect Execution

步态信息不只是执行层的：

- `STANCE / SWING` 标签
- 摆动腿轨迹相位

它还会直接改变 NMPC 的问题定义。

具体来说，gait / mode schedule 会决定：

1. 哪些腿当前是有效接触点
   - `STANCE` 腿允许提供地面反力
   - `SWING` 腿不应承担支撑力
2. 哪些接触力输入分量有效
   - stance legs: contact force variables are active
   - swing legs: contact-force-related constraints should be inactive / zeroed
3. 哪些约束在当前 mode 下启用
   - force limits
   - friction constraints
   - contact consistency
4. 未来预测时域的接触切换时机
   - 这直接决定 NMPC horizon 上的 mode transitions

因此，后续 unified gait scheduler 的设计目标不是只给执行层一个 `contact_/phase_`，而是要给 NMPC 一个可消费的 planner-side mode schedule。

#### 11.24.2 What NMPC Consumes from Gait

从 NMPC 的角度，它需要从 gait scheduler 获得的是：

1. 当前时刻的 mode
2. 未来预测时域上的 `mode schedule`
3. 每个 mode 的接触模式
4. mode 切换时间 / event times

它并不直接需要“某条腿当前的 cycloid 轨迹点”，但它必须知道：

- 哪些腿当前支撑
- 哪些腿未来会切成摆动
- 哪些腿未来会重新着地

#### 11.24.3 What Execution Consumes from Gait

执行层需要从 gait scheduler 获得的是：

1. 当前周期的 `status`
2. 当前周期的 `contact`
3. 当前周期的 `phase`
4. 摆动腿轨迹推进所需的 swing timing

因此 planner-side 与 execution-side 消费的是同一个 gait truth 的不同投影：

- planner side: horizon schedule
- execution side: instantaneous contact/phase

#### 11.24.4 Support Legs and Swing Legs Affect NMPC Differently

后续设计中，必须明确区分：

1. `STANCE` legs
   - 这些腿是可用接触点
   - NMPC 可以利用这些腿提供接触力
   - 这些腿影响：
     - contact-force-related inputs
     - balance feasibility
     - support polygon / support geometry
2. `SWING` legs
   - 这些腿当前不参与支撑
   - 它们的接触力不应被 NMPC 当成有效支撑输入
   - 它们主要影响：
     - 可用接触集合减少
     - 下一落脚时段的 mode transition
     - execution-side swing trajectory tracking

换句话说：

- `STANCE` 腿主要通过“接触力可用性”影响 NMPC
- `SWING` 腿主要通过“接触切换与未来落脚时机”影响 NMPC

#### 11.24.5 What Outputs We Expect from NMPC vs Gait

必须区分两类输出：

1. unified gait scheduler 输出：
   - mode schedule
   - current contact
   - current phase
   - swing timing
2. NMPC 输出：
   - optimized state
   - optimized input
   - planner-side `planned_mode`

执行层不应要求 NMPC 自己去生成完整的摆动腿足端轨迹细节；更自然的分工是：

- gait scheduler / gait subsystem 负责 swing timing and foot trajectory progression
- NMPC 负责在给定 mode schedule 下优化系统动态与接触使用

### 11.25 Unified Gait Scheduler Three-Phase Roadmap

后续实现统一 gait scheduler，推荐分 3 个阶段推进。

#### 11.25.1 Phase 1: Execution-Side Unification

目标：

- 先让执行层只消费一份 gait state

当前含义：

- `WaveGenerator` 暂时作为 execution-side single source of truth
- `StateNMPC` 内收敛出显式的 execution gait state

当前阶段的责任边界：

- `planned_mode`
  - 只做 monitor / consistency check
- `wave_generator_->status/contact/phase`
  - 决定 execution-side gait state
- `GaitGenerator`
  - 基于 execution gait state 生成 swing foot target

#### 11.25.2 Phase 2: Planner-Side Schedule Unification

目标：

- 不再让 planner side 和 execution side 各自拥有独立 gait 真相源

这一阶段要做的事：

1. 提取一个更上层的 gait schedule definition
   - period
   - stance ratio
   - leg bias / sequence
   - zero-command overrides / special states
2. 由它同时生成：
   - NMPC 需要的 horizon `mode schedule`
   - execution 需要的 instantaneous `status/contact/phase`
3. 将 `planned_mode` 的来源逐步收敛为：
   - unified scheduler 所生成的 planner-side schedule 的当前 mode 视图

这一阶段之后，`wave_generator` 不再是“一个独立真相源”，而更像 unified scheduler 的 execution-side adapter。

#### 11.25.3 Phase 3: Full Scheduler Ownership

目标：

- 建立真正统一的 gait scheduler ownership

最终应达到：

1. 有一个明确模块负责 gait truth
2. planner-side 与 execution-side 不再需要做长期一致性修补
3. `planned_mode` 不再是“另一个来源”，而只是统一 schedule 在当前时刻的 mode 表达
4. `StateNMPC` 退回到：
   - 观测更新
   - scheduler consumption
   - stance/swing execution dispatch

### 11.26 Immediate Practical Implication for Current Work

在当前代码阶段，实施上应坚持以下原则：

1. 不再继续扩展“`planned_mode -> 本地腿角色派生`”逻辑
2. 执行层所有 gait 判断优先收敛到统一 execution gait state
3. `planned_mode` 的价值当前主要体现在：
   - planner monitor
   - full-stance-consistent reference filtering
   - consistency diagnostics
4. 下一步修稳定性问题时，要优先保证：
   - execution-side contact 假设
   - stance hold 参考
   - support torque contact
   三者相互一致

### 11.27 Reference Repo Mapping: `MPC_legged_control`

参考仓库：

- `/home/xiangh9/xhros2/MPC/MPC_legged_control`

当前已经确认，这个仓库最值得借鉴的不是某一个单独的 gait 类，而是它“步态、MPC、摆动腿轨迹、WBC”使用同一份 gait truth 的协同方式。

#### 11.27.1 Reference Workflow

参考仓库中的主链可以概括为：

1. gait command / gait definition
2. `GaitSchedule`
3. `SwitchedModelReferenceManager`
4. `ModeSchedule` for MPC
5. `SwingTrajectoryPlanner`
6. MPC evaluates:
   - optimized state
   - optimized input
   - planned mode
7. WBC consumes:
   - optimized state
   - optimized input
   - planned mode
8. hybrid joint command

它的关键点在于：

- gait 不只是执行层自己用；
- gait 先进入 reference manager；
- 同一份 gait information 同时驱动：
  - MPC 用的 mode schedule
  - swing trajectory planner
  - WBC stance/swing task dispatch

#### 11.27.2 Why It Matters for Current Work

这与当前 `unitree_guide_controller` 的最大差异是：

- 当前仍然存在 planner-side 与 execution-side gait truth 分裂
- `planned_mode` 与 `wave_generator_` 还不是同源

因此当前项目的长期目标，不应是继续“修补 `planned_mode` 与 `wave contact` 的关系”，而应当是：

- 建立一个 unified gait scheduler
- 后续同时服务于：
  - planner-side mode schedule
  - execution-side contact/phase
  - swing trajectory timing

#### 11.27.3 Module Mapping to Current Project

参考仓库中的结构，对应到当前项目的目标映射应理解为：

1. `GaitSchedule` / `SwitchedModelReferenceManager`
   - 对应我们将要建立的 `UnifiedGaitScheduler`
2. `SwingTrajectoryPlanner`
   - 对应当前项目中：
     - `WaveGenerator`
     - `FeetEndCalc`
     - `GaitGenerator`
     后续应统一由 scheduler 驱动
3. WBC stance/swing dispatch
   - 对应当前项目中：
     - `applySupportTorque(...)`
     - `applyStanceLeg...(...)`
     - `applySwingLegControl(...)`

这意味着：

- `UnifiedGaitScheduler` 在当前项目里的地位应当是“上层 gait truth owner”
- `StateNMPC` 应退化为：
  - gait consumer
  - MPC runtime orchestrator
  - stance/swing execution dispatcher

### 11.28 Immediate Goal Shift (2026-05-08)

当前目标已经明确调整为：

**立即优先实现 unified gait scheduler，而不是先把机器人站稳。**

原因：

1. 当前最重要的是整体架构方向正确
2. 如果继续在 `StateNMPC` 里为“先站稳”不断加过渡补丁，会进一步稀释 unified scheduler 的主线
3. 当前已经确认，最终系统需要一个真正单一的 gait truth owner

因此从现在开始，当前阶段的优先级为：

1. 先把 unified gait scheduler 作为独立模块立起来
2. 让 `StateNMPC` 只消费 scheduler 的输出
3. 暂时接受：
   - 机器人还未完全站稳
   - planner-side full horizon mode schedule 还未完成

### 11.29 Unified Gait Scheduler Immediate Build Plan

从当前时点开始，推荐立即按以下顺序实施。

#### 11.29.1 Step 1: Build the Module

新建独立模块：

- `include/unitree_guide_controller/gait/UnifiedGaitScheduler.h`
- `src/gait/UnifiedGaitScheduler.cpp`

第一版先只做：

1. 输入：
   - `zero_command`
   - `planned_mode`
2. 输出：
   - execution-side gait state
   - planner-side gait snapshot
3. 内部处理：
   - 用 `WaveGenerator` 产生当前 execution gait state
   - 用 `planned_mode` 恢复当前 planner-side contact snapshot

#### 11.29.2 Step 2: Make `StateNMPC` a Consumer

`StateNMPC` 立即停止继续扩张自己的 gait ownership，具体方向是：

1. 不再保留本地 execution gait truth 成员
2. 不再自己维护 wave status / contact / phase 的主逻辑
3. 改为每周期调用：
   - `unified_gait_scheduler_.update(...)`
4. 后续所有 gait 相关执行判断都从 scheduler 读取：
   - `contact`
   - `phase`
   - planner-side current mode snapshot

#### 11.29.3 Step 3: Keep Phase-1 Scope Controlled

第一版 unified scheduler **暂时不做**：

1. OCS2 horizon `ModeSchedule` 生成
2. planner-side full future gait schedule 注入
3. stance-hold 稳定性补丁优化

第一版统一 scheduler 只做一件最重要的事：

**把 gait truth ownership 从 `StateNMPC` 中抽出来。**

#### 11.29.4 Step 4: Next Phase After This

等 Phase-1 unified scheduler 成型后，再进入下一阶段：

1. planner-side horizon mode schedule generation
2. execution-side contact/phase 与 planner-side schedule 真正同源
3. 再处理 zero-command 站立稳定性与 stance reference 一致性问题
