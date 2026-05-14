# 第一阶段实验操作手册

## 1. 这份手册的目标

这份手册只服务于第一阶段目标：

- 先把问题看清楚
- 先把实验流程固定下来
- 先把估计器和控制器的表现客观记录下来

当前阶段不追求：

- 直接把 trotting 调稳
- 直接上 MPC
- 直接上 WBC

当前阶段追求的是：

- 日志能稳定采
- baseline 能稳定复现
- 实验结论能重复验证

## 2. 第一阶段总原则

这阶段始终遵循三条原则：

1. 先记录，再下结论。
2. 先和 Gazebo 真值对比，再讨论估计器好坏。
3. 先搞清楚“谁先发散”，再决定先改估计器还是控制器。

## 3. 当前最关键的实验文件

这一阶段最值得动的文件主要有：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`
- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/BalanceCtrl.cpp`

它们各自的角色：

- `Estimator.cpp`
  - 记录估计器输出
  - 观察 IMU、姿态、速度、位置估计
- `StateTrotting.cpp`
  - 记录 body target、tracking error、控制输出、步态状态
- `BalanceCtrl.cpp`
  - 后面如果要看接触力分配是否异常，这里是最合适的补充观察点

## 4. 第一阶段必须记录的日志项

建议统一按三大类记录。

## 4.1 估计器日志

建议记录这些量：

- 时间戳
- IMU 四元数
- IMU gyro
- IMU acc
- 估计的 base position
- 估计的 base velocity
- 估计的 yaw
- 接触状态

这些量已经有相当一部分在你现在的 `Estimator.cpp` 调试输出里出现过。

### 建议最终统一成的字段

建议未来统一字段名类似：

- `time`
- `est_pos_x`
- `est_pos_y`
- `est_pos_z`
- `est_vel_x`
- `est_vel_y`
- `est_vel_z`
- `est_quat_w`
- `est_quat_x`
- `est_quat_y`
- `est_quat_z`
- `imu_gx`
- `imu_gy`
- `imu_gz`
- `imu_ax`
- `imu_ay`
- `imu_az`
- `contact_fl`
- `contact_fr`
- `contact_rl`
- `contact_rr`

### 当前建议加日志的位置

文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp`

函数：

- `Estimator::update()`

建议记录位置：

- 完成 `x_hat_` 更新之后
- 完成 low-pass filter 之后
- 此时输出的 `x_hat_` 已经是控制器实际会使用的状态

## 4.2 Trotting 控制器日志

建议记录这些量：

- 时间戳
- `wave_status`
- `pcd_`
- `pos_body_`
- `vel_target_`
- `vel_body_`
- `yaw_cmd_`
- `pos_error_`
- `vel_error_`
- `dd_pcd`
- `d_wbd`
- 各腿 torque

### 建议最终统一成的字段

建议未来统一字段名类似：

- `time`
- `wave_status`
- `pcd_x`
- `pcd_y`
- `pcd_z`
- `body_x`
- `body_y`
- `body_z`
- `target_vx`
- `target_vy`
- `target_vz`
- `body_vx`
- `body_vy`
- `body_vz`
- `yaw_cmd`
- `pos_err_x`
- `pos_err_y`
- `pos_err_z`
- `vel_err_x`
- `vel_err_y`
- `vel_err_z`
- `dd_pcd_x`
- `dd_pcd_y`
- `dd_pcd_z`
- `d_wbd_x`
- `d_wbd_y`
- `d_wbd_z`
- `tau_leg0_hip`
- `tau_leg0_thigh`
- `tau_leg0_calf`

后面如果需要，再逐步扩展到四条腿。

### 当前建议加日志的位置

文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp`

函数：

- `StateTrotting::run()`
- `StateTrotting::calcTau()`

建议分工：

- `run()` 里主要记录“状态量、目标量、误差量、步态状态”
- `calcTau()` 里主要记录“控制输出量和力矩量”

## 4.3 力分配日志

这一项不是第一优先级，但很值得预留。

建议记录：

- `bd_`
- `F_`
- 接触约束状态

因为后面如果发现：

- 估计器还算正常
- 但 torque / force 分配异常

那么就需要靠这里判断：

- 是 QP 约束有问题
- 还是目标广义力本身就不合理

### 当前建议加日志的位置

文件：

- `src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/BalanceCtrl.cpp`

函数：

- `BalanceCtrl::calF()`

建议记录量：

- `ddPcd`
- `dWbd`
- `bd_`
- `F_`
- `contact`

第一阶段可以先只记录每 100 次或每 200 次一次，不要把终端刷爆。

## 5. Gazebo ground truth 应该怎么拿

第一阶段最重要的工作之一，就是把估计值和 Gazebo 真值放在一起。

## 5.1 为什么必须拿 ground truth

因为只看估计器自己的输出，你很难知道：

- 它是真稳，还是“看起来稳”
- 它是真漂，还是 robot 本体就在动

只有和真值放一起，才能判断：

- 估计器到底准不准
- 控制器到底是在追错目标，还是估计器本来就错了

## 5.2 第一阶段可接受的做法

第一阶段不一定非要写复杂插件，先满足“能拿到真值”即可。

推荐优先尝试以下方向：

1. 查看 Gazebo/ROS 当前是否已经暴露：
   - `/gazebo/model_states`
   - `/gazebo/link_states`
   - `/odom`
   - 其他 base 真值 topic
2. 如果已有 topic，优先直接订阅并记录。
3. 如果没有，再考虑加一个专门的真值发布节点。

## 5.3 第一阶段最想拿到的真值量

至少应该拿到：

- base position
- base linear velocity
- base orientation
- base angular velocity

建议最终字段名类似：

- `gt_pos_x`
- `gt_pos_y`
- `gt_pos_z`
- `gt_vel_x`
- `gt_vel_y`
- `gt_vel_z`
- `gt_quat_w`
- `gt_quat_x`
- `gt_quat_y`
- `gt_quat_z`
- `gt_wx`
- `gt_wy`
- `gt_wz`

## 6. 推荐的日志输出形式

第一阶段不要过度复杂化。

建议按下面顺序演进：

### 第一种：先用控制台调试输出确认字段对不对

优点：

- 速度快
- 立刻可用

缺点：

- 不利于后续自动分析

### 第二种：尽快切到结构化日志

推荐最终形式：

- CSV
- 或者 ROS topic + rosbag

如果你想后续方便画图，我更建议：

- 结构化 CSV
- 或 rosbag 后再转 CSV

### 为什么不建议一直靠 `std::cout`

因为后面一旦要做：

- 多次重复实验
- 画误差曲线
- 对比不同分支

纯终端输出会很快变得难以维护。

## 7. 第一阶段标准实验流程

后续每次实验，建议都尽量按固定流程做。

## 7.1 实验前准备

每次开始前记录：

- 当前 git 分支
- 当前是否有未提交改动
- 当前 robot description
- 当前 launch 参数
- 当前控制频率

这样后面你回看数据时，才知道当时到底跑的是哪版。

## 7.2 实验 1：fixedStand 基线

目标：

- 确认 fixedStand 的稳定程度
- 给 trotting 做对比基线

操作：

1. 启动 Gazebo 与控制器。
2. 切换到 `fixedStand`。
3. 记录 30s。
4. 再记录一轮 60s。

重点看：

- ground truth 下是否基本不漂
- estimator 是否也基本不漂
- roll/pitch 是否稳定

## 7.3 实验 2：trotting 零命令测试

目标：

- 验证 trotting 在没有速度命令时是否仍然不稳

操作：

1. 切换到 `trotting`。
2. 不给速度命令。
3. 记录直到明显失稳或记录满 30s。

重点看：

- `pcd_` 是否自己在漂
- `pos_body_` 是否逐渐偏离
- `dd_pcd` / `d_wbd` 是否逐渐变大
- ground truth 与估计值哪个先明显变坏

## 7.4 实验 3：trotting 低速命令测试

目标：

- 观察在最轻微运动指令下，系统先在哪里出问题

操作：

1. 切换到 `trotting`。
2. 给极小前向速度。
3. 保持短时间。
4. 回到零命令。

重点看：

- 起步瞬间误差是否陡增
- `wave_status` 改变时是否发生突变
- 控制输出是否突然放大

## 7.5 实验 4：扰动测试

目标：

- 看控制器在 baseline 条件下的抗扰性

操作：

1. 在 `fixedStand` 或 `trotting` 零命令状态下施加小扰动。
2. 记录恢复过程。

重点看：

- 姿态误差峰值
- 恢复时间
- 是否出现持续漂移

## 8. 第一阶段推荐优先画的图

后续建议至少先把下面这些图固定下来。

## 8.1 估计器误差图

建议优先画：

- `est_pos_x - gt_pos_x`
- `est_pos_y - gt_pos_y`
- `est_pos_z - gt_pos_z`
- `est_vel_x - gt_vel_x`
- `est_vel_y - gt_vel_y`
- `est_vel_z - gt_vel_z`
- `yaw_est - yaw_gt`

这些图主要是为了回答：

- 估计器在静止时漂不漂
- 在运动时误差会不会累积
- 在接触变化时是否会跳变

## 8.2 body target 跟踪图

建议画：

- `pcd_x` 与 `body_x`
- `pcd_y` 与 `body_y`
- `pcd_z` 与 `body_z`
- `target_vx` 与 `body_vx`
- `target_vy` 与 `body_vy`
- `target_vz` 与 `body_vz`

这些图主要是为了回答：

- 控制器是不是在追一个不合理的目标
- tracking error 是否逐步失控

## 8.3 控制输出图

建议画：

- `dd_pcd_x/y/z`
- `d_wbd_x/y/z`
- `tau_leg0_hip/thigh/calf`

这些图主要是为了回答：

- 控制器输出是否在逐步打满
- 失稳前是否有突刺
- 姿态项和位置项谁先明显变坏

## 9. 第一阶段结束后，必须能给出的结论

第一阶段的成果不是“跑得更快”，而是能客观回答下面这些问题：

1. `trotting` 零命令失稳时，估计器先坏还是控制器先坏？
2. 当前 estimator 在 Gazebo 真值对比下，误差到底有多大？
3. 当前 trotting controller 是不是在追不合理目标？
4. `dd_pcd` 和 `d_wbd` 在失稳前有没有明显持续增长？
5. 力矩输出在失稳前是否已经出现异常放大？

如果这五个问题答不出来，就还不应该直接进入大规模控制器替换。

## 10. 第一阶段结束后如何决定下一步

做完第一阶段后，再做决策：

### 情况 A：估计器问题更大

典型现象：

- truth-vs-estimate 误差很大
- 静止也明显漂
- 接触切换时估计跳变

那么下一步优先：

- 升级估计器

### 情况 B：控制器问题更大

典型现象：

- 估计与真值大体还对得上
- 但 tracking error 和控制输出明显发散
- trotting 站立和走动都压不住

那么下一步优先：

- 重构控制器架构
- 再逐步引入 MPC/WBC

### 情况 C：两边都明显薄弱

典型现象：

- 估计误差也明显
- 控制输出也发散

那么建议：

- 先修估计器到“基本可信”
- 再上高级控制

不要反过来。

## 11. 最后一句操作建议

第一阶段最重要的不是“多改代码”，而是“少猜，多测”。

只要你把：

- baseline 固定住
- 日志采齐
- 真值对比打通
- 图画出来

后面不管是升级估计器，还是做 MPC/WBC，都会轻松很多。
