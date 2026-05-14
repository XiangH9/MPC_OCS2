
#include "unitree_guide_controller/FSM/StateNMPC.h"

#include <unitree_guide_controller/control/CtrlComponent.h>

#include <algorithm>
#include <vector>

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/control/BalanceCtrl.h>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

//运行时调度壳？ 

namespace {

// roll/pitch/yaw  ->  Rotmatrix
RotMat rpyToRotMat(const Vec3& rpy) {
    const double roll = rpy(0);
    const double pitch = rpy(1);
    const double yaw = rpy(2);

    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);

    RotMat Rx;
    Rx << 1, 0, 0,
          0, cr, -sr,
          0, sr, cr;

    RotMat Ry;
    Ry << cp, 0, sp,
          0, 1, 0,
          -sp, 0, cp;

    RotMat Rz;
    Rz << cy, -sy, 0,
          sy, cy, 0,
          0, 0, 1;

    return Rz * Ry * Rx;
}

//角度包到[-pi,pi]
double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

//处理姿态误差，防止角度跳变
Vec3 wrapRpyError(const Vec3& target_rpy, const Vec3& current_rpy) {
    Vec3 error = target_rpy - current_rpy;
    error(0) = wrapAngle(error(0));
    error(1) = wrapAngle(error(1));
    error(2) = wrapAngle(error(2));
    return error;
}

//plan_mode 转成四条腿的接触状态
VecInt4 modeToContact(size_t mode) {
    VecInt4 contact;
    contact << static_cast<int>((mode >> 3) & 0x1),
               static_cast<int>((mode >> 2) & 0x1),
               static_cast<int>((mode >> 1) & 0x1),
               static_cast<int>(mode & 0x1);
    return contact;
}

}  // namespace

const char* StateNMPC::legRoleToString(LegRole role) const {
    switch (role) {
        case LegRole::STANCE:
            return "STANCE";
        case LegRole::SWING:
            return "SWING";
        default:
            return "UNKNOWN";
    }
}

std::array<StateNMPC::LegRole, 4> StateNMPC::getLegRoles(const VecInt4& contact) const {
    std::array<LegRole, 4> roles{};
    for (int i = 0; i < 4; ++i) {
        roles[i] = (contact(i) == 1) ? LegRole::STANCE : LegRole::SWING;
    }

    return roles;
}


StateNMPC::StateNMPC(CtrlInterfaces& ctrl_interfaces, CtrlComponent& ctrl_component)
    : FSMState(FSMStateName::NMPC, "nmpc", ctrl_interfaces),
      ctrl_component_(ctrl_component),
      unified_gait_scheduler_(ctrl_component){
}//初始构造函数，但这里需要思考的是 Unified_gait_scheduler的必要性


//进入时调用
void StateNMPC::enter() {
    std::cout << "[StateNMPC] enter" << std::endl;
    ctrl_interfaces_.control_inputs_.command = 0;

    //检查NMPC的接口是否准备好
    if (!ctrl_component_.isNmpcInterfaceReady()) {
        std::cerr << "[StateNMPC] NMPC interface is not ready." << std::endl;
        return;
    }
    //简单理解为： legged_interface创建好、配置已经读入、基础模型/问题定义完成初始化

    const auto& config = ctrl_component_.legged_interface_->getBasicConfig();
    // 读取基础配置状态用来做完整性检查

    const bool task_sections_ok =
        config.has_model_settings &&
        config.has_mpc_settings &&
        config.has_sqp_settings &&
        config.has_rollout_settings &&
        config.has_swing_trajectory_config &&
        config.has_initial_state;

    //  检查 task 是否配置齐全 ： 模型设置、MPC设置、SQP设置、rollout设置、swing trajectory设置、初始状态
 
    const bool reference_sections_ok =
        config.has_initial_mode_schedule &&
        config.has_default_mode_sequence_template;
    //参考轨迹/模式调度相关配置    不仅需要模型和代价函数，也需要参考模式信息

    std::cout << "[StateNMPC] NMPC interface is ready." << std::endl;
    std::cout << "[StateNMPC] verbose=" << (config.verbose ? "true" : "false")
              << ", task sections ok=" << (task_sections_ok ? "true" : "false")
              << ", reference sections ok=" << (reference_sections_ok ? "true" : "false")
              << std::endl;
    //NMPC总体接口ready  verbose模式是否开启？  task配置是否齐全   reference配置是否齐全

    unified_gait_scheduler_.restart();
    ctrl_component_.setupMpcRuntime();
    // Optimal control problem , initializer, reference manager, SQP MPC , MPC_MRT_interface，运行时对象都准备好

    if (ctrl_component_.isMpcRuntimeReady()) {
        std::cout << "[StateNMPC] MPC runtime is ready." << std::endl;
    } else if (ctrl_component_.hasMpcRuntimeShell()) {
        std::cout << "[StateNMPC] MPC runtime shell is ready, solver not connected yet." << std::endl;
    } else {
        std::cout << "[StateNMPC] MPC runtime setup did not complete." << std::endl;
    }

    if (ctrl_component_.estimator_) {
        support_body_pos_target_ = ctrl_component_.estimator_->getPosition();
        support_body_vel_target_.setZero();

        support_body_rpy_target_ =
            rotMatToRPY(ctrl_component_.estimator_->getRotation());
        support_body_rpy_rate_target_.setZero();

        support_target_initialized_ = true;
    } else {
        support_target_initialized_ = false;
    }//初始目标设置为估计器状态


    std::cout << "[StateNMPC] initial support target z: "
            << support_body_pos_target_(2) << std::endl;
    std::cout << "[StateNMPC] initial support target rpy: "
            << "roll=" << support_body_rpy_target_(0)
            << ", pitch=" << support_body_rpy_target_(1)
            << ", yaw=" << support_body_rpy_target_(2) << std::endl;
            //打印初始化高度和初始化姿态

}



void StateNMPC::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    static int counter = 0;
    counter++;
    const double control_dt = period.seconds();
    constexpr int mpc_update_interval = 20;//打印的频率

    if (counter % mpc_update_interval == 0) {
        std::cout << "[StateNMPC] run alive" << std::endl;
    }

    if (counter % mpc_update_interval == 0 && ctrl_component_.estimator_) {
        const Vec3 pos = ctrl_component_.estimator_->getPosition();
        const Vec3 vel = ctrl_component_.estimator_->getVelocity();
        const Vec3 rpy = rotMatToRPY(ctrl_component_.estimator_->getRotation());//得到位置速度姿态

        std::cout << "[StateNMPC] est base pos: "
                << "x=" << pos(0) << ", y=" << pos(1) << ", z=" << pos(2) << std::endl;
        std::cout << "[StateNMPC] est base vel: "
                << "vx=" << vel(0) << ", vy=" << vel(1) << ", vz=" << vel(2) << std::endl;
        std::cout << "[StateNMPC] estimator base rpy: "
                << "roll=" << rpy(0)
                << ", pitch=" << rpy(1)
                << ", yaw=" << rpy(2) << std::endl;
    }//打印估计器姿态

    if (!ctrl_component_.isNmpcInterfaceReady()) {
        return;
    }
    //每次运行都保证接口ready


    const double cmd_scale = 1.0;
    // accept keyboard velocity command
    const double vx_cmd = 0.4 * ctrl_interfaces_.control_inputs_.ly * cmd_scale;
    const double vy_cmd = -0.2 * ctrl_interfaces_.control_inputs_.lx * cmd_scale;
    const double yaw_rate_cmd = -0.5 * ctrl_interfaces_.control_inputs_.rx * cmd_scale;

    if (ctrl_component_.isMpcRuntimeReady()) {
        ctrl_component_.advanceMpcClock(control_dt);//推进OCS2观测时间
        ctrl_component_.updateMeasuredRbdStateFromEstimator();
    }//准备好就运行MPC 并且从估计器更信测量状态


    const bool zero_command =
        std::abs(vx_cmd) < 1e-3 &&
        std::abs(vy_cmd) < 1e-3 &&
        std::abs(yaw_rate_cmd) < 1e-3;

    unified_gait_scheduler_.setGaitCommand(Vec2(vx_cmd, vy_cmd), yaw_rate_cmd, 0.05);
    unified_gait_scheduler_.update(zero_command);//主要生成足端reference，OCS2 mode来源仍然是MPC 规划器  reference.info 本身

    ctrl_component_.pushCurrentObservationToMpc();  

    if (zero_command) {
        ctrl_component_.updateDesiredBodyVelocityCommand(0.0, 0.0, 0.0);
    } else {
        ctrl_component_.updateDesiredBodyVelocityCommand(vx_cmd, vy_cmd, yaw_rate_cmd);
    }

    ctrl_component_.updateTargetTrajectoriesFromCommand();//这里更新OCS2层的TargetTrajectories
    //同一拍更新足端参考和body-level Trajectories



    if (ctrl_component_.isMpcRuntimeReady()) {
        ctrl_component_.evaluateCurrentPolicy();

        if (counter % mpc_update_interval == 0 && ctrl_component_.hasLatestPolicy()) {
            std::cout << "[StateNMPC] latest NMPC policy is available." << std::endl;
        }
    }//后台算好的Policy取到前台来进行评估 optimized_state_ optimized_input_ planned_mode_ nmpc_joint_position_cmd nmpc_joint_velocity_cmd

    const bool nmpc_command_ready =
        ctrl_component_.hasNmpcJointPositionCommand() &&
        ctrl_component_.hasNmpcJointVelocityCommand();
    //下命令的接口检查
    const size_t solver_mode = ctrl_component_.getPlannedMode();
    const VecInt4 solver_contact = modeToContact(solver_mode);
    const auto leg_roles = getLegRoles(solver_contact);
    //根据 mode 分腿

    
    if (counter % mpc_update_interval == 0) {
        std::cout << "[StateNMPC] zero_command: "
                << (zero_command ? "true" : "false") << std::endl;

        std::cout << "[StateNMPC] execution mode: "
                << (zero_command ? "support_torque_only" :
                    (nmpc_command_ready ?
                        "support_torque_plus_nmpc_hybrid" :
                        "support_torque_only"))
                << std::endl;

    }

    if (counter % mpc_update_interval == 0) {
        std::cout << "[StateNMPC] solver leg roles: "
                << legRoleToString(leg_roles[0]) << ", "
                << legRoleToString(leg_roles[1]) << ", "
                << legRoleToString(leg_roles[2]) << ", "
                << legRoleToString(leg_roles[3]) << std::endl;
    }
    if (counter % mpc_update_interval == 0) {
        std::cout << "[StateNMPC] solver contact: "
                << solver_contact(0) << ", "
                << solver_contact(1) << ", "
                << solver_contact(2) << ", "
                << solver_contact(3) << std::endl;
    }
    if (counter % mpc_update_interval == 0) {
        const auto& execution_phase = unified_gait_scheduler_.getExecutionPhase();
        std::cout << "[StateNMPC] wave phase: "
                << execution_phase(0) << ", "
                << execution_phase(1) << ", "
                << execution_phase(2) << ", "
                << execution_phase(3) << std::endl;
    }

    if (counter % mpc_update_interval == 0) {
        const auto& foot_pos_goal = unified_gait_scheduler_.getFootPosGoal();
        const auto& foot_vel_goal = unified_gait_scheduler_.getFootVelGoal();

        std::cout << "[StateNMPC] reference foot target[0]: "
                << "x=" << foot_pos_goal(0, 0)
                << ", y=" << foot_pos_goal(1, 0)
                << ", z=" << foot_pos_goal(2, 0) << std::endl;

        std::cout << "[StateNMPC] reference foot vel[0]: "
                << "vx=" << foot_vel_goal(0, 0)
                << ", vy=" << foot_vel_goal(1, 0)
                << ", vz=" << foot_vel_goal(2, 0) << std::endl;
    }


    // Locomotion execution is now split by leg role:
    // stance legs are handled by support torque / J^T f,
    // swing legs are handled by joint-space tracking.
    updateSupportTarget(vx_cmd, vy_cmd, yaw_rate_cmd, control_dt);
    applySupportTorque(zero_command);

    if (!zero_command && nmpc_command_ready) {
        applyStanceLegNmpcHoldCommand(leg_roles);
    } else if (!zero_command) {
        std::cout << "[StateNMPC] NMPC stance hold command is not ready." << std::endl;
    }

    if (!zero_command && nmpc_command_ready) {
        applySwingLegControl(leg_roles);
    } else if (!zero_command) {
        std::cout << "[StateNMPC] NMPC swing command is not ready, keep stance support only." << std::endl;
    }



}

void StateNMPC::updateSupportTarget(double vx_body, double vy_body, double yaw_rate_cmd, double control_dt) {
    if (!support_target_initialized_ || !ctrl_component_.estimator_) {
        return;
    }//目标或者估计器有一个出问题就直接退出

    const RotMat body_to_world = ctrl_component_.estimator_->getRotation();//姿态矩阵
    const Vec3 current_pos = ctrl_component_.estimator_->getPosition();
    const Vec3 current_rpy = rotMatToRPY(body_to_world);
    const Vec3 body_velocity_cmd(vx_body, vy_body, 0.0);
    const Vec3 world_velocity_cmd = body_to_world * body_velocity_cmd;
    constexpr double support_preview_time = 0.25;

    support_body_vel_target_(0) = world_velocity_cmd(0);
    support_body_vel_target_(1) = world_velocity_cmd(1);
    support_body_vel_target_(2) = 0.0;

    // Re-anchor the support target to the current estimate each cycle and use
    // only a short preview step. This avoids unbounded drift of the support
    // target when the robot is already deviating from the commanded motion.
    support_body_pos_target_(0) = current_pos(0) + support_body_vel_target_(0) * support_preview_time;
    support_body_pos_target_(1) = current_pos(1) + support_body_vel_target_(1) * support_preview_time;
    support_body_pos_target_(2) = current_pos(2);//不是积分远期目标，而是当前测量位置加一个短时预瞄速度 不加额外高度

    support_body_rpy_rate_target_.setZero();
    support_body_rpy_target_(0) = current_rpy(0);
    support_body_rpy_target_(1) = current_rpy(1);
    support_body_rpy_rate_target_(2) = yaw_rate_cmd;
    support_body_rpy_target_(2) = wrapAngle(current_rpy(2) + yaw_rate_cmd * support_preview_time);//当前测量加短时预瞄

    static int support_target_log_counter = 0;
    support_target_log_counter++;
    if (support_target_log_counter % 20 == 0 && ctrl_component_.estimator_) {
        std::cout << "[StateNMPC] support target update: "
                  << "target_px=" << support_body_pos_target_(0)
                  << ", target_py=" << support_body_pos_target_(1)
                  << ", target_pz=" << support_body_pos_target_(2)
                  << ", target_yaw=" << support_body_rpy_target_(2)
                  << ", target_roll=" << support_body_rpy_target_(0)
                  << ", target_pitch=" << support_body_rpy_target_(1)
                  << ", target_vx_world=" << support_body_vel_target_(0)
                  << ", target_vy_world=" << support_body_vel_target_(1)
                  << ", meas_py=" << current_pos(1)
                  << ", meas_pz=" << current_pos(2)
                  << ", meas_roll=" << current_rpy(0)
                  << ", meas_pitch=" << current_rpy(1)
                  << ", meas_yaw=" << current_rpy(2)
                  << std::endl;
    }


}

void StateNMPC::applySupportTorque(bool force_full_contact) {
    if (!support_target_initialized_ ||
        !ctrl_component_.estimator_ ||
        !ctrl_component_.balance_ctrl_ ||
        !ctrl_component_.robot_model_) {
        return;
    }

    if (ctrl_interfaces_.joint_torque_command_interface_.size() < 12) {
        std::cerr << "[StateNMPC] joint_torque_command_interface size is insufficient." << std::endl;
        return;
    }

    constexpr bool verbose_support_debug = false;  //debug日志控制

    const Vec3 pos_body = ctrl_component_.estimator_->getPosition();
    const Vec3 vel_body = ctrl_component_.estimator_->getVelocity();
    const RotMat body_to_world = ctrl_component_.estimator_->getRotation();
    const RotMat world_to_body = body_to_world.transpose();

    const Vec3 pos_error = support_body_pos_target_ - pos_body;
    const Vec3 vel_error = support_body_vel_target_ - vel_body;
    static int support_error_log_counter = 0;
    support_error_log_counter++;
    if (support_error_log_counter % 20 == 0) {
        const Vec3 current_rpy = rotMatToRPY(body_to_world);

        std::cout << "[StateNMPC] support lateral error: "
                  << "pos_error_y=" << pos_error(1)
                  << ", vel_error_y=" << vel_error(1)
                  << ", target_py=" << support_body_pos_target_(1)
                  << ", meas_py=" << pos_body(1)
                  << ", target_yaw=" << support_body_rpy_target_(2)
                  << ", meas_yaw=" << current_rpy(2)
                  << std::endl;
    }


    const Mat3 Kpp = Vec3(70, 70, 120).asDiagonal();
    const Mat3 Kdp = Vec3(10, 10, 15).asDiagonal();
    const double kp_w = 500.0;
    const Mat3 Kd_w = Vec3(50, 50, 20).asDiagonal();

    Vec3 dd_pcd = Kpp * pos_error + Kdp * vel_error;  //期望线加速度 PD控制

    const RotMat Rd = rpyToRotMat(support_body_rpy_target_);
    const Vec3 w_cmd_global = support_body_rpy_rate_target_;

    Vec3 d_wbd =
        kp_w * rotMatToExp(Rd * world_to_body) +
        Kd_w * (w_cmd_global - ctrl_component_.estimator_->getGyroGlobal());  //期望角加速度项

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 feet_pos_2_body = ctrl_component_.estimator_->getFeetPos2Body();

    if (verbose_support_debug) {
        for (int i = 0; i < 4; ++i) {
            std::cout << "[StateNMPC] foot_pos_2_body[" << i << "]: "
                    << "x=" << feet_pos_2_body(0, i)
                    << ", y=" << feet_pos_2_body(1, i)
                    << ", z=" << feet_pos_2_body(2, i) << std::endl;
        }
    }

    VecInt4 contact;
    if (force_full_contact) {
        contact << 1, 1, 1, 1;
    } else {
        contact = modeToContact(ctrl_component_.getPlannedMode()); //support torque 也跟着solver mode走
    }


    Vec34 force_feet_global =
        -ctrl_component_.balance_ctrl_->calF(dd_pcd, d_wbd, body_to_world, feet_pos_2_body, contact);

    Vec34 force_feet_body = world_to_body * force_feet_global;

    for (int i = 0; i < 4; ++i) {
        KDL::JntArray torque = ctrl_component_.robot_model_->getTorque(force_feet_body.col(i), i);
        for (int j = 0; j < 3; ++j) {
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }

    static int support_log_counter = 0;
    support_log_counter++;
    if (support_log_counter % 100 == 0) {
        const Vec3 current_rpy = rotMatToRPY(body_to_world);
        const Vec3 raw_rpy_error = support_body_rpy_target_ - current_rpy;
        const Vec3 wrapped_rpy_error = wrapRpyError(support_body_rpy_target_, current_rpy);


        std::cout << "[StateNMPC] support target z: " << support_body_pos_target_(2)
                << ", measured z: " << pos_body(2)
                << ", pos_error z: " << pos_error(2) << std::endl;
        std::cout << "[StateNMPC] support rpy error raw: "
                << "roll=" << raw_rpy_error(0)
                << ", pitch=" << raw_rpy_error(1)
                << ", yaw=" << raw_rpy_error(2) << std::endl;

        std::cout << "[StateNMPC] support rpy error wrapped: "
                << "roll=" << wrapped_rpy_error(0)
                << ", pitch=" << wrapped_rpy_error(1)
                << ", yaw=" << wrapped_rpy_error(2) << std::endl;

        std::cout << "[StateNMPC] support contact: "
                << contact(0) << ", "
                << contact(1) << ", "
                << contact(2) << ", "
                << contact(3)
                << " (planned_mode=" << ctrl_component_.getPlannedMode()
                << ", force_full_contact=" << (force_full_contact ? "true" : "false")
                << ")" << std::endl;


        if (verbose_support_debug) {

            std::cout << "[StateNMPC] dd_pcd: "
                << "x=" << dd_pcd(0)
                << ", y=" << dd_pcd(1)
                << ", z=" << dd_pcd(2) << std::endl;

            std::cout << "[StateNMPC] support torque[0]: "
                    << ctrl_interfaces_.joint_torque_command_interface_[0].get().get_value() << std::endl;

            std::cout << "[StateNMPC] d_wbd: "
            << "roll=" << d_wbd(0)
            << ", pitch=" << d_wbd(1)
            << ", yaw=" << d_wbd(2) << std::endl;

            std::cout << "[StateNMPC] foot fz: "
                    << force_feet_global(2, 0) << ", "
                    << force_feet_global(2, 1) << ", "
                    << force_feet_global(2, 2) << ", "
                    << force_feet_global(2, 3) << std::endl;

            for (int i = 0; i < 4; ++i) {
                std::cout << "[StateNMPC] foot force[" << i << "]: "
                        << "fx=" << force_feet_global(0, i)
                        << ", fy=" << force_feet_global(1, i)
                        << ", fz=" << force_feet_global(2, i) << std::endl;
            }


            std::cout << "[StateNMPC] torque summary: "
                    << ctrl_interfaces_.joint_torque_command_interface_[0].get().get_value() << ", "
                    << ctrl_interfaces_.joint_torque_command_interface_[3].get().get_value() << ", "
                    << ctrl_interfaces_.joint_torque_command_interface_[6].get().get_value() << ", "
                    << ctrl_interfaces_.joint_torque_command_interface_[9].get().get_value() << std::endl;

            std::cout << "[StateNMPC] rear leg torque RL: "
            << ctrl_interfaces_.joint_torque_command_interface_[6].get().get_value() << ", "
            << ctrl_interfaces_.joint_torque_command_interface_[7].get().get_value() << ", "
            << ctrl_interfaces_.joint_torque_command_interface_[8].get().get_value() << std::endl;

            std::cout << "[StateNMPC] rear leg torque RR: "
                    << ctrl_interfaces_.joint_torque_command_interface_[9].get().get_value() << ", "
                    << ctrl_interfaces_.joint_torque_command_interface_[10].get().get_value() << ", "
                    << ctrl_interfaces_.joint_torque_command_interface_[11].get().get_value() << std::endl;

        }

    }
}


void StateNMPC::applyStanceLegNmpcHoldCommand(const std::array<LegRole, 4>& leg_roles) {
    if (!ctrl_component_.hasNmpcJointPositionCommand() ||
        !ctrl_component_.hasNmpcJointVelocityCommand()) {
        return;
    }

    const auto& joint_pos_cmd = ctrl_component_.getNmpcJointPositionCommand();
    const auto& joint_vel_cmd = ctrl_component_.getNmpcJointVelocityCommand();
    //支撑腿的joint命令源也是NMPC policy

    const Eigen::Index cmd_size = joint_pos_cmd.size();
    if (joint_vel_cmd.size() != cmd_size) {
        std::cerr << "[StateNMPC] stance hold joint command size mismatch." << std::endl;
        return;
    }

    if (ctrl_interfaces_.joint_position_command_interface_.size() < static_cast<size_t>(cmd_size) ||
        ctrl_interfaces_.joint_velocity_command_interface_.size() < static_cast<size_t>(cmd_size) ||
        ctrl_interfaces_.joint_kp_command_interface_.size() < static_cast<size_t>(cmd_size) ||
        ctrl_interfaces_.joint_kd_command_interface_.size() < static_cast<size_t>(cmd_size)) {
        std::cerr << "[StateNMPC] stance hold command interface size is insufficient." << std::endl;
        return;
    }

    constexpr double stance_kp = 0.8;
    constexpr double stance_kd = 0.8;
    constexpr double stance_vel_limit = 2.0;

    static int stance_log_counter = 0;
    stance_log_counter++;

    int first_stance_joint_index = -1;
    int first_stance_leg_index = -1;
    double first_stance_q_cmd = 0.0;
    double first_stance_qd_before_clamp = 0.0;
    double first_stance_qd_after_clamp = 0.0;
    bool first_stance_joint_captured = false;

    for (Eigen::Index i = 0; i < cmd_size; ++i) {
        const int leg_index = static_cast<int>(i / 3);
        const bool is_stance_leg = (leg_roles[leg_index] == LegRole::STANCE);

        if (!is_stance_leg) {
            continue;
        }

        double qd_cmd = joint_vel_cmd(i);
        if (!first_stance_joint_captured) {
            first_stance_joint_index = static_cast<int>(i);
            first_stance_leg_index = leg_index;
            first_stance_qd_before_clamp = qd_cmd;
        }

        qd_cmd = std::clamp(qd_cmd, -stance_vel_limit, stance_vel_limit);

        if (!first_stance_joint_captured) {
            first_stance_qd_after_clamp = qd_cmd;
        }

        const double q_cmd = joint_pos_cmd(i);

        if (!first_stance_joint_captured) {
            first_stance_q_cmd = q_cmd;
            first_stance_joint_captured = true;
        }

        ctrl_interfaces_.joint_position_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(q_cmd);

        ctrl_interfaces_.joint_velocity_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(qd_cmd);

        ctrl_interfaces_.joint_kp_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(stance_kp);

        ctrl_interfaces_.joint_kd_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(stance_kd);
    }

    if (stance_log_counter % 100 == 0) {
        std::cout << "[StateNMPC] applying stance NMPC hold command." << std::endl;

        if (!first_stance_joint_captured) {
            std::cout << "[StateNMPC] no stance joint was controlled in this cycle." << std::endl;
        } else {
            std::cout << "[StateNMPC] first stance joint index: "
                      << first_stance_joint_index
                      << ", leg index: " << first_stance_leg_index << std::endl;

            std::cout << "[StateNMPC] first stance joint vel before clamp: "
                      << first_stance_qd_before_clamp << std::endl;

            std::cout << "[StateNMPC] first stance joint vel after clamp: "
                      << first_stance_qd_after_clamp << std::endl;

            std::cout << "[StateNMPC] first stance joint pos from NMPC: "
                      << first_stance_q_cmd << std::endl;
        }
    }
}


void StateNMPC::applySwingLegControl(const std::array<LegRole, 4>& leg_roles) {
    if (!ctrl_component_.hasNmpcJointPositionCommand() ||
        !ctrl_component_.hasNmpcJointVelocityCommand()) {
        return;
    }

    const auto& joint_pos_cmd = ctrl_component_.getNmpcJointPositionCommand();
    const auto& joint_vel_cmd = ctrl_component_.getNmpcJointVelocityCommand();


    static int log_counter = 0;
    log_counter++;

    const size_t cmd_size = static_cast<size_t>(joint_vel_cmd.size());

    if (ctrl_interfaces_.joint_velocity_command_interface_.size() < cmd_size ||
        ctrl_interfaces_.joint_position_command_interface_.size() < cmd_size ||
        ctrl_interfaces_.joint_kp_command_interface_.size() < cmd_size ||
        ctrl_interfaces_.joint_kd_command_interface_.size() < cmd_size ||
        ctrl_interfaces_.joint_position_state_interface_.size() < cmd_size) {
        std::cerr << "[StateNMPC] hybrid command interfaces size is insufficient." << std::endl;
        return;
    }

    double first_swing_cmd_before_clamp = 0.0;
    double first_swing_cmd_after_clamp = 0.0;
    double first_swing_pos_cmd = 0.0;
    int first_swing_joint_index = -1;
    int first_swing_leg_index = -1;
    bool first_swing_joint_captured = false;

    constexpr double swing_kp = 3.0;
    constexpr double swing_kd = 2.0;

    if (!ctrl_component_.estimator_ || !ctrl_component_.robot_model_) {
        return;
    }

    const Vec3 pos_body = ctrl_component_.estimator_->getPosition();
    const Vec3 vel_body = ctrl_component_.estimator_->getVelocity();
    const RotMat B2G_RotMat = ctrl_component_.estimator_->getRotation();
    const RotMat G2B_RotMat = B2G_RotMat.transpose();

    const std::vector<KDL::Frame> pos_feet_body =
        ctrl_component_.robot_model_->getFeet2BPositions();

    Vec34 pos_feet_target = Vec34::Zero();
    Vec34 vel_feet_target = Vec34::Zero();

    const auto& foot_pos_goal = unified_gait_scheduler_.getFootPosGoal();
    const auto& foot_vel_goal = unified_gait_scheduler_.getFootVelGoal();

    for (int leg_index = 0; leg_index < 4; ++leg_index) {
        pos_feet_target.col(leg_index) =
            G2B_RotMat * (foot_pos_goal.col(leg_index) - pos_body);

        vel_feet_target.col(leg_index) =
            G2B_RotMat * (foot_vel_goal.col(leg_index) - vel_body);

    }

    Vec12 q_goal = ctrl_component_.robot_model_->getQ(pos_feet_target);
    Vec12 qd_goal = ctrl_component_.robot_model_->getQd(pos_feet_body, vel_feet_target);


    for (Eigen::Index i = 0; i < joint_vel_cmd.size(); ++i) {
        const int leg_index = static_cast<int>(i / 3);
        const bool is_swing_leg = (leg_roles[leg_index] == LegRole::SWING);

        if (!is_swing_leg) {
            continue;
        }

        double vel_cmd = qd_goal(i);

        if (!first_swing_joint_captured) {
            first_swing_joint_index = static_cast<int>(i);
            first_swing_leg_index = leg_index;
            first_swing_cmd_before_clamp = vel_cmd;
        }

        vel_cmd = std::clamp(vel_cmd, -2.0, 2.0);

        if (!first_swing_joint_captured) {
            first_swing_cmd_after_clamp = vel_cmd;
        }

        const double q_cmd = q_goal(i);

        if (!first_swing_joint_captured) {
            first_swing_pos_cmd = q_cmd;
            first_swing_joint_captured = true;
        }

        ctrl_interfaces_.joint_position_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(q_cmd);

        ctrl_interfaces_.joint_velocity_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(vel_cmd);

        ctrl_interfaces_.joint_kp_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(swing_kp);

        ctrl_interfaces_.joint_kd_command_interface_[static_cast<size_t>(i)]
            .get()
            .set_value(swing_kd);
    }



    if (log_counter % 100 == 0) {
        std::cout << "[StateNMPC] applying swing leg control, dim: "
                << joint_vel_cmd.size() << std::endl;

        std::cout << "[StateNMPC] swing planned_mode: "
                << ctrl_component_.getPlannedMode() << std::endl;

        std::cout << "[StateNMPC] swing leg roles: "
                << legRoleToString(leg_roles[0]) << ", "
                << legRoleToString(leg_roles[1]) << ", "
                << legRoleToString(leg_roles[2]) << ", "
                << legRoleToString(leg_roles[3]) << std::endl;

        std::cout << "[StateNMPC] swing control uses reference foot target -> IK -> joint tracking."
                << std::endl;

        std::cout << "[StateNMPC] swing gains: kp="
                << swing_kp << ", kd=" << swing_kd << std::endl;

        if (!first_swing_joint_captured) {
            std::cout << "[StateNMPC] no swing joint was controlled in this cycle."
                    << std::endl;
        } else {
            std::cout << "[StateNMPC] first swing joint index: "
                    << first_swing_joint_index
                    << ", leg index: " << first_swing_leg_index << std::endl;

            std::cout << "[StateNMPC] first swing foot target body: "
                    << "x=" << pos_feet_target(0, first_swing_leg_index)
                    << ", y=" << pos_feet_target(1, first_swing_leg_index)
                    << ", z=" << pos_feet_target(2, first_swing_leg_index)
                    << std::endl;

            std::cout << "[StateNMPC] first swing joint vel from reference IK before clamp: "
                    << first_swing_cmd_before_clamp << std::endl;

            std::cout << "[StateNMPC] first swing joint vel from reference IK after clamp: "
                    << first_swing_cmd_after_clamp << std::endl;

            std::cout << "[StateNMPC] first swing joint pos from reference IK: "
                    << first_swing_pos_cmd << std::endl;

            std::cout << "[StateNMPC] first swing joint q/qd from reference IK: "
                    << "q=" << q_goal(first_swing_joint_index)
                    << ", qd=" << qd_goal(first_swing_joint_index) << std::endl;

            std::cout << "[StateNMPC] first swing joint source compare: "
                    << "ref_q=" << q_goal(first_swing_joint_index)
                    << ", nmpc_q=" << joint_pos_cmd(first_swing_joint_index)
                    << ", ref_qd=" << qd_goal(first_swing_joint_index)
                    << ", nmpc_qd=" << joint_vel_cmd(first_swing_joint_index)
                    << std::endl;
        }
    }


}


void StateNMPC::exit() {
}

FSMStateName StateNMPC::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::NMPC;
    }
}
