//
// Created by tlab-uav on 24-9-18.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h> //拿估计器、机器人模型、平衡控制器、步态器
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

namespace {
constexpr const char* kTrottingLogPath = "/home/xiangh9/xhros2/uni_ws/tmp/unitree_phase1_logs/trotting.csv";
constexpr std::size_t kTrottingLogDecimation = 20;

const char* waveStatusToString(const WaveStatus status) {
    switch (status) {
        case WaveStatus::SWING_ALL:
            return "SWING_ALL";
        case WaveStatus::STANCE_ALL:
            return "STANCE_ALL";
        case WaveStatus::WAVE_ALL:
            return "WAVE_ALL";
        default:
            return "UNKNOWN";
    }
}
}

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : FSMState(FSMStateName::TROTTING, "trotting",
                                                                       ctrl_interfaces),
                                                              estimator_(ctrl_component.estimator_),
                                                              robot_model_(ctrl_component.robot_model_),
                                                              balance_ctrl_(ctrl_component.balance_ctrl_),
                                                              wave_generator_(ctrl_component.wave_generator_),
                                                              gait_generator_(ctrl_component) {
    gait_height_ = 0.08;
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();//机身位置控制的PD增益   Kpp 位置误差转加速度  Kdp 速度误差转加速度
    kp_w_ = 780;
    Kd_w_ = Vec3(70, 70, 70).asDiagonal();   //姿态控制的PD增益   姿态误差转角加速度    角速度误差转加速度
    Kp_swing_ = Vec3(400, 400, 400).asDiagonal();
    Kd_swing_ = Vec3(10, 10, 10).asDiagonal();     //摆动腿足端PD控制   某条腿在空中时，不再使用地面接触力分配，直接做足端轨迹跟踪

    v_x_limit_ << -0.4, 0.4;
    v_y_limit_ << -0.3, 0.3;
    w_yaw_limit_ << -0.5, 0.5;
    dt_ = 1.0 / ctrl_interfaces_.frequency_;

    csv_logger_.open(kTrottingLogPath, {
        "sample",
        "time",
        "wave_status",
        "cmd_vx", "cmd_vy", "cmd_vz",
        "target_vx", "target_vy", "target_vz",
        "pcd_x", "pcd_y", "pcd_z",
        "body_x", "body_y", "body_z",
        "body_vx", "body_vy", "body_vz",
        "yaw_cmd",
        "pos_err_x", "pos_err_y", "pos_err_z",
        "vel_err_x", "vel_err_y", "vel_err_z",
        "dd_pcd_x", "dd_pcd_y", "dd_pcd_z",
        "d_wbd_x", "d_wbd_y", "d_wbd_z",
        "contact_fl", "contact_fr", "contact_rl", "contact_rr"
    });
}// 相当于初始化函数，声明一些必要的参数

void StateTrotting::enter() {
    pcd_ = estimator_->getPosition();
    pcd_(2) = -estimator_->getFeetPos2Body()(2, 0);
    v_cmd_body_.setZero();
    yaw_cmd_ = estimator_->getYaw();
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();

    ctrl_interfaces_.control_inputs_.command = 0;
    gait_generator_.restart();
}
//在进入Trotting模式时，会执行一次，并不是每个周期都会执行
void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    pos_body_ = estimator_->getPosition();
    vel_body_ = estimator_->getVelocity();
    //读取当前位置和速度估计

    B2G_RotMat = estimator_->getRotation();
    G2B_RotMat = B2G_RotMat.transpose();
    //当前机身到世界的旋转矩阵，以及它的转置

    getUserCmd();
    calcCmd();
    //读取用户命令  并且把用户命令进一步处理为平滑后的机身目标

    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    //机身平动目标、转向目标、摆腿高度传给步态生成器
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);
    //四条腿下一步的足端目标位置，速度

    calcTau();
    //算关节力矩命令；  根据机身误差算期望机身加速度/角加速度，BanlanceCtrl分配成足端力，摆动腿改成足端PD跟踪力  机器人模型映射为关节力矩

    calcQQd();
    //算关节位置、速度命令  将足端目标转为关节目标    torque  position  velocity都同时写

    if (checkStepOrNot()) {
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    } else {
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
    }

    calcGain();
    //根据当前腿是支撑还是摆动，设置不同的关节  kp/kd

    log_counter_++;
    if (csv_logger_.isOpen() && log_counter_ % kTrottingLogDecimation == 0) {
        csv_logger_.writeRow(
            log_counter_,
            log_counter_ * dt_,
            waveStatusToString(wave_generator_->status_),
            v_cmd_body_(0), v_cmd_body_(1), v_cmd_body_(2),
            vel_target_(0), vel_target_(1), vel_target_(2),
            pcd_(0), pcd_(1), pcd_(2),
            pos_body_(0), pos_body_(1), pos_body_(2),
            vel_body_(0), vel_body_(1), vel_body_(2),
            yaw_cmd_,
            pos_error_(0), pos_error_(1), pos_error_(2),
            vel_error_(0), vel_error_(1), vel_error_(2),
            last_dd_pcd_(0), last_dd_pcd_(1), last_dd_pcd_(2),
            last_d_wbd_(0), last_d_wbd_(1), last_d_wbd_(2),
            wave_generator_->contact_(0),
            wave_generator_->contact_(1),
            wave_generator_->contact_(2),
            wave_generator_->contact_(3)
        );
    }
}

void StateTrotting::exit() {
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateTrotting::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        case 7:
            return FSMStateName::NMPC;
        default:
            return FSMStateName::TROTTING;
    }
}

void StateTrotting::getUserCmd() {
    /* Movement */
    v_cmd_body_(0) = invNormalize(ctrl_interfaces_.control_inputs_.ly, v_x_limit_(0), v_x_limit_(1));
    v_cmd_body_(1) = -invNormalize(ctrl_interfaces_.control_inputs_.lx, v_y_limit_(0), v_y_limit_(1));
    v_cmd_body_(2) = 0;

    /* Turning */
    d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
    d_yaw_cmd_ = 0.9 * d_yaw_cmd_past_ + (1 - 0.9) * d_yaw_cmd_;
    d_yaw_cmd_past_ = d_yaw_cmd_;
}

void StateTrotting::calcCmd() {
    /* Movement */
    vel_target_ = B2G_RotMat * v_cmd_body_;//机体系速度命令变成世界系速度命令

    vel_target_(0) =
            saturation(vel_target_(0), Vec2(vel_body_(0) - 0.2, vel_body_(0) + 0.2));
    vel_target_(1) =
            saturation(vel_target_(1), Vec2(vel_body_(1) - 0.2, vel_body_(1) + 0.2));

    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_,
                         Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_,
                         Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));

    vel_target_(2) = 0;

    /* Turning */
    yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
    Rd = rotz(yaw_cmd_);
    w_cmd_global_(2) = d_yaw_cmd_;
}

void StateTrotting::calcTau() {
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) +
                 Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    last_dd_pcd_ = dd_pcd;
    last_d_wbd_ = d_wbd;

    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();//得到足端相对于机身的位置
    Vec34 force_feet_global =
            -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);
//  输入：期望机身线加速度、角加速度、当前姿态、足端位置、接触状态   输出  四条腿在世界系中的足端力

    Vec34 pos_feet_global = estimator_->getFeetPos();
    Vec34 vel_feet_global = estimator_->getFeetVel();
    // 拿当前四足在世界系的位置和速度

    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            force_feet_global.col(i) =
                    Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }//对于摆动腿，要将足端力改成足端PD 控制

    // 支撑腿 用力分配结果   摆动腿 用轨迹跟踪结果

    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;
    // 足端力从世界系转回机体系

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
    //足端力 通过雅可比转成该腿3个关节的力矩，再写入torque
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();
    //足端在body系下的位置

    Vec34 pos_feet_target, vel_feet_target;
    // 目标足端  位置/速度  这里都是body系

    for (int i(0); i < 4; ++i) {
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }
    // 将前面生成的世界系足端目标  转换到  body系

    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    //足端 目标位置做逆运动学  得到12个关节目标角

    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);
    //足端目标速度求12个关节目标速度

    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
    //关节目标位置和速度写入接口
}

void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            // swing gain
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2);
            }
        } else {
            // stable gain
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(0.8);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(0.8);
            }
        }
    }
}
//   根据腿是摆动还是支撑，设置不同的关节增益   摆动腿增益大，利于轨迹跟踪
//    支撑腿更加柔和，避免过硬

bool StateTrotting::checkStepOrNot() {
    if (fabs(v_cmd_body_(0)) > 0.03 || fabs(v_cmd_body_(1)) > 0.03 ||
        fabs(pos_error_(0)) > 0.08 || fabs(pos_error_(1)) > 0.08 ||
        fabs(vel_error_(0)) > 0.05 || fabs(vel_error_(1)) > 0.05 ||
        fabs(d_yaw_cmd_) > 0.20) {
        return true;
    }
    return false;
}

//用户前后/左右命令明显不为 0
// 机身位置误差过大
// 机身速度误差过大
// 转向命令过大    满足这些条件都会触发迈步
