//
// Created by biao on 24-9-14.
//

#include "unitree_guide_controller/control/Estimator.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>

#include "controller_common/CtrlInterfaces.h"

namespace {
constexpr const char* kEstimatorLogPath = "/home/xiangh9/xhros2/uni_ws/tmp/unitree_phase1_logs/estimator.csv";
constexpr std::size_t kEstimatorLogDecimation = 20;
}

Estimator::Estimator(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component) : ctrl_interfaces_(ctrl_interfaces),
                                                      robot_model_(ctrl_component.robot_model_),
                                                      wave_generator_(ctrl_component.wave_generator_) {
    g_ << 0, 0, -9.81;
    dt_ = 1.0 / ctrl_interfaces.frequency_;  //从控制器更新频率退出控制周期

    std::cout << "dt: " << dt_ << std::endl;
    large_variance_ = 100;   //不可信接触时放大噪声
    for (int i(0); i < Qdig.rows(); ++i) {
        Qdig(i) = i < 6 ? 0.0003 : 0.01;
    }//前六维是机身位置和速度，过程噪声小一点；后12维是足端位置状态，过程噪声大一点  更信机身状态模型的连续性，对足端状态保留更大调整空间

    x_hat_.setZero();
    u_.setZero();

    A.setZero();
    A.block(0, 0, 3, 3) = I3;
    A.block(0, 3, 3, 3) = I3 * dt_;
    A.block(3, 3, 3, 3) = I3;
    A.block(6, 6, 12, 12) = I12;
    //position_{k+1} = position_k + dt * velocity_k
    // velocity_{k+1} = velocity_k + dt * input
    // 足端状态默认保持不变
    // 所以这就是一个很典型的“机身匀加速度 + 足端随机游走”的线性模型。

    B.setZero();
    B.block(3, 0, 3, 3) = I3 * dt_;
    //说明输入只作用到速度，不直接作用到位置。

    C.setZero();
    C.block(0, 0, 3, 3) = -I3;
    C.block(3, 0, 3, 3) = -I3;
    C.block(6, 0, 3, 3) = -I3;
    C.block(9, 0, 3, 3) = -I3;
    C.block(12, 3, 3, 3) = -I3;
    C.block(15, 3, 3, 3) = -I3;
    C.block(18, 3, 3, 3) = -I3;
    C.block(21, 3, 3, 3) = -I3;
    C.block(0, 6, 12, 12) = I12;
    C(24, 8) = 1;
    C(25, 11) = 1;
    C(26, 14) = 1;
    C(27, 17) = 1;
    // 足端相对机身位置 = 足端世界位置 - 机身世界位置
    // 足端相对机身速度 = 足端世界速度 - 机身世界速度
    // 足高 = 对应足端 z

    P.setIdentity();
    P = large_variance_ * P;//初始不确定度很大，表示一开始不太信自己的状态初值。

    RInit_ << 0.008, 0.012, -0.000, -0.009, 0.012, 0.000, 0.009, -0.009, -0.000,
            -0.009, -0.009, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, -0.001,
            -0.002, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            0.012, 0.019, -0.001, -0.014, 0.018, -0.000, 0.014, -0.013, -0.000,
            -0.014, -0.014, 0.001, -0.001, 0.001, -0.001, 0.000, 0.000, -0.001,
            -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, 0.001, 0.001, -0.001, 0.000, -0.000, 0.000, -0.000, 0.001,
            0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000,
            -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.009, -0.014,
            0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000,
            0.001, 0.000, 0.000, 0.001, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003,
            0.000, 0.001, 0.000, 0.000, 0.000, 0.000, 0.012, 0.018, -0.001, -0.013,
            0.018, -0.000, 0.013, -0.013, -0.000, -0.013, -0.013, 0.001, -0.001,
            0.000, -0.001, 0.000, 0.001, -0.001, -0.003, 0.000, -0.001, -0.004,
            -0.000, -0.001, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
            -0.000, 0.001, 0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000,
            -0.000, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.009, 0.014, -0.000, -0.010, 0.013, 0.000,
            0.010, -0.010, -0.000, -0.010, -0.010, 0.000, -0.001, 0.000, -0.001,
            0.000, -0.000, -0.001, -0.001, 0.000, -0.000, -0.003, -0.000, -0.001,
            0.000, 0.000, 0.000, 0.000, -0.009, -0.013, 0.000, 0.010, -0.013, 0.000,
            -0.010, 0.009, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000,
            0.000, 0.001, 0.002, 0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000,
            0.000, 0.000, -0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000,
            0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000,
            -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, 0.000, 0.000,
            0.000, -0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000,
            0.010, 0.010, -0.000, 0.001, 0.000, 0.000, -0.000, -0.000, 0.001, 0.002,
            -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.009,
            -0.014, 0.000, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010,
            -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, -0.000, 0.000,
            0.003, 0.001, 0.001, 0.000, 0.000, 0.000, 0.000, 0.000, 0.001, -0.000,
            -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, -0.000, -0.000, 0.001, 0.000,
            -0.000, -0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, -0.000, -0.001, 0.000, 0.001, -0.001,
            -0.000, -0.001, 0.001, 0.000, 0.001, 0.001, 0.000, 1.708, 0.048, 0.784,
            0.062, 0.042, 0.053, 0.077, 0.001, -0.061, 0.046, -0.019, -0.029, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.001, -0.000, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.000, 0.000, -0.000, -0.000, 0.048, 5.001, -1.631, -0.036,
            0.144, 0.040, 0.036, 0.016, -0.051, -0.067, -0.024, -0.005, 0.000, 0.000,
            0.000, 0.000, -0.000, -0.001, 0.000, 0.000, -0.001, -0.000, -0.001, 0.000,
            0.000, 0.000, 0.000, -0.000, 0.784, -1.631, 1.242, 0.057, -0.037, 0.018,
            0.034, -0.017, -0.015, 0.058, -0.021, -0.029, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.001, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000,
            -0.000, -0.000, 0.062, -0.036, 0.057, 6.228, -0.014, 0.932, 0.059, 0.053,
            -0.069, 0.148, 0.015, -0.031, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000,
            -0.000, -0.000, 0.001, 0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
            0.042, 0.144, -0.037, -0.014, 3.011, 0.986, 0.076, 0.030, -0.052, -0.027,
            0.057, 0.051, 0.000, 0.000, 0.000, 0.000, -0.001, -0.001, -0.000, 0.001,
            -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, 0.053, 0.040,
            0.018, 0.932, 0.986, 0.885, 0.090, 0.044, -0.055, 0.057, 0.051, -0.003,
            0.000, 0.000, 0.000, 0.000, -0.002, -0.003, 0.000, 0.002, -0.003, -0.000,
            -0.001, 0.002, 0.000, 0.002, 0.002, -0.000, 0.077, 0.036, 0.034, 0.059,
            0.076, 0.090, 6.230, 0.139, 0.763, 0.013, -0.019, -0.024, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000,
            -0.000, -0.000, -0.000, 0.000, 0.001, 0.016, -0.017, 0.053, 0.030, 0.044,
            0.139, 3.130, -1.128, -0.010, 0.131, 0.018, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, -0.000, 0.000, -0.001, -0.000, -0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055,
            0.763, -1.128, 0.866, -0.022, -0.053, 0.007, 0.000, 0.000, 0.000, 0.000,
            -0.003, -0.004, -0.000, 0.003, -0.004, -0.000, -0.003, 0.003, 0.000,
            0.003, 0.003, 0.000, 0.046, -0.067, 0.058, 0.148, -0.027, 0.057, 0.013,
            -0.010, -0.022, 2.437, -0.102, 0.938, 0.000, 0.000, 0.000, 0.000, -0.000,
            -0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.001,
            0.000, -0.019, -0.024, -0.021, 0.015, 0.057, 0.051, -0.019, 0.131, -0.053,
            -0.102, 4.944, 1.724, 0.000, 0.000, 0.000, 0.000, -0.001, -0.001, 0.000,
            0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, -0.029,
            -0.005, -0.029, -0.031, 0.051, -0.003, -0.024, 0.018, 0.007, 0.938, 1.724,
            1.569, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 1.0;

    Cu << 268.573, -43.819, -147.211, -43.819, 92.949, 58.082, -147.211, 58.082,
            302.120;//输入噪声协方差  ，加速度输入的不确定性

    QInit_ = Qdig.asDiagonal();
    QInit_ += B * Cu * B.transpose();//过程噪声不仅有状态本身的不确定性，还有IMU加速度输入传播进来的噪声

    low_pass_filters_.resize(3);
    low_pass_filters_[0] = std::make_shared<LowPassFilter>(dt_, 3.0);
    low_pass_filters_[1] = std::make_shared<LowPassFilter>(dt_, 3.0);
    low_pass_filters_[2] = std::make_shared<LowPassFilter>(dt_, 3.0);  //卡尔曼更新后的速度平滑，给速度加低通滤波

    csv_logger_.open(kEstimatorLogPath, {
        "sample",
        "time",
        "quat_w", "quat_x", "quat_y", "quat_z",
        "gyro_x", "gyro_y", "gyro_z",
        "acc_x", "acc_y", "acc_z",
        "pos_x", "pos_y", "pos_z",
        "vel_x", "vel_y", "vel_z",
        "yaw",
        "contact_fl", "contact_fr", "contact_rl", "contact_rr"
    });
}

double Estimator::getYaw() const {
    return rotMatToRPY(rotation_)(2);
}

void Estimator::update() {
    if (robot_model_->mass_ == 0) return;

    Q = QInit_;
    R = RInit_;//不是在旧 Q/R 上累加，而是每一帧重新从初值出发，再按接触情况调整。

    foot_poses_ = robot_model_->getFeet2BPositions();
    foot_vels_ = robot_model_->getFeet2BVelocities();
    feet_h_.setZero();//读取足端运动学

    // Adjust the covariance based on foot contact and phase.
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_[i] == 0) {
            // foot not contact
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = large_variance_ * Eigen::MatrixXd::Identity(3, 3);
            R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = large_variance_ * Eigen::MatrixXd::Identity(3, 3);
            R(24 + i, 24 + i) = large_variance_;//摆动腿的数据只保留参考价值，不强约机身状态
        } else {
            // foot contact
            const double trust = windowFunc(wave_generator_->phase_[i], 0.2);
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) =
                    (1 + (1 - trust) * large_variance_) *
                    QInit_.block(6 + 3 * i, 6 + 3 * i, 3, 3);
            R.block(12 + 3 * i, 12 + 3 * i, 3, 3) =
                    (1 + (1 - trust) * large_variance_) *
                    RInit_.block(12 + 3 * i, 12 + 3 * i, 3, 3);
            R(24 + i, 24 + i) =
                    (1 + (1 - trust) * large_variance_) * RInit_(24 + i, 24 + i);
        }//接触腿根据相位，接触边缘阶段不稳定，中间支撑阶段最可信
        feet_pos_body_.segment(3 * i, 3) = Vec3(foot_poses_[i].p.data);
        feet_vel_body_.segment(3 * i, 3) = Vec3(foot_vels_[i].data);//根据接触情况，修改腿对应的噪声，把腿的运动学量塞进观测向量缓存
    }

    Quat quat;
    quat << ctrl_interfaces_.imu_state_interface_[0].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[1].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[2].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    rotation_ = quatToRotMat(quat);  //四元数得到旋转矩阵，其实也就是姿态

    gyro_ << ctrl_interfaces_.imu_state_interface_[4].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[5].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[6].get().get_value();//读取角速度

    acceleration_ << ctrl_interfaces_.imu_state_interface_[7].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[8].get().get_value(),
            ctrl_interfaces_.imu_state_interface_[9].get().get_value();//读取线加速度

    u_ = rotation_ * acceleration_ + g_;
    x_hat_ = A * x_hat_ + B * u_;
    y_hat_ = C * x_hat_;//做一轮预测

    // Update the measurement value
    y_ << feet_pos_body_, feet_vel_body_, feet_h_;//明确更新阶段拿来纠正预测的量

    // Update the covariance matrix  卡尔曼更新
    Ppriori = A * P * A.transpose() + Q;  //状态传播后不确定性跟着传播，再叠加过程噪声
    S = R + C * Ppriori * C.transpose();
    Slu = S.lu();//观测残差  会有多大不确定性
    Sy = Slu.solve(y_ - y_hat_);
    Sc = Slu.solve(C);
    SR = Slu.solve(R);
    STC = S.transpose().lu().solve(C);
    IKC = Eigen::MatrixXd::Identity(18, 18) - Ppriori * C.transpose() * Sc;//避免显式求逆，改为解线性方程

    // Update the state and covariance matrix
    x_hat_ += Ppriori * C.transpose() * Sy;//更新状态
    P = IKC * Ppriori * IKC.transpose() +
        Ppriori * C.transpose() * SR * STC * Ppriori.transpose();//更新协方差

    // // Using low pass filter to smooth the velocity
    low_pass_filters_[0]->addValue(x_hat_(3));
    low_pass_filters_[1]->addValue(x_hat_(4));
    low_pass_filters_[2]->addValue(x_hat_(5));
    x_hat_(3) = low_pass_filters_[0]->getValue();
    x_hat_(4) = low_pass_filters_[1]->getValue();
    x_hat_(5) = low_pass_filters_[2]->getValue();

    log_counter_++;
    if (csv_logger_.isOpen() && log_counter_ % kEstimatorLogDecimation == 0) {
        csv_logger_.writeRow(
            log_counter_,
            log_counter_ * dt_,
            quat(0), quat(1), quat(2), quat(3),
            gyro_(0), gyro_(1), gyro_(2),
            acceleration_(0), acceleration_(1), acceleration_(2),
            x_hat_(0), x_hat_(1), x_hat_(2),
            x_hat_(3), x_hat_(4), x_hat_(5),
            getYaw(),
            wave_generator_->contact_(0),
            wave_generator_->contact_(1),
            wave_generator_->contact_(2),
            wave_generator_->contact_(3)
        );
    }
}
