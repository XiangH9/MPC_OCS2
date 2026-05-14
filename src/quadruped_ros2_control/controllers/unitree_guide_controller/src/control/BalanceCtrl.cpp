//
// Created by tlab-uav on 24-9-16.
//

#include "unitree_guide_controller/control/BalanceCtrl.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

#include "quadProgpp/QuadProg++.hh"

namespace {
constexpr const char* kBalanceLogPath = "/home/xiangh9/xhros2/uni_ws/tmp/unitree_phase1_logs/balance.csv";
constexpr std::size_t kBalanceLogDecimation = 20;
}

BalanceCtrl::BalanceCtrl(const std::shared_ptr<QuadrupedRobot> &robot) {
    mass_ = robot->mass_;

    alpha_ = 0.001;
    beta_ = 0.1;
    gamma_ = 0.05;
    min_normal_force_ = 15.0;

    g_ << 0, 0, -9.81;
    friction_ratio_ = 0.4;

    friction_mat_ << 1, 0, friction_ratio_, -1, 0, friction_ratio_, 0, 1, friction_ratio_, 0, -1,
            friction_ratio_, 0, 0, 1;  //线性摩擦锥近似

    pcb_ = Vec3(0.0, 0.0, 0.0);  //默认参考点在质心
    Ib_ = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();  //机身惯量是写死的对角矩阵
    //当前惯量不是从URDF或机器人模型在线读取  机器人型号，负载，坐标定义发生变化，机器人可能会出现失真

    Vec6 s;
    Vec12 w, u;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    s << 20, 20, 50, 450, 450, 450;

    S_ = s.asDiagonal();   // 机身动力学误差的权重   后 3 个角向量的权重很大，说明作者很重视姿态/角加速度控制
    W_ = w.asDiagonal();     // 力大小惩罚
    U_ = u.asDiagonal();   // 力变化惩罚

    F_prev_.setZero();   //因为有和之前力变化的约束，所以优化器会倾向于从零力平滑的建立接触力
    F_ref_.setZero();


    csv_logger_.open(kBalanceLogPath, {
        "sample",
        "dd_pcd_x", "dd_pcd_y", "dd_pcd_z",
        "d_wbd_x", "d_wbd_y", "d_wbd_z",
        "bd_x", "bd_y", "bd_z", "bd_rx", "bd_ry", "bd_rz",
        "force_fl_x", "force_fl_y", "force_fl_z",
        "force_fr_x", "force_fr_y", "force_fr_z",
        "force_rl_x", "force_rl_y", "force_rl_z",
        "force_rr_x", "force_rr_y", "force_rr_z",
        "contact_fl", "contact_fr", "contact_rl", "contact_rr"
    });
}

//真正的足端力分配入口
Vec34 BalanceCtrl::calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
                        const Vec34 &feet_pos_2_body, const VecInt4 &contact) {


    calMatrixA(feet_pos_2_body, rot_matrix);
    calVectorBd(ddPcd, dWbd, rot_matrix);
    calConstraints(contact);
    calSupportForceReference(contact);

    G_ = A_.transpose() * S_ * A_ + (alpha_ + gamma_) * W_ + beta_ * U_;
    g0T_ = -bd_.transpose() * S_ * A_
        - gamma_ * F_ref_.transpose() * W_
        - beta_ * F_prev_.transpose() * U_;

    solveQP();

    F_prev_ = F_;

    log_counter_++;
    if (csv_logger_.isOpen() && log_counter_ % kBalanceLogDecimation == 0) {
        csv_logger_.writeRow(
            log_counter_,
            ddPcd(0), ddPcd(1), ddPcd(2),
            dWbd(0), dWbd(1), dWbd(2),
            bd_(0), bd_(1), bd_(2), bd_(3), bd_(4), bd_(5),
            F_(0), F_(1), F_(2),
            F_(3), F_(4), F_(5),
            F_(6), F_(7), F_(8),
            F_(9), F_(10), F_(11),
            contact(0), contact(1), contact(2), contact(3)
        );
    }

    return vec12ToVec34(F_);
}

void BalanceCtrl::calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM) {
    for (int i = 0; i < 4; ++i) {
        A_.block(0, 3 * i, 3, 3) = I3;
        A_.block(3, 3 * i, 3, 3) = skew(Vec3(feet_pos_2_body.col(i)) - rotM * pcb_);
    }
}//应该叫动力学方程  和运动学方程有区别

void BalanceCtrl::calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM) {
    bd_.head(3) = mass_ * (ddPcd - g_);
    bd_.tail(3) = rotM * Ib_ * rotM.transpose() * dWbd;
}//建立期望机身动力学目标    四足合起来产生什么样的wrench

void BalanceCtrl::calConstraints(const VecInt4 &contact) {
    int contactLegNum = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            contactLegNum += 1;
        }
    }//计算接触腿的数量

    CI_.resize(5 * contactLegNum, 12);
    ci0_.resize(5 * contactLegNum);
    CE_.resize(3 * (4 - contactLegNum), 12);
    ce0_.resize(3 * (4 - contactLegNum));   //每条支撑腿5个不等式约束  摆动腿3个不等式约束

    CI_.setZero();
    ci0_.setZero();
    CE_.setZero();
    ce0_.setZero();

    int ceID = 0;
    int ciID = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            CI_.block(5 * ciID, 3 * i, 5, 3) = friction_mat_;
            ci0_.segment(5 * ciID, 5).setZero();
            ci0_(5 * ciID + 4) = -min_normal_force_;
            ++ciID;
        } else {
            CE_.block(3 * ceID, 3 * i, 3, 3) = I3;
            ++ceID;
        }
    }//支撑腿 把该腿的 3 维力放进摩擦锥不等式      摆动腿 直接约束这条腿的 3 维力等于 0
}

void BalanceCtrl::calSupportForceReference(const VecInt4 &contact) {
    F_ref_.setZero();

    int contact_leg_num = 0;
    for (int i = 0; i < 4; ++i) {
        if (contact(i) == 1) {
            contact_leg_num += 1;
        }
    }

    if (contact_leg_num == 0) {
        return;
    }

    const double nominal_fz =
        mass_ * std::abs(g_(2)) / static_cast<double>(contact_leg_num);

    for (int i = 0; i < 4; ++i) {
        if (contact(i) == 1) {
            F_ref_(3 * i + 2) = nominal_fz;
        }
    }
}



void BalanceCtrl::solveQP() {
    const long n = F_.size();
    const long m = ce0_.size();
    const long p = ci0_.size();

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = G_(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = CE_.transpose()(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = CI_.transpose()(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = g0T_[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = ce0_[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = ci0_[i];
    }

    solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i) {
        F_[i] = x[i];
    }
}
