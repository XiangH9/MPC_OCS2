//
// Created by tlab-uav on 24-9-16.
//

#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <memory>

#include "unitree_guide_controller/common/CsvLogger.h"
#include "unitree_guide_controller/common/mathTypes.h"
class QuadrupedRobot;

class BalanceCtrl {
public:
    explicit BalanceCtrl(const std::shared_ptr<QuadrupedRobot>& robot);//传入机器人模型，读取机器人质量，惯量等动力学参数

    ~BalanceCtrl() = default;

    /**
     * Calculate the desired feet end force
     * @param ddPcd desired body acceleration
     * @param dWbd desired body angular acceleration
     * @param rot_matrix current body rotation matrix
     * @param feet_pos_2_body feet positions to body under world frame
     * @param contact feet contact
     * @return
     */
    Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
               const Vec34 &feet_pos_2_body, const VecInt4 &contact);

private:
    void calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM);
    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM);
    void calConstraints(const VecInt4 &contact);
    void calSupportForceReference(const VecInt4 &contact);
    void solveQP();


    Mat12 G_, W_, U_;  //代价函数的权重大小  G 是QP的二次项  W是对力大小的正则项权重  U则是对力变化的正则项权重
    Mat6 S_;   //对机身wrench 加速度跟踪误差的权重矩阵
    Mat3 Ib_;  //机身惯量矩阵
    Vec6 bd_;  //目标动力学向量
    Vec3 g_, pcb_;  //重力向量  body frame下COM相对参考点的位置偏移
    Vec12 F_, F_prev_, F_ref_, g0T_;
    double mass_, alpha_, beta_, gamma_, friction_ratio_, min_normal_force_;

    Eigen::MatrixXd CE_, CI_;  //等式约束

    Eigen::VectorXd ce0_, ci0_;  //不等式约束
    Eigen::Matrix<double, 6, 12> A_;  //动力学，或者说四足机器人方程
    Eigen::Matrix<double, 5, 3> friction_mat_;  //单条腿线性摩擦约束矩阵
    CsvLogger csv_logger_;
    std::size_t log_counter_{0};
};


#endif //BALANCECTRL_H
