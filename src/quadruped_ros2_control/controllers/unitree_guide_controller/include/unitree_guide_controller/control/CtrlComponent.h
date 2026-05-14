#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H

#include <memory>
#include <string>
#include <atomic>
#include <thread>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <unitree_guide_controller/gait/WaveGenerator.h>
#include "BalanceCtrl.h"
#include "Estimator.h"

#include <Eigen/Core>

#include "unitree_guide_controller/nmpc/LeggedInterface.h"
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

//velocity command
#include <ocs2_core/reference/TargetTrajectories.h>

//System Observation
#include <ocs2_mpc/SystemObservation.h>

//Estimator-> RBDState
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

#include <unitree_guide_controller/gait/UnifiedGaitScheduler.h>

namespace ocs2 {
    class MPC_BASE;
    class MPC_MRT_Interface;
    class CentroidalModelRbdConversions;
}

class QuadrupedRobot;
struct CtrlInterfaces;

class CtrlComponent {
public:
    CtrlComponent() = default;
    ~CtrlComponent();
    //joint position velocity
    void bindCtrlInterfaces(CtrlInterfaces& ctrl_interfaces);
    // 访问关节状态、命令接口、IMU状态等
    //读取NMPC相关配置并且确认配置是否有效
    void setupNmpcConfig(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);
    bool hasNmpcConfig() const;
    //内部和MPC相关的缓存变量初始化/清零
    void setupMpcObjects();
    bool hasMpcObjects() const;
    //建立NMPC基础接口层，确认接口是否ready
    void setupLeggedInterface();
    bool isNmpcInterfaceReady() const;
    //真正创建运行时的求解器
    void setupMpcRuntime();
    bool isMpcRuntimeReady() const;
    // Existing guide-controller components
    std::shared_ptr<QuadrupedRobot> robot_model_;//正逆运动学、雅可比、足端力到关节力矩映射
    std::shared_ptr<Estimator> estimator_;//机身位置、速度、姿态、足端状态估计
    std::shared_ptr<BalanceCtrl> balance_ctrl_;//机身期望加速度/角加速度  四足力分配
    std::shared_ptr<WaveGenerator> wave_generator_;//步态生成
    //Joint Position and Velocity
    CtrlInterfaces* ctrl_interfaces_ptr_{nullptr};
    //ROBOT joints and feet
    std::vector<std::string> joint_names_;
    std::vector<std::string> feet_names_;
    // NMPC configuration placeholders
    bool nmpc_enabled_{false};
    std::string robot_pkg_;
    std::string urdf_file_;
    std::string task_file_;
    std::string reference_file_;
    std::string gait_file_;
    // MPC-only runtime placeholders
    std::shared_ptr<unitree_guide_controller::nmpc::LeggedInterface> legged_interface_; //问题定义接口层，读取配置，准备模型，提供优化问题、提供初始化
    std::shared_ptr<ocs2::MPC_BASE> mpc_;  //mpc求解器本体
    std::shared_ptr<ocs2::MPC_MRT_Interface> mpc_mrt_interface_;//和外界交互
    std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbd_conversions_;//刚体动力学状态转换器、把估计器状态转成OCS2 centroidal model的状态格式
    // Measured/observed quantities:
    // observation_ is the current observation passed into MPC.
    // Its state should come from measured_rbd_state_, not from optimized prediction.
    ocs2::SystemObservation observation_;// 系统观测缓存
    Eigen::VectorXd measured_rbd_state_;//当前测得的刚体动力学状态

    // MPC runtime context:
    // these values are the cached time/input/mode context that the runtime carries forward.
    double mpc_time_{0.0};
    Eigen::VectorXd mpc_state_;//状态量
    Eigen::VectorXd mpc_input_;//输入量，其实就是控制量
    size_t mpc_mode_{0};

    // Latest solver outputs:
    // these are produced by evaluatePolicy() before being selectively synced into runtime context.
    Eigen::VectorXd optimized_state_;//优化状态   预测结果
    Eigen::VectorXd optimized_input_;//优化输入   可执行控制量
    size_t planned_mode_{0}; //策略预测模式
    bool policy_available_{false};

    ocs2::TargetTrajectories latest_target_trajectories_;

    // Execution-facing command caches:
    // desired body velocity is the reference input,
    // joint velocity command is the extracted command sent toward ros2_control interfaces.
    Eigen::VectorXd nmpc_joint_position_cmd_; //从优化状态中截取的关节角部分，缓存到这里
    Eigen::VectorXd nmpc_joint_velocity_cmd_; //从优化输入中截取的关节速度部分，缓存到这里
    Eigen::Vector3d nmpc_desired_body_velocity_{Eigen::Vector3d::Zero()};  // vx, vy, yaw_rate

    std::atomic_bool mpc_running_{false};  //mpc线程是否运行中
    std::atomic_bool controller_running_{false};//控制器整体
    std::thread mpc_thread_;//工作线程对象
    std::atomic_bool mpc_thread_faulted_{false};
    std::atomic<uint64_t> mpc_thread_cycle_count_{0};
    std::atomic<double> mpc_thread_last_observation_time_{-1.0};
    std::atomic<double> mpc_thread_last_final_time_{-1.0};
    bool policy_expired_reported_{false};
    bool no_fresh_policy_reported_{false};
    double last_fresh_policy_swap_time_{-1.0};


    bool mpc_objects_initialized_{false};
    bool mpc_runtime_ready_{false};// 两极ready标志，MPC相关对象是否初始化  MPC运行时是否准备好

    bool hasMpcRuntimeShell() const;  //求解器相关对象是否至少已经建立出来

    //SystemObservation
    void advanceMpcOnce();  
    void pushCurrentObservationToMpc();
    bool hasLatestPolicy() const;

    const Eigen::VectorXd& getNmpcJointPositionCommand() const;
    bool hasNmpcJointPositionCommand() const;
    const Eigen::VectorXd& getNmpcJointVelocityCommand() const;
    bool hasNmpcJointVelocityCommand() const;
    size_t getPlannedMode() const;

    //Test Policy optimized input
    bool evaluateCurrentPolicy();

    //Estimator->RBDstate
    void updateMeasuredRbdStateFromEstimator();

    //velocity command
    void updateDesiredBodyVelocityCommand(double vx, double vy, double yaw_rate);  //写入新的机身速度目标

    void updateTargetTrajectoriesFromCommand(); //根据观测状态和nmpc和期望速度生成目标轨迹

    void advanceMpcClock(double dt);
    void syncMpcRuntimeFromLatestPolicy();

    void startMpcThread();
    void stopMpcThread();

};

#endif // CTRLCOMPONENT_H
