#include "unitree_guide_controller/control/CtrlComponent.h"
#include "controller_common/CtrlInterfaces.h"


#include <ament_index_cpp/get_package_share_directory.hpp>   //拼配置文件路径
#include<rclcpp/rclcpp.hpp>  //打印ROS日志
#include <ocs2_sqp/SqpMpc.h>   //真正创建SQP求解器
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
// #include <ocs2_legged_robot/LeggedRobotInterface.h>

//Estimator->RBDState
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <unitree_guide_controller/common/mathTools.h>

//velocity command
#include <cmath>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

//reference
#include <vector>




//Get Joint Position and Velocity 
CtrlComponent::~CtrlComponent() {
    stopMpcThread();
}

void CtrlComponent::bindCtrlInterfaces(CtrlInterfaces& ctrl_interfaces) {
    ctrl_interfaces_ptr_ = &ctrl_interfaces;
}
//
//单纯绑定接口

//读取NMPC参数配置
void CtrlComponent::setupNmpcConfig(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node) {
    if (!node->has_parameter("robot_pkg")) {
        node->declare_parameter("robot_pkg", "");
    }
    //robot_pkg是在哪里发布的

    robot_pkg_ = node->get_parameter("robot_pkg").as_string();
    nmpc_enabled_ = !robot_pkg_.empty(); //robot_pkg_非空就启用NMPC

    RCLCPP_INFO(node->get_logger(), "NMPC raw robot_pkg: '%s'", robot_pkg_.c_str());
    RCLCPP_INFO(node->get_logger(), "NMPC enabled: %s", nmpc_enabled_ ? "true" : "false");


    if (!nmpc_enabled_) {
        urdf_file_.clear();
        task_file_.clear();
        reference_file_.clear();
        gait_file_.clear();
        RCLCPP_WARN(node->get_logger(), "NMPC config disabled because robot_pkg is empty");
        return;
    }

    const std::string package_share_directory =
        ament_index_cpp::get_package_share_directory(robot_pkg_);

    urdf_file_ = package_share_directory + "/urdf/robot.urdf";
    task_file_ = package_share_directory + "/config/ocs2/task.info";
    reference_file_ = package_share_directory + "/config/ocs2/reference.info";
    gait_file_ = package_share_directory + "/config/ocs2/gait.info";

    //如果启用，就要配置文件路径，步态配置

    RCLCPP_INFO(node->get_logger(), "NMPC robot_pkg: %s", robot_pkg_.c_str());
    RCLCPP_INFO(node->get_logger(), "NMPC urdf_file: %s", urdf_file_.c_str());
    RCLCPP_INFO(node->get_logger(), "NMPC task_file: %s", task_file_.c_str());
    RCLCPP_INFO(node->get_logger(), "NMPC reference_file: %s", reference_file_.c_str());
    RCLCPP_INFO(node->get_logger(), "NMPC gait_file: %s", gait_file_.c_str());

}

bool CtrlComponent::hasNmpcConfig() const {
    return nmpc_enabled_;
}


//函数的作用不是创建求解器，而是“把和 MPC 相关的内部缓存全部重置”。
void CtrlComponent::setupMpcObjects() {
    //观测
    observation_.time = 0.0;
    observation_.state.resize(0);
    observation_.input.resize(0);
    observation_.mode = 0;
    //当前状态
    mpc_time_ = 0.0;
    mpc_state_.resize(0);
    mpc_input_.resize(0);
    mpc_mode_ = 0;
    measured_rbd_state_.resize(0);
    //POLICY缓存
    optimized_state_.resize(0);
    optimized_input_.resize(0);
    planned_mode_ = 0;
    policy_available_ = false;

    //关节命令缓存
    nmpc_joint_position_cmd_.resize(0);
    nmpc_joint_velocity_cmd_.resize(0);

    latest_target_trajectories_ = ocs2::TargetTrajectories();

}


//检查3个关键对象是否已经存在
bool CtrlComponent::hasMpcObjects() const {
    return static_cast<bool>(legged_interface_) &&
           static_cast<bool>(mpc_) &&
           static_cast<bool>(mpc_mrt_interface_);
}

//建立NMPC的基础问题接口层，所以问题的定义又是在LeggedInterface去完成的
void CtrlComponent::setupLeggedInterface() {
    if (!nmpc_enabled_) {
        std::cout << "[CtrlComponent] NMPC is disabled, skip LeggedInterface setup." << std::endl;
        return;
    }

    //相关配置交给LeggedInterface去构建问题定义
    legged_interface_ = std::make_shared<unitree_guide_controller::nmpc::LeggedInterface>(
        task_file_, urdf_file_, reference_file_);
    

    legged_interface_->setupJointNames(joint_names_, feet_names_);

    if (!legged_interface_->initialize()) {
        std::cerr << "[CtrlComponent] LeggedInterface initialization failed." << std::endl;
        return;
    }

    std::cout << "[CtrlComponent] LeggedInterface setup completed." << std::endl;
}

//检查是否成功
bool CtrlComponent::isNmpcInterfaceReady() const {
    return nmpc_enabled_ &&
           static_cast<bool>(legged_interface_) &&
           legged_interface_->isInitialized();
}


//最关键的函数，从leggedInterface进一步建立真正可运行的MPC runtime
void CtrlComponent::setupMpcRuntime() {
    stopMpcThread();

    mpc_objects_initialized_ = false;
    mpc_runtime_ready_ = false;//防止沿用旧状态

    if (!isNmpcInterfaceReady()) {
        std::cerr << "[CtrlComponent] Cannot setup MPC runtime because NMPC interface is not ready." << std::endl;
        return;
    }

    //在LeggedInterface定义优化问题
    if (!legged_interface_->hasOptimalControlProblem()) {
        std::cerr << "[CtrlComponent] OptimalControlProblem is not ready." << std::endl;
        return;
    }

    if (!legged_interface_->hasRollout()) {
        std::cerr << "[CtrlComponent] Rollout is not ready." << std::endl;
        return;
    }

    std::cout << "[CtrlComponent] setupMpcRuntime() called." << std::endl;


    
    //读取初始状态和模型信息
    const auto& initial_state = legged_interface_->getInitialState();
    const auto& model_info = legged_interface_->getCentroidalModelInfo();

    //rbd State
    rbd_conversions_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(
        legged_interface_->getPinocchioInterface(),
        model_info);


    if (!legged_interface_->hasReferenceManager()) {
        std::cerr << "[CtrlComponent] ReferenceManager is not ready." << std::endl;
        return;
    }

    if (!legged_interface_->hasInitializer()) {
        std::cerr << "[CtrlComponent] Initializer is not ready." << std::endl;
        return;
    }
    //参考轨迹管理器和初始化器

    //当前状态缓存
    mpc_time_ = 0.0;
    mpc_mode_ = 15;
    mpc_state_ = initial_state;
    mpc_input_ = Eigen::VectorXd::Zero(model_info.inputDim);
    measured_rbd_state_ = initial_state;

//在 runtime 刚建立时，solver 立刻就需要一份合法、维度正确、语义完整的当前状态，不能等第一次 estimator 更新之后再说；用 initial_state 作为一个可用的启动占位

    //观测缓存
    observation_.time = mpc_time_;
    observation_.state = mpc_state_;
    observation_.input = mpc_input_;
    observation_.mode = mpc_mode_;
    
    //policy缓存
    optimized_state_ = mpc_state_;
    optimized_input_ = mpc_input_;
    planned_mode_ = mpc_mode_;
    policy_available_ = false;

    policy_expired_reported_ = false;
    no_fresh_policy_reported_ = false;
    last_fresh_policy_swap_time_ = -1.0;
    mpc_thread_faulted_ = false;
    mpc_thread_cycle_count_ = 0;
    mpc_thread_last_observation_time_ = -1.0;
    mpc_thread_last_final_time_ = -1.0;

    //位置和速度命令都有合法缓存
    nmpc_joint_position_cmd_ = Eigen::VectorXd::Zero(model_info.actuatedDofNum);
    nmpc_joint_velocity_cmd_ = Eigen::VectorXd::Zero(model_info.actuatedDofNum);

    mpc_.reset();
    mpc_mrt_interface_.reset();


//OptimalProblems    SQP create
    try {
        auto mpc_ptr = std::make_shared<ocs2::SqpMpc>(
            legged_interface_->getMpcSettings(),
            legged_interface_->getSqpSettings(),
            legged_interface_->getOptimalControlProblem(),
            legged_interface_->getInitializer());
        //相关配置全部传给OCS2的SqpMpc构造函数

        mpc_ptr->getSolverPtr()->setReferenceManager(
            legged_interface_->getReferenceManagerPtr());//从 reference.info 和 GaitSchedule 构出来的参考管理器 装进solver

        mpc_ = mpc_ptr;
        mpc_mrt_interface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

        const ocs2::TargetTrajectories initial_target_trajectories(
            {observation_.time},
            {observation_.state},
            {observation_.input});//并不是我们构造的future trajectory 启动占位

        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(initial_target_trajectories);

        std::cout << "[CtrlComponent] Waiting for initial MPC policy..." << std::endl;

        size_t init_policy_counter = 0;
        const size_t max_init_policy_iterations = 200;
        const double mrt_frequency = legged_interface_->getMpcSettings().mrtDesiredFrequency_;
        const auto sleep_dt = std::chrono::duration<double>(1.0 / std::max(1.0, mrt_frequency));

        while (!mpc_mrt_interface_->initialPolicyReceived() &&
            init_policy_counter < max_init_policy_iterations) {
            mpc_mrt_interface_->advanceMpc();
            std::this_thread::sleep_for(sleep_dt);
            ++init_policy_counter;
        }

        if (!mpc_mrt_interface_->initialPolicyReceived()) {
            std::cerr << "[CtrlComponent] Failed to receive initial MPC policy." << std::endl;
            mpc_.reset();
            mpc_mrt_interface_.reset();
            mpc_objects_initialized_ = false;
            mpc_runtime_ready_ = false;
            return;
        }

        mpc_objects_initialized_ = true;
        mpc_runtime_ready_ = true;
        startMpcThread();

        std::cout << "[CtrlComponent] SqpMpc created successfully." << std::endl;
        std::cout << "[CtrlComponent] MPC_MRT_Interface created successfully." << std::endl;
        std::cout << "[CtrlComponent] Initial state dimension: " << mpc_state_.size() << std::endl;
        std::cout << "[CtrlComponent] Input dimension: " << mpc_input_.size() << std::endl;
        std::cout << "[CtrlComponent] Initial MPC policy has been received." << std::endl;
        std::cout << "[CtrlComponent] MPC runtime is ready." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[CtrlComponent] Failed to create real MPC runtime: "
                  << e.what() << std::endl;
        mpc_.reset();
        mpc_mrt_interface_.reset();
        mpc_objects_initialized_ = false;
        mpc_runtime_ready_ = false;
        return;
    }

}


bool CtrlComponent::isMpcRuntimeReady() const {
    return mpc_objects_initialized_ && mpc_runtime_ready_;
}


bool CtrlComponent::hasMpcRuntimeShell() const {
    return mpc_objects_initialized_;
}

void CtrlComponent::startMpcThread() {
    if (!isMpcRuntimeReady() || mpc_thread_.joinable()) {
        return;
    }

    controller_running_ = true;
    mpc_running_ = true;
    mpc_thread_faulted_ = false;
    mpc_thread_cycle_count_ = 0;
    mpc_thread_last_observation_time_ = observation_.time;
    mpc_thread_last_final_time_ = -1.0;

    mpc_thread_ = std::thread([this]() {
        while (controller_running_) {
            try {
                ocs2::executeAndSleep(
                    [this]() {
                        if (mpc_running_) {
                            static int mpc_thread_tick = 0;
                            ++mpc_thread_tick;
                            mpc_thread_cycle_count_.fetch_add(1, std::memory_order_relaxed);
                            mpc_thread_last_observation_time_.store(
                                observation_.time, std::memory_order_relaxed);

                            if (mpc_thread_tick % 100 == 0) {
                                std::cout << "[CtrlComponent] MPC thread tick: "
                                        << "obs_time=" << observation_.time
                                        << ", mpc_time_=" << mpc_time_
                                        << ", cycle_count="
                                        << mpc_thread_cycle_count_.load(std::memory_order_relaxed)
                                        << std::endl;
                            }

                            mpc_mrt_interface_->advanceMpc();
                        }
                    },
                    legged_interface_->getMpcSettings().mpcDesiredFrequency_);
            } catch (const std::exception& e) {
                controller_running_ = false;
                mpc_running_ = false;
                mpc_thread_faulted_ = true;
                std::cerr << "[CtrlComponent] MPC thread failed: " << e.what()
                          << ", cycle_count="
                          << mpc_thread_cycle_count_.load(std::memory_order_relaxed)
                          << ", last_obs_time="
                          << mpc_thread_last_observation_time_.load(std::memory_order_relaxed)
                          << ", last_policy_final_time="
                          << mpc_thread_last_final_time_.load(std::memory_order_relaxed)
                          << std::endl;
            }
        }
    });

    ocs2::setThreadPriority(legged_interface_->getSqpSettings().threadPriority, mpc_thread_);
    std::cout << "[CtrlComponent] MPC thread started." << std::endl;
}

void CtrlComponent::stopMpcThread() {
    mpc_running_ = false;
    controller_running_ = false;
    if (mpc_thread_.joinable()) {
        mpc_thread_.join();
    }
}

void CtrlComponent::pushCurrentObservationToMpc() {
    if (!isMpcRuntimeReady()) {
        return;
    }

    observation_.time = mpc_time_;
    observation_.state = measured_rbd_state_;
    observation_.input = mpc_input_;
    observation_.mode = mpc_mode_;
    mpc_mrt_interface_->setCurrentObservation(observation_);
}

// Push the current observation into MPC and advance the solver once.
void CtrlComponent::advanceMpcOnce() {
    if (!isMpcRuntimeReady()) {
        return;
    }

    pushCurrentObservationToMpc();

    try {
        std::cout << "[CtrlComponent] advanceMpcOnce(): before setCurrentObservation" << std::endl;
        std::cout << "[CtrlComponent] observation mode before advanceMpc: "
          << observation_.mode << std::endl;

        std::cout << "[CtrlComponent] advanceMpcOnce(): before advanceMpc" << std::endl;
        mpc_mrt_interface_->advanceMpc();

        std::cout << "[CtrlComponent] advanceMpcOnce() succeeded." << std::endl;

        std::cout << "[CtrlComponent] observation time: " << observation_.time << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[CtrlComponent] advanceMpcOnce() failed: "
                  << e.what() << std::endl;
    }
}

bool CtrlComponent::hasLatestPolicy() const {
    return isMpcRuntimeReady() && policy_available_;
}

const Eigen::VectorXd& CtrlComponent::getNmpcJointPositionCommand() const {
    return nmpc_joint_position_cmd_;
}

bool CtrlComponent::hasNmpcJointPositionCommand() const {
    return hasLatestPolicy() && nmpc_joint_position_cmd_.size() > 0;
}

const Eigen::VectorXd& CtrlComponent::getNmpcJointVelocityCommand() const {
    return nmpc_joint_velocity_cmd_;
}

bool CtrlComponent::hasNmpcJointVelocityCommand() const {
    return hasLatestPolicy() && nmpc_joint_velocity_cmd_.size() > 0;
}

size_t CtrlComponent::getPlannedMode() const {
    return planned_mode_;
}

//velocity command
void CtrlComponent::updateDesiredBodyVelocityCommand(double vx, double vy, double yaw_rate) {
    nmpc_desired_body_velocity_ << vx, vy, yaw_rate;
}


// Build a short-horizon target trajectory from the latest measured observation and desired body velocity command.
void CtrlComponent::updateTargetTrajectoriesFromCommand() {
    if (!isMpcRuntimeReady()) {
        return;
    }

    if (!legged_interface_ || !legged_interface_->hasReferenceManager()) {
        return;
    }

    const auto& model_info = legged_interface_->getCentroidalModelInfo();

    if (observation_.state.size() != static_cast<Eigen::Index>(model_info.stateDim)) {
        std::cerr << "[CtrlComponent] updateTargetTrajectoriesFromCommand(): observation state dim mismatch." << std::endl;
        return;
    }

    if (!mpc_mrt_interface_) {
        std::cerr << "[CtrlComponent] updateTargetTrajectoriesFromCommand(): MRT interface is null." << std::endl;
        return;
    }

    auto& reference_manager = mpc_mrt_interface_->getReferenceManager();


    const double t0 = observation_.time;
    const double dt = 0.3;
    constexpr size_t num_points = 4;

    const double vx_body = nmpc_desired_body_velocity_(0);
    const double vy_body = nmpc_desired_body_velocity_(1);
    const double yaw_rate = nmpc_desired_body_velocity_(2);

    std::vector<ocs2::scalar_t> time_trajectory;
    std::vector<ocs2::vector_t> state_trajectory;
    std::vector<ocs2::vector_t> input_trajectory;

    time_trajectory.reserve(num_points);
    state_trajectory.reserve(num_points);
    input_trajectory.reserve(num_points);

    Eigen::VectorXd current_state = observation_.state;

    for (size_t k = 0; k < num_points; ++k) {
        time_trajectory.push_back(t0 + static_cast<double>(k) * dt);
        state_trajectory.push_back(current_state);
        input_trajectory.push_back(Eigen::VectorXd::Zero(mpc_input_.size()));

        if (k + 1 < num_points) {
            auto current_base_pose = ocs2::centroidal_model::getBasePose(current_state, model_info);

            const double yaw = current_base_pose(3);
            const double vx_world = std::cos(yaw) * vx_body - std::sin(yaw) * vy_body;
            const double vy_world = std::sin(yaw) * vx_body + std::cos(yaw) * vy_body;

            //传入的类似于引用，不需要显视写回改变也能满足要求
            current_base_pose(0) += vx_world * dt;
            current_base_pose(1) += vy_world * dt;
            current_base_pose(3) += yaw_rate * dt;

        }
    }

    ocs2::TargetTrajectories target_trajectories(
        time_trajectory,
        state_trajectory,
        input_trajectory
    );

    latest_target_trajectories_ = target_trajectories;

    reference_manager.setTargetTrajectories(target_trajectories);


    static int ref_log_counter = 0;
    ref_log_counter++;
    if (ref_log_counter % 20 == 0) {
        const auto& x_start = state_trajectory.front();
        const auto& x_end = state_trajectory.back();

        const auto base_pose_x0 = ocs2::centroidal_model::getBasePose(x_start, model_info);
        const auto base_pose_x1 = ocs2::centroidal_model::getBasePose(x_end, model_info);


        std::cout << "[CtrlComponent] target cmd vx: " << vx_body
                << ", vy: " << vy_body
                << ", yaw_rate: " << yaw_rate << std::endl;

        std::cout << "[CtrlComponent] x0 base: "
                << "px=" << base_pose_x0(0)
                << ", py=" << base_pose_x0(1)
                << ", pz=" << base_pose_x0(2)
                << ", yaw=" << base_pose_x0(3)
                << ", pitch=" << base_pose_x0(4)
                << ", roll=" << base_pose_x0(5) << std::endl;


        std::cout << "[CtrlComponent] x1 base: "
                << "px=" << base_pose_x1(0)
                << ", py=" << base_pose_x1(1)
                << ", pz=" << base_pose_x1(2)
                << ", yaw=" << base_pose_x1(3)
                << ", pitch=" << base_pose_x1(4)
                << ", roll=" << base_pose_x1(5) << std::endl;


        std::cout << "[CtrlComponent] delta base: "
                << "dpx=" << (base_pose_x1(0) - base_pose_x0(0))
                << ", dpy=" << (base_pose_x1(1) - base_pose_x0(1))
                << ", dyaw=" << (base_pose_x1(3) - base_pose_x0(3)) << std::endl;
    }


}

// test optimized policy
//把当前策略评估出来并且缓存结果
// Pull the latest policy from MRT, cache solver outputs, and sync runtime context where appropriate.
bool CtrlComponent::evaluateCurrentPolicy() {
    if (!isMpcRuntimeReady()) {
        std::cerr << "[CtrlComponent] evaluateCurrentPolicy(): MPC runtime is not ready." << std::endl;
        policy_available_ = false;
        return false;
    }

    const bool has_new_policy = mpc_mrt_interface_->updatePolicy();
    const auto policy = mpc_mrt_interface_->getPolicy();
    const bool policy_empty = policy.timeTrajectory_.empty();
    const bool policy_expired =
    !policy_empty && observation_.time > policy.timeTrajectory_.back();

    static int reused_policy_counter = 0;

    if (has_new_policy) {
        reused_policy_counter = 0;
        policy_expired_reported_ = false;
        no_fresh_policy_reported_ = false;
        last_fresh_policy_swap_time_ = observation_.time;
        if (!policy_empty) {
            mpc_thread_last_final_time_.store(
                policy.timeTrajectory_.back(), std::memory_order_relaxed);
        }
        std::cout << "[CtrlComponent] evaluateCurrentPolicy(): fresh MPC policy swapped in. "
                << "currentTime=" << observation_.time;
        if (!policy_empty) {
            std::cout << ", policyStartTime=" << policy.timeTrajectory_.front()
                      << ", policyFinalTime=" << policy.timeTrajectory_.back();
        }
        std::cout << std::endl;
        std::cout << "[CtrlComponent] evaluateCurrentPolicy(): fresh policy mode sequence:";
        for (size_t mode : policy.modeSchedule_.modeSequence) {
            std::cout << " " << mode;
        }
        std::cout << std::endl;
        std::cout << "[CtrlComponent] evaluateCurrentPolicy(): fresh policy event times:";
        for (double t : policy.modeSchedule_.eventTimes) {
            std::cout << " " << t;
        }
        std::cout << std::endl;

    } else {
        ++reused_policy_counter;
        if (policy_empty || policy_expired) {
            if (!no_fresh_policy_reported_ || policy_expired) {
                std::cerr << "[CtrlComponent] evaluateCurrentPolicy(): no fresh MPC policy is available. "
                          << "has_new_policy=" << has_new_policy
                          << ", policy_empty=" << policy_empty
                          << ", policy_expired=" << policy_expired
                          << ", mpc_running=" << mpc_running_.load()
                          << ", controller_running=" << controller_running_.load()
                          << ", mpc_thread_faulted=" << mpc_thread_faulted_.load();
                if (!policy_empty) {
                    std::cerr << ", currentTime=" << observation_.time
                              << ", policyStartTime=" << policy.timeTrajectory_.front()
                              << ", policyFinalTime=" << policy.timeTrajectory_.back()
                              << ", stale_duration="
                              << ((last_fresh_policy_swap_time_ >= 0.0)
                                      ? (observation_.time - last_fresh_policy_swap_time_)
                                      : -1.0);
                }
                std::cerr << std::endl;
                no_fresh_policy_reported_ = true;
            }

            if (policy_expired) {
                if (!policy_expired_reported_) {
                    std::cerr << "[CtrlComponent] evaluateCurrentPolicy(): active policy expired. "
                              << "Runtime recovery is intentionally disabled to match the reference "
                              << "OCS2 control flow: setCurrentObservation() + advanceMpc() + "
                              << "updatePolicy() + evaluatePolicy(). "
                              << "According to OCS2 MPC_BASE::run(), once currentTime >= solver finalTime, "
                              << "advanceMpc() stops producing new buffered policies until the runtime is rebuilt."
                              << std::endl;
                    policy_expired_reported_ = true;
                }
                policy_available_ = false;
                return false;
            }


            policy_available_ = false;
            return false;
        }

        std::cout << "[CtrlComponent] evaluateCurrentPolicy(): reusing active MPC policy. "
                << "reuse_count=" << reused_policy_counter
                << ", currentTime=" << observation_.time
                << ", policyStartTime=" << policy.timeTrajectory_.front()
                << ", policyFinalTime=" << policy.timeTrajectory_.back()
                << std::endl;
    }

    ocs2::vector_t optimized_state;
    ocs2::vector_t optimized_input;
    size_t planned_mode = mpc_mode_;

    try {
        mpc_mrt_interface_->evaluatePolicy(
            observation_.time,
            observation_.state,
            optimized_state,
            optimized_input,
            planned_mode);
    } catch (const std::exception& e) {
        std::cerr << "[CtrlComponent] evaluateCurrentPolicy(): evaluatePolicy failed: "
                  << e.what() << std::endl;
        policy_available_ = false;
        return false;
    }

    optimized_state_ = optimized_state;
    optimized_input_ = optimized_input;
    planned_mode_ = planned_mode;
    policy_available_ = true;

    std::cout << "[CtrlComponent] Latest NMPC policy cached successfully." << std::endl;
    std::cout << "[CtrlComponent] optimized state dim: " << optimized_state_.size() << std::endl;
    std::cout << "[CtrlComponent] optimized input dim: " << optimized_input_.size() << std::endl;
    std::cout << "[CtrlComponent] planned mode: " << planned_mode_ << std::endl;
    if (optimized_input_.size() > 0) {
        std::cout << "[CtrlComponent] optimized input[0]: " << optimized_input_[0] << std::endl;
    }

    const Eigen::Index joint_dim =
        static_cast<Eigen::Index>(legged_interface_->getCentroidalModelInfo().actuatedDofNum);

    if (optimized_state_.size() >= 12 + joint_dim) {
        const auto joint_pos = optimized_state_.segment(12, joint_dim);
        nmpc_joint_position_cmd_ = joint_pos;
        std::cout << "[CtrlComponent] candidate joint_pos[0]: " << joint_pos[0] << std::endl;
        std::cout << "[CtrlComponent] cached nmpc_joint_position_cmd dim: "
                  << nmpc_joint_position_cmd_.size() << std::endl;
    }

    if (optimized_input_.size() >= 12 + joint_dim) {
        const auto joint_vel = optimized_input_.segment(12, joint_dim);
        nmpc_joint_velocity_cmd_ = joint_vel;
        std::cout << "[CtrlComponent] candidate joint_vel[0]: " << joint_vel[0] << std::endl;
        std::cout << "[CtrlComponent] candidate joint_vel dim: " << joint_vel.size() << std::endl;
        std::cout << "[CtrlComponent] cached nmpc_joint_velocity_cmd dim: "
                  << nmpc_joint_velocity_cmd_.size() << std::endl;
    }

    syncMpcRuntimeFromLatestPolicy();
    return true;
}


// Refresh the measured state used by MPC observation from estimator + joint interfaces.
void CtrlComponent::updateMeasuredRbdStateFromEstimator() {
    if (!isMpcRuntimeReady()) {
        std::cerr << "[CtrlComponent] updateMeasuredRbdStateFromEstimator(): MPC runtime not ready." << std::endl;
        return;
    }

    if (!estimator_) {
        std::cerr << "[CtrlComponent] updateMeasuredRbdStateFromEstimator(): estimator is not ready." << std::endl;
        return;
    }

    if (!rbd_conversions_) {
        std::cerr << "[CtrlComponent] updateMeasuredRbdStateFromEstimator(): rbd_conversions_ is not ready." << std::endl;
        return;
    }

    if (!ctrl_interfaces_ptr_) {
        std::cerr << "[CtrlComponent] updateMeasuredRbdStateFromEstimator(): ctrl_interfaces_ptr_ is not ready." << std::endl;
        return;
    }
    //依次检查：
    // runtime ready
    // estimator_ ready
    // rbd_conversions_ ready
    // ctrl_interfaces_ptr_ ready


    const auto& model_info = legged_interface_->getCentroidalModelInfo();
    Eigen::VectorXd rbd_state = Eigen::VectorXd::Zero(2 * model_info.generalizedCoordinatesNum);

    if (ctrl_interfaces_ptr_->joint_position_state_interface_.size() < model_info.actuatedDofNum ||
        ctrl_interfaces_ptr_->joint_velocity_state_interface_.size() < model_info.actuatedDofNum) {
        std::cerr << "[CtrlComponent] updateMeasuredRbdStateFromEstimator(): joint state interfaces size is insufficient." << std::endl;
        return;
    }


    const Vec3 base_position = estimator_->getPosition();
    const RotMat rotation = estimator_->getRotation();
    const Vec3 base_rpy = rotMatToRPY(rotation);  // [roll, pitch, yaw]
    Vec3 base_zyx;
    base_zyx << base_rpy(2), base_rpy(1), base_rpy(0);  // [yaw, pitch, roll]

    const Vec3 base_omega_world = estimator_->getGyroGlobal();
    const Vec3 base_linear_velocity = estimator_->getVelocity();

    rbd_state.segment<3>(0) = base_zyx;
    rbd_state.segment<3>(3) = base_position;

    // First keep joint states from the current MPC state/input as a placeholder.
    for (size_t i = 0; i < model_info.actuatedDofNum; ++i) {
        rbd_state(6 + i) =
            ctrl_interfaces_ptr_->joint_position_state_interface_[i].get().get_value();
    }
    //joint position直接从ros2_control state interface 读取


    //填充base角速度和线速度
    rbd_state.segment<3>(model_info.generalizedCoordinatesNum) = base_omega_world;
    rbd_state.segment<3>(model_info.generalizedCoordinatesNum + 3) = base_linear_velocity;


    //和postion 一样，是从interface读取joint velocity
    for (size_t i = 0; i < model_info.actuatedDofNum; ++i) {
        rbd_state(model_info.generalizedCoordinatesNum + 6 + i) =
            ctrl_interfaces_ptr_->joint_velocity_state_interface_[i].get().get_value();
    }

    static int estimator_log_counter = 0;
    estimator_log_counter++;

    if (estimator_log_counter % 100 == 0) {
        std::cout << "[CtrlComponent] rbd_state joint_pos[0]: " << rbd_state(6) << std::endl;
        std::cout << "[CtrlComponent] rbd_state joint_vel[0]: "
                << rbd_state(model_info.generalizedCoordinatesNum + 6) << std::endl;
        std::cout << "[CtrlComponent] interface joint_pos size: "
                << ctrl_interfaces_ptr_->joint_position_state_interface_.size()
                << ", joint_vel size: "
                << ctrl_interfaces_ptr_->joint_velocity_state_interface_.size()
                << std::endl;
    }




    measured_rbd_state_ = rbd_conversions_->computeCentroidalStateFromRbdModel(rbd_state);
    observation_.state = measured_rbd_state_;

    // static int update_counter = 0;
    // update_counter++;
    // if (update_counter % 20 == 0) {
    //     std::cout << "[CtrlComponent] measured_rbd_state_ updated from estimator." << std::endl;
    // }

    if (estimator_log_counter % 100 == 0) {
        std::cout << "[CtrlComponent] measured_rbd_state_ updated from estimator." << std::endl;
        std::cout << "[CtrlComponent] measured_rbd_state_ dim: "
                << measured_rbd_state_.size() << std::endl;
        std::cout << "[CtrlComponent] measured_rbd_state_[0]: "
            << measured_rbd_state_[0] << std::endl;
    }



}

void CtrlComponent::advanceMpcClock(double dt) {
    if (!isMpcRuntimeReady()) {
        return;
    }

    if (dt <= 0.0) {
        return;
    }

    mpc_time_ += dt;
    observation_.time = mpc_time_;
}

void CtrlComponent::syncMpcRuntimeFromLatestPolicy() {
    if (!hasLatestPolicy()) {
        return;
    }

    if (optimized_state_.size() == mpc_state_.size()) {
        mpc_state_ = optimized_state_;
    }

    if (optimized_input_.size() == mpc_input_.size()) {
        mpc_input_ = optimized_input_;
    }

    mpc_mode_ = planned_mode_;

    observation_.time = mpc_time_;
    observation_.state = measured_rbd_state_;
    observation_.input = mpc_input_;
    observation_.mode = mpc_mode_;
}
