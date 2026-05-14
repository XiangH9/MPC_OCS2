#ifndef UNITREE_GUIDE_CONTROLLER_NMPC_LEGGEDINTERFACE_H
#define UNITREE_GUIDE_CONTROLLER_NMPC_LEGGEDINTERFACE_H

#include <string>
#include <vector>
#include <memory>

#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_oc/rollout/RolloutSettings.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <Eigen/Core>


#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>
#include <ocs2_legged_robot/initialization/LeggedRobotInitializer.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>


//  OptimalProblem
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// Optimal Cost
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/Types.h>

//OCS2 运行前的大部分装配工作


//保存配置是否存在的检查结果
namespace unitree_guide_controller::nmpc {

    struct BasicNmpcConfig {
        bool verbose{false};  //task/reference是否启用了verbose模式

        bool has_model_settings{false};  //是否存在model_settings
        bool has_mpc_settings{false};  //是否存在mpc
        bool has_sqp_settings{false};   //是否存在sqp
        bool has_rollout_settings{false};   //是否存在rollout
        bool has_swing_trajectory_config{false};  //是否存在swing_trajectory_config
        bool has_initial_state{false};  //是否存在initial_state

        bool has_initial_mode_schedule{false};
        bool has_default_mode_sequence_template{false};  //是否存在defaultModeSequence
    };

class LeggedInterface {
public:
    // read config files


    const BasicNmpcConfig& getBasicConfig() const;//返回配置完整性检查结果

    // prepare the NMPC interface

    LeggedInterface(std::string task_file,
                    std::string urdf_file,
                    std::string reference_file);//步态是从reference_file中加载，而不是单独从gait_file中加载

    void setupJointNames(const std::vector<std::string>& joint_names,
                         const std::vector<std::string>& foot_names); //传入控制器层的joint和foot

    bool initialize();  //最核心的入口函数

    bool isInitialized() const;

    const std::string& getTaskFile() const;
    const std::string& getUrdfFile() const;
    const std::string& getReferenceFile() const;

    const std::vector<std::string>& getJointNames() const;
    const std::vector<std::string>& getFootNames() const;

    const ocs2::legged_robot::ModelSettings& getModelSettings() const;
    const ocs2::mpc::Settings& getMpcSettings() const;
    const ocs2::sqp::Settings& getSqpSettings() const;
    const ocs2::rollout::Settings& getRolloutSettings() const;  //leggedInterface会在内部持有解析后的settings

//模型层核心
    const ocs2::PinocchioInterface& getPinocchioInterface() const; //返回机器人模型接口
    const ocs2::CentroidalModelInfo& getCentroidalModelInfo() const;  //centroidal model的信息结构
    const Eigen::VectorXd& getInitialState() const;  


    bool hasReferenceManager() const;
    bool hasInitializer() const;

    std::shared_ptr<ocs2::legged_robot::SwitchedModelReferenceManager> getReferenceManagerPtr() const;  //reference manager指针，核心接口，挂载TargetTrajectories
    const ocs2::Initializer& getInitializer() const;


    //OptimalProblem
    bool hasOptimalControlProblem() const;
    bool hasRollout() const;

    const ocs2::OptimalControlProblem& getOptimalControlProblem() const;
    const ocs2::RolloutBase& getRollout() const;





private:
    std::string task_file_;
    std::string urdf_file_;
    std::string reference_file_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> foot_names_;

    bool initialized_{false};

    BasicNmpcConfig basic_config_;
    bool loadBasicConfig();

    bool verbose_{false};
    ocs2::legged_robot::ModelSettings model_settings_;
    ocs2::mpc::Settings mpc_settings_;
    ocs2::sqp::Settings sqp_settings_;
    ocs2::rollout::Settings rollout_settings_;

    // bool hasChild(const boost::property_tree::ptree& tree, const std::string& key) const;

    //模型内部对象
    std::unique_ptr<ocs2::PinocchioInterface> pinocchio_interface_ptr_;
    ocs2::CentroidalModelInfo centroidal_model_info_;
    Eigen::VectorXd initial_state_;


    void setupReferenceManagerAndInitializer();
    std::shared_ptr<ocs2::legged_robot::GaitSchedule> loadGaitSchedule(bool verbose) const;

    std::shared_ptr<ocs2::legged_robot::SwitchedModelReferenceManager> reference_manager_ptr_; //TargetTrajectories ModeSchedule gait Schedule
    std::unique_ptr<ocs2::Initializer> initializer_ptr_;  //

    //OptimalProblem
    void setupOptimalControlProblemSkeleton();  //OptimalProblem骨架，dynamics,cost,rollout

    // Optimal Cost
    ocs2::matrix_t initializeInputCostWeight(const std::string& task_file, const ocs2::CentroidalModelInfo& info);
    std::unique_ptr<ocs2::StateInputCost> getBaseTrackingCost(
        const std::string& task_file,
        const ocs2::CentroidalModelInfo& info,
        bool verbose);



    //Optimal Problem
    std::unique_ptr<ocs2::OptimalControlProblem> optimal_control_problem_ptr_;
    std::unique_ptr<ocs2::RolloutBase> rollout_ptr_;







};

}  // namespace unitree_guide_controller::nmpc

#endif
