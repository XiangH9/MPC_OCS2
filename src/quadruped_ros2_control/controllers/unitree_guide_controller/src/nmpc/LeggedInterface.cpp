#include "unitree_guide_controller/nmpc/LeggedInterface.h"

#include <filesystem>
#include <iostream>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>


//OptimalProblem
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

//OptimalCost
#include <ocs2_legged_robot/cost/LeggedRobotQuadraticTrackingCost.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>






namespace unitree_guide_controller::nmpc {

LeggedInterface::LeggedInterface(std::string task_file,
                                 std::string urdf_file,
                                 std::string reference_file)
    : task_file_(std::move(task_file)),
      urdf_file_(std::move(urdf_file)),
      reference_file_(std::move(reference_file)) {
}//保存相应的参考文件，不做任何初始化动作，真正的初始化动作在 initialize()里

namespace {
bool hasChild(const boost::property_tree::ptree& tree, const std::string& key) {
    return tree.find(key) != tree.not_found();
}
}

//里面检查了Task 和 Reference配置的完整性
bool LeggedInterface::loadBasicConfig() {
    using boost::property_tree::ptree;

    ptree task_tree;
    ptree reference_tree;

    try {
        boost::property_tree::read_info(task_file_, task_tree);
    } catch (const std::exception& e) {
        std::cerr << "[LeggedInterface] failed to read task.info: " << e.what() << std::endl;
        return false;
    }

    try {
        boost::property_tree::read_info(reference_file_, reference_tree);
    } catch (const std::exception& e) {
        std::cerr << "[LeggedInterface] failed to read reference.info: " << e.what() << std::endl;
        return false;
    }

    basic_config_.verbose = task_tree.get("legged_robot_interface.verbose", false);

    basic_config_.has_model_settings = hasChild(task_tree, "model_settings");
    basic_config_.has_mpc_settings = hasChild(task_tree, "mpc");
    basic_config_.has_sqp_settings = hasChild(task_tree, "sqp");
    basic_config_.has_rollout_settings = hasChild(task_tree, "rollout");
    basic_config_.has_swing_trajectory_config = hasChild(task_tree, "swing_trajectory_config");
    basic_config_.has_initial_state = hasChild(task_tree, "initialState");

    basic_config_.has_initial_mode_schedule = hasChild(reference_tree, "initialModeSchedule");
    basic_config_.has_default_mode_sequence_template = hasChild(reference_tree, "defaultModeSequenceTemplate");

    std::cout << "[LeggedInterface] verbose: " << (basic_config_.verbose ? "true" : "false") << std::endl;
    std::cout << "[LeggedInterface] task.info sections:"
              << " model_settings=" << basic_config_.has_model_settings
              << " mpc=" << basic_config_.has_mpc_settings
              << " sqp=" << basic_config_.has_sqp_settings
              << " rollout=" << basic_config_.has_rollout_settings
              << " swing_trajectory_config=" << basic_config_.has_swing_trajectory_config
              << " initialState=" << basic_config_.has_initial_state
              << std::endl;

    std::cout << "[LeggedInterface] reference.info sections:"
              << " initialModeSchedule=" << basic_config_.has_initial_mode_schedule
              << " defaultModeSequenceTemplate=" << basic_config_.has_default_mode_sequence_template
              << std::endl;

    const bool task_sections_ok =
        basic_config_.has_model_settings &&
        basic_config_.has_mpc_settings &&
        basic_config_.has_sqp_settings &&
        basic_config_.has_rollout_settings &&
        basic_config_.has_swing_trajectory_config &&
        basic_config_.has_initial_state;

    const bool reference_sections_ok =
        basic_config_.has_initial_mode_schedule &&
        basic_config_.has_default_mode_sequence_template;

    if (!task_sections_ok) {
        std::cerr << "[LeggedInterface] task.info is missing one or more required sections." << std::endl;
        return false;
    }

    if (!reference_sections_ok) {
        std::cerr << "[LeggedInterface] reference.info is missing one or more required sections." << std::endl;
        return false;
    }

    return true;
}

//从reference中构造gait schedule
// gait schedule的具体形式和步态的结构
std::shared_ptr<ocs2::legged_robot::GaitSchedule> LeggedInterface::loadGaitSchedule(bool verbose) const {
    const auto init_mode_schedule =
        ocs2::legged_robot::loadModeSchedule(reference_file_, "initialModeSchedule", false);
    const auto default_mode_sequence_template =
        ocs2::legged_robot::loadModeSequenceTemplate(reference_file_, "defaultModeSequenceTemplate", false);

    if (verbose) {
        std::cout << "[LeggedInterface] Loaded gait schedule from reference.info" << std::endl;
        std::cout << "[LeggedInterface] initialModeSchedule events: "
                  << init_mode_schedule.eventTimes.size() << std::endl;
        std::cout << "[LeggedInterface] defaultModeSequenceTemplate modes: "
                  << default_mode_sequence_template.modeSequence.size() << std::endl;
    }

    return std::make_shared<ocs2::legged_robot::GaitSchedule>(
        init_mode_schedule,
        default_mode_sequence_template,
        model_settings_.phaseTransitionStanceTime);
}


void LeggedInterface::setupReferenceManagerAndInitializer() {
    auto swing_trajectory_planner =
        std::make_shared<ocs2::legged_robot::SwingTrajectoryPlanner>(
            ocs2::legged_robot::loadSwingTrajectorySettings(task_file_, "swing_trajectory_config", verbose_),
            foot_names_.size());// 创建摆腿轨迹规划器  OCS2里什么都有了，有没有和它交互的更灵活的方式

    reference_manager_ptr_ =
        std::make_shared<ocs2::legged_robot::SwitchedModelReferenceManager>(
            loadGaitSchedule(verbose_),
            std::move(swing_trajectory_planner));//referenceManager 主要  Gait 和  Swing trajectory planner

    //如果已经提前定义好了这些参数，为什么还要运用下层的执行器去重新执行步态的控制逻辑

    constexpr bool extend_normalized_momentum = true;
    initializer_ptr_ = std::make_unique<ocs2::legged_robot::LeggedRobotInitializer>(
        centroidal_model_info_,
        *reference_manager_ptr_,
        extend_normalized_momentum);

    std::cout << "[LeggedInterface] ReferenceManager created successfully." << std::endl;
    std::cout << "[LeggedInterface] Initializer created successfully." << std::endl;
}

// OptimalProblem
void LeggedInterface::setupOptimalControlProblemSkeleton() {
    optimal_control_problem_ptr_ = std::make_unique<ocs2::OptimalControlProblem>();

    //加入动力学
    optimal_control_problem_ptr_->dynamicsPtr =
        std::make_unique<ocs2::legged_robot::LeggedRobotDynamicsAD>(
            *pinocchio_interface_ptr_,
            centroidal_model_info_,
            "dynamics",
            model_settings_);//当前Dynamics用的是LeggedRobotDynamicAD

    //加入tracking cost
    optimal_control_problem_ptr_->costPtr->add(
        "baseTrackingCost",
        getBaseTrackingCost(task_file_, centroidal_model_info_, verbose_));

    rollout_ptr_ = std::make_unique<ocs2::TimeTriggeredRollout>(
        *optimal_control_problem_ptr_->dynamicsPtr,
        rollout_settings_);

    std::cout << "[LeggedInterface] baseTrackingCost added successfully." << std::endl;
    std::cout << "[LeggedInterface] OptimalControlProblem skeleton created successfully." << std::endl;
    std::cout << "[LeggedInterface] Rollout created successfully." << std::endl;
}

//

// Optimal Cost
//构造输入权重矩阵
ocs2::matrix_t LeggedInterface::initializeInputCostWeight(
    const std::string& task_file,
    const ocs2::CentroidalModelInfo& info) {
    const size_t total_contact_dim = 3 * info.numThreeDofContacts;

    const auto& model = pinocchio_interface_ptr_->getModel();
    auto& data = pinocchio_interface_ptr_->getData();
    const auto q = ocs2::centroidal_model::getGeneralizedCoordinates(initial_state_, centroidal_model_info_);

    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ocs2::matrix_t base_to_feet_jacobians(total_contact_dim, info.actuatedDofNum);
    for (size_t i = 0; i < info.numThreeDofContacts; ++i) {
        ocs2::matrix_t jacobian = ocs2::matrix_t::Zero(6, info.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(
            model,
            data,
            model.getBodyId(model_settings_.contactNames3DoF[i]),
            pinocchio::LOCAL_WORLD_ALIGNED,
            jacobian);

        base_to_feet_jacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
            jacobian.block(0, 6, 3, info.actuatedDofNum);
    }

    ocs2::matrix_t r_taskspace(total_contact_dim + total_contact_dim, total_contact_dim + total_contact_dim);
    ocs2::loadData::loadEigenMatrix(task_file, "R", r_taskspace);

    ocs2::matrix_t r = ocs2::matrix_t::Zero(info.inputDim, info.inputDim);
    r.topLeftCorner(total_contact_dim, total_contact_dim) =
        r_taskspace.topLeftCorner(total_contact_dim, total_contact_dim);

    r.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
        base_to_feet_jacobians.transpose() *
        r_taskspace.bottomRightCorner(total_contact_dim, total_contact_dim) *
        base_to_feet_jacobians;

    return r;
}

//真正创建tracking_cost
std::unique_ptr<ocs2::StateInputCost> LeggedInterface::getBaseTrackingCost(
    const std::string& task_file,
    const ocs2::CentroidalModelInfo& info,
    bool verbose) {
    ocs2::matrix_t q(info.stateDim, info.stateDim);
    ocs2::loadData::loadEigenMatrix(task_file, "Q", q);

    ocs2::matrix_t r = initializeInputCostWeight(task_file, info);

    if (verbose) {
        std::cerr << "\n #### Base Tracking Cost Coefficients: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "Q:\n" << q << "\n";
        std::cerr << "R:\n" << r << "\n";
        std::cerr << " #### =============================================================================\n";
    }

    return std::make_unique<ocs2::legged_robot::LeggedRobotStateInputQuadraticCost>(
        std::move(q),
        std::move(r),
        info,
        *reference_manager_ptr_);
}//是“参考轨迹如何进入 NMPC 求解”的核心入口。





void LeggedInterface::setupJointNames(const std::vector<std::string>& joint_names,
                                      const std::vector<std::string>& foot_names) {
    joint_names_ = joint_names;
    foot_names_ = foot_names;
}

//核心函数，串起前面所有的装配步骤
bool LeggedInterface::initialize() {
    initialized_ = false;  //初始状态清零

    if (task_file_.empty() || urdf_file_.empty() || reference_file_.empty()) {
        std::cerr << "[LeggedInterface] file path is empty." << std::endl;
        return false;
    }

    if (!std::filesystem::exists(task_file_)) {
        std::cerr << "[LeggedInterface] task file not found: " << task_file_ << std::endl;
        return false;
    }

    if (!std::filesystem::exists(urdf_file_)) {
        std::cerr << "[LeggedInterface] urdf file not found: " << urdf_file_ << std::endl;
        return false;
    }

    if (!std::filesystem::exists(reference_file_)) {
        std::cerr << "[LeggedInterface] reference file not found: " << reference_file_ << std::endl;
        return false;
    }

    if (joint_names_.empty()) {
        std::cerr << "[LeggedInterface] joint names are empty." << std::endl;
        return false;
    }

    if (foot_names_.empty()) {
        std::cerr << "[LeggedInterface] foot names are empty." << std::endl;
        return false;
    }

    if (!loadBasicConfig()) {
        std::cerr << "[LeggedInterface] failed to load basic config." << std::endl;
        return false;
    }
    // 基础合法性检查task_file_ 是否空 urdf_file_ 是否空 reference_file_ 是否空 对应文件是否存在 joint_names_ 是否空 foot_names_ 是否空

    try {
        verbose_ = basic_config_.verbose;

        model_settings_ = ocs2::legged_robot::loadModelSettings(task_file_, "model_settings", verbose_);
        mpc_settings_ = ocs2::mpc::loadSettings(task_file_, "mpc", verbose_);
        sqp_settings_ = ocs2::sqp::loadSettings(task_file_, "sqp", verbose_);
        rollout_settings_ = ocs2::rollout::loadSettings(task_file_, "rollout", verbose_);//读取OCS2 Settings

        model_settings_.jointNames = joint_names_;
        model_settings_.contactNames3DoF = foot_names_;

        std::cout << "[LeggedInterface] model_settings jointNames size: "
                << model_settings_.jointNames.size() << std::endl;
        std::cout << "[LeggedInterface] model_settings contactNames3DoF size: "
                << model_settings_.contactNames3DoF.size() << std::endl;
        std::cout << "[LeggedInterface] OCS2 settings loaded successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[LeggedInterface] failed to load OCS2 settings: "
                << e.what() << std::endl;
        return false;
    }
    //创建 Pinocchio 模型和 centroidal model info
    try {
        pinocchio_interface_ptr_ = std::make_unique<ocs2::PinocchioInterface>(
            ocs2::centroidal_model::createPinocchioInterface(urdf_file_, model_settings_.jointNames));

        centroidal_model_info_ = ocs2::centroidal_model::createCentroidalModelInfo(
            *pinocchio_interface_ptr_,
            ocs2::centroidal_model::loadCentroidalType(task_file_),
            ocs2::centroidal_model::loadDefaultJointState(
                pinocchio_interface_ptr_->getModel().nq - 6,
                reference_file_,
                "defaultJointState"),
            model_settings_.contactNames3DoF,
            model_settings_.contactNames6DoF);

        initial_state_.setZero(centroidal_model_info_.stateDim);
        ocs2::loadData::loadEigenMatrix(task_file_, "initialState", initial_state_);//读取初始状态,这里利用的是OCS2库在读取初始状态

        std::cout << "[LeggedInterface] PinocchioInterface created successfully." << std::endl;
        std::cout << "[LeggedInterface] CentroidalModelInfo stateDim: "
                << centroidal_model_info_.stateDim << std::endl;
        std::cout << "[LeggedInterface] CentroidalModelInfo inputDim: "
                << centroidal_model_info_.inputDim << std::endl;
        std::cout << "[LeggedInterface] initialState size: "
                << initial_state_.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[LeggedInterface] failed to create minimal model objects: "
                << e.what() << std::endl;
        return false;
    }

// 创建 reference manager 和 initializer
    try {
        setupReferenceManagerAndInitializer();  
    } catch (const std::exception& e) {
        std::cerr << "[LeggedInterface] failed to create reference manager or initializer: "
                  << e.what() << std::endl;
        return false;
    }

    //OptimalProblem
    try {
        setupOptimalControlProblemSkeleton();
    } catch (const std::exception& e) {
        std::cerr << "[LeggedInterface] failed to create OptimalControlProblem skeleton: "
                  << e.what() << std::endl;
        return false;
    }
    //


    initialized_ = true;
    std::cout << "[LeggedInterface] initialized successfully." << std::endl;
    return true; //初始化完成
}

bool LeggedInterface::isInitialized() const {
    return initialized_;
}

const std::string& LeggedInterface::getTaskFile() const {
    return task_file_;
}

const std::string& LeggedInterface::getUrdfFile() const {
    return urdf_file_;
}

const std::string& LeggedInterface::getReferenceFile() const {
    return reference_file_;
}

const std::vector<std::string>& LeggedInterface::getJointNames() const {
    return joint_names_;
}

const std::vector<std::string>& LeggedInterface::getFootNames() const {
    return foot_names_;
}


const BasicNmpcConfig& LeggedInterface::getBasicConfig() const {
    return basic_config_;
}

const ocs2::legged_robot::ModelSettings& LeggedInterface::getModelSettings() const {
    return model_settings_;
}

const ocs2::mpc::Settings& LeggedInterface::getMpcSettings() const {
    return mpc_settings_;
}

const ocs2::sqp::Settings& LeggedInterface::getSqpSettings() const {
    return sqp_settings_;
}

const ocs2::rollout::Settings& LeggedInterface::getRolloutSettings() const {
    return rollout_settings_;
}

const ocs2::PinocchioInterface& LeggedInterface::getPinocchioInterface() const {
    return *pinocchio_interface_ptr_;
}

const ocs2::CentroidalModelInfo& LeggedInterface::getCentroidalModelInfo() const {
    return centroidal_model_info_;
}

const Eigen::VectorXd& LeggedInterface::getInitialState() const {
    return initial_state_;
}


bool LeggedInterface::hasReferenceManager() const {
    return static_cast<bool>(reference_manager_ptr_);
}

bool LeggedInterface::hasInitializer() const {
    return static_cast<bool>(initializer_ptr_);
}

std::shared_ptr<ocs2::legged_robot::SwitchedModelReferenceManager>
LeggedInterface::getReferenceManagerPtr() const {
    return reference_manager_ptr_;
}

const ocs2::Initializer& LeggedInterface::getInitializer() const {
    return *initializer_ptr_;
}

//OptimalProblem
bool LeggedInterface::hasOptimalControlProblem() const {
    return static_cast<bool>(optimal_control_problem_ptr_);
}

bool LeggedInterface::hasRollout() const {
    return static_cast<bool>(rollout_ptr_);
}

const ocs2::OptimalControlProblem& LeggedInterface::getOptimalControlProblem() const {
    return *optimal_control_problem_ptr_;
}

const ocs2::RolloutBase& LeggedInterface::getRollout() const {
    return *rollout_ptr_;
}
//





}  // namespace unitree_guide_controller::nmpc
