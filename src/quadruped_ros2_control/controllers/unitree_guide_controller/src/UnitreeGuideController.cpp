//
// Created by tlab-uav on 24-9-6.
//

#include "unitree_guide_controller/UnitreeGuideController.h"

#include <unitree_guide_controller/gait/WaveGenerator.h>
#include "unitree_guide_controller/robot/QuadrupedRobot.h"

namespace unitree_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration UnitreeGuideController::command_interface_configuration() const// 想申请哪些 command interfaces
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};//明确点名每一个接口

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }// 本质上是在拼字符串列表，返回给ros2_control   “控制器启动前，先向系统声明：我需要这些关节的 position/velocity/effort/kp/kd 命令口。”

    controller_interface::InterfaceConfiguration UnitreeGuideController::state_interface_configuration() const  // 声明需要读取哪些 state_interfaces
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }//把所有关节的状态接口加入配置，比如每个关节的 position/velocity/effort

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }//第 50 行到第 53 行：再额外把 IMU 的状态接口加入配置。

        return conf;
    }


    //这是整个控制器最核心的周期函数。每次控制周期，controller manager 都会调用它一次
    //1. time 的意义
    // time 表示“当前这次 update 所对应的时刻”。

    // 通常它可以用来做这些事：

    // 记录某个状态开始的绝对时间
    // 计算某个动作已经持续了多久
    // 做基于时间戳的轨迹跟踪
    // 打日志、对齐传感器时间

    //period 的意义
    // period 表示“距离上一次 update 过去了多长时间”，也就是控制周期时长。

    // 它通常比 time 更直接用于控制计算，比如：

    // 积分：x += v * dt
    // 微分：dx = (x_now - x_last) / dt
    // 轨迹推进：每周期推进一点目标
    // 控制器离散化
    controller_interface::return_type UnitreeGuideController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        if (ctrl_component_.robot_model_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        ctrl_component_.robot_model_->update();   //最新关节状态同步进模型
        ctrl_component_.wave_generator_->update(); //更新“当前哪条腿支撑，哪条腿摆动”
        ctrl_component_.estimator_->update(); //根据 IMU 和足端信息估计机身姿态、位置、速度等。

        if (mode_ == FSMMode::NORMAL)// 开始状态机调度 当前 FSM 模式是 NORMAL，说明当前没有在切状态，而是在正常执行某个状态
        {
            current_state_->run(time, period); //真正执行当前状态的控制逻辑
            next_state_name_ = current_state_->checkChange();  //根据当前输入，下一个切换到哪个状态
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }//或者直接执行状态切换

        return controller_interface::return_type::OK;
    }

    // 控制器初始阶段调用，主要作   参数声明和基础对象准备
    controller_interface::CallbackReturn UnitreeGuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);// 读取基本参数  这里应该是ros2的参数通信   

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);//声明并读取与 IMU、base、feet、命令前缀相关的参数：

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            //声明并读取姿态相关参数：

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);
            //读取 update_rate 参数到 ctrl_interfaces_.frequency_    打印控制器控制频率


            ctrl_component_.setupNmpcConfig(get_node());     //初始化NMPC的配置，初始化  ctrl_component里的某部分配置
            ctrl_component_.joint_names_ = joint_names_;
            ctrl_component_.feet_names_ = feet_names_; //把关节名和足端名塞给 ctrl_component_

            ctrl_component_.bindCtrlInterfaces(ctrl_interfaces_); //ctrl_component_ 以后也能访问这组共享接口数据

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_); //创建估计器 Estimator，并交给 ctrl_component_ 保存。
            //UnitreeGuideController 不是自己保存所有算法对象，而是把它们挂到 ctrl_component_ 里统一管理。

        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // 订阅建立、运行依赖对象建立 
    controller_interface::CallbackReturn UnitreeGuideController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        ctrl_component_.setupLeggedInterface();  //因为接入了nmpc,做了额外的控制后端初始化
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });


            //第 163 行到第 170 行：订阅 /robot_description
// 这个 topic 用的是 transient_local()，说明它希望即使晚订阅，也能拿到最近一次发布的 URDF 字符串。
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
            });//收到的URDF字符串创建 QuadrupedRobot,基于机器人模型再做BalanceCtrl

        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(0.45, 0.5, Vec4(0, 0.5, 0.5, 0));
        //创建 WaveGenerator , 传入的参数 0.45, 0.5, Vec4(0, 0.5, 0.5, 0) 可以先理解成步态周期、支撑比、四足相位偏置。

        return CallbackReturn::SUCCESS;
    }

    //  on_configure意义：建立输入订阅
    // 建立机器人模型相关对象
    // 建立步态生成器


    //真正把ros2_control 接口接上
    controller_interface::CallbackReturn
    UnitreeGuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();  //清空

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }//按照接口名字  绑定command interfaces 后面的状态就可以直接写：
// ctrl_interfaces_.joint_position_command_interface_[i]
// 而不必每次自己去解析 ros2_control 接口名。

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);
        state_list_.nmpc = std::make_shared<StateNMPC>(ctrl_interfaces_, ctrl_component_);
        //ctrl_interfaces_ 提供“读写硬件与输入”
        //ctrl_component_ 提供“算法资源与模型能力” 


        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn UnitreeGuideController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> UnitreeGuideController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::NMPC:
            return state_list_.nmpc;
        case FSMStateName::SWINGTEST:
            return state_list_.swingTest;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_guide_controller::UnitreeGuideController, controller_interface::ControllerInterface);
