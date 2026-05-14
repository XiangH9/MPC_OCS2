
#ifndef STATENMPC_H
#define STATENMPC_H

#include "controller_common/FSM/FSMState.h"
#include <unitree_guide_controller/common/mathTypes.h>
#include <array>
#include <unitree_guide_controller/gait/UnifiedGaitScheduler.h>


struct CtrlComponent;

class StateNMPC final : public FSMState {
public:

    enum class LegRole {
        STANCE = 0,
        SWING = 1
    };

    explicit StateNMPC(CtrlInterfaces& ctrl_interfaces, CtrlComponent& ctrl_component);

    void enter() override;

    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:

    std::array<LegRole, 4> getLegRoles(const VecInt4& contact) const;
    const char* legRoleToString(LegRole role) const;

    void updateSupportTarget(double vx_body, double vy_body, double yaw_rate_cmd, double control_dt);
    void applySupportTorque(bool force_full_contact);
    void applySwingLegControl(const std::array<LegRole, 4>& leg_roles);
    void applyStanceLegNmpcHoldCommand(const std::array<LegRole, 4>& leg_roles);

    CtrlComponent& ctrl_component_;
    UnifiedGaitScheduler unified_gait_scheduler_;

    Vec3 support_body_pos_target_{Vec3::Zero()};
    Vec3 support_body_vel_target_{Vec3::Zero()};
    Vec3 support_body_rpy_target_{Vec3::Zero()};   // [roll, pitch, yaw]
    Vec3 support_body_rpy_rate_target_{Vec3::Zero()};
    bool support_target_initialized_{false};
};

#endif // STATENMPC_H
