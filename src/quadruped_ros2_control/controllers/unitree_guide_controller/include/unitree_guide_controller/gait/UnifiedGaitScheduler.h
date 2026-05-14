#ifndef UNIFIEDGAITSCHEDULER_H
#define UNIFIEDGAITSCHEDULER_H

#include <memory>
#include <cstddef>
#include <vector>

#include <controller_common/common/enumClass.h>
#include <unitree_guide_controller/common/mathTypes.h>
#include <unitree_guide_controller/gait/GaitGenerator.h>
#include <ocs2_core/reference/ModeSchedule.h>

class WaveGenerator;

struct ExecutionGaitState {
    WaveStatus requested_status{WaveStatus::STANCE_ALL};
    WaveStatus active_status{WaveStatus::STANCE_ALL};
    VecInt4 contact{VecInt4::Ones()};
    Vec4 phase{Vec4::Constant(0.5)};
    bool zero_command_override{true};
};

struct PlannerGaitState {
    size_t current_mode{15};
    VecInt4 current_contact{VecInt4::Ones()};
    bool is_full_stance{true};
};

struct GaitPhaseSegment {
    double start_time{0.0};
    double end_time{0.0};
    size_t mode{15};
    VecInt4 contact{VecInt4::Ones()};
};

class UnifiedGaitScheduler {
public:
    explicit UnifiedGaitScheduler(CtrlComponent& ctrl_component);

    void update(bool zero_command);

    const ExecutionGaitState& getExecutionState() const;
    const PlannerGaitState& getPlannerState() const;
    const VecInt4& getExecutionContact() const;
    const Vec4& getExecutionPhase() const;
    const VecInt4& getPlannerContact() const;
    bool isExecutionFullStance() const;
    bool isPlannerFullStance() const;

    void rebuildFutureGaitSegments(double start_time, double horizon_duration);
    const std::vector<GaitPhaseSegment>& getFutureGaitSegments() const;
    ocs2::ModeSchedule buildModeSchedule(double base_time) const;

    void setGaitCommand(const Vec2& vxy_goal, double yaw_rate_goal, double gait_height);
    const Vec34& getFootPosGoal() const;
    const Vec34& getFootVelGoal() const;

    void restart();

private:
    std::shared_ptr<WaveGenerator>& wave_generator_;
    ExecutionGaitState execution_state_;
    PlannerGaitState planner_state_;
    std::vector<GaitPhaseSegment> future_gait_segments_;

    Vec34 foot_pos_goal_{Vec34::Zero()};
    Vec34 foot_vel_goal_{Vec34::Zero()};
    double gait_height_{0.05};
    Vec2 vxy_goal_{Vec2::Zero()};
    double yaw_rate_goal_{0.0};

    GaitGenerator gait_generator_;

};

#endif // UNIFIEDGAITSCHEDULER_H
