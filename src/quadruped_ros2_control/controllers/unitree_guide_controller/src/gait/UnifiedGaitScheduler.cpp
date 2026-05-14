#include "unitree_guide_controller/gait/UnifiedGaitScheduler.h"

#include "unitree_guide_controller/gait/WaveGenerator.h"
#include <vector>
#include <algorithm>
#include <limits>

#include "unitree_guide_controller/control/CtrlComponent.h"

namespace {

constexpr size_t kModeRfLh = 6;
constexpr size_t kModeLfRh = 9;
constexpr size_t kModeStance = 15;


size_t contactToMode(const VecInt4& contact) {
    if (contact(0) == 0 && contact(1) == 1 && contact(2) == 1 && contact(3) == 0) {
        return kModeRfLh;
    }
    if (contact(0) == 1 && contact(1) == 0 && contact(2) == 0 && contact(3) == 1) {
        return kModeLfRh;
    }
    return kModeStance;
}

bool isFullStanceContact(const VecInt4& contact) {
    return contact(0) == 1 &&
           contact(1) == 1 &&
           contact(2) == 1 &&
           contact(3) == 1;
}

bool isRfLhContact(const VecInt4& contact) {
    return contact(0) == 0 &&
           contact(1) == 1 &&
           contact(2) == 1 &&
           contact(3) == 0;
}

bool isLfRhContact(const VecInt4& contact) {
    return contact(0) == 1 &&
           contact(1) == 0 &&
           contact(2) == 0 &&
           contact(3) == 1;
}

VecInt4 makeRfLhContact() {
    VecInt4 contact;
    contact << 0, 1, 1, 0;
    return contact;
}

VecInt4 makeLfRhContact() {
    VecInt4 contact;
    contact << 1, 0, 0, 1;
    return contact;
}

double getLegRemainingPhaseTime(int contact_flag,
                                double phase,
                                double t_stance,
                                double t_swing) {
    const double clamped_phase = std::clamp(phase, 0.0, 1.0);
    const double duration = (contact_flag == 1) ? t_stance : t_swing;
    return std::max(0.0, (1.0 - clamped_phase) * duration);
}

double getCurrentSegmentRemainingTime(const VecInt4& contact,
                                      const Vec4& phase,
                                      double t_stance,
                                      double t_swing) {
    double min_remaining = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 4; ++i) {
        const double remaining =
            getLegRemainingPhaseTime(contact(i), phase(i), t_stance, t_swing);
        min_remaining = std::min(min_remaining, remaining);
    }

    if (!std::isfinite(min_remaining)) {
        return 1e-3;
    }

    return std::max(min_remaining, 1e-3);
}

VecInt4 estimateNextContact(const VecInt4& current_contact) {
    if (isFullStanceContact(current_contact)) {
        // Align the first leaving-stance phase with the standing-trot template
        // in reference.info: LF_RH -> STANCE -> RF_LH -> STANCE.
        return makeLfRhContact();
    }

    if (isRfLhContact(current_contact)) {
        return makeLfRhContact();
    }

    if (isLfRhContact(current_contact)) {
        return makeRfLhContact();
    }

    return VecInt4::Ones();
}


}  // namespace

UnifiedGaitScheduler::UnifiedGaitScheduler(CtrlComponent& ctrl_component)
    : wave_generator_(ctrl_component.wave_generator_),
      gait_generator_(ctrl_component) {
}

void UnifiedGaitScheduler::update(bool zero_command) {

    execution_state_.requested_status =
        zero_command ? WaveStatus::STANCE_ALL : WaveStatus::WAVE_ALL;
    execution_state_.zero_command_override = zero_command;

    if (!wave_generator_) {
        execution_state_.active_status = execution_state_.requested_status;
        execution_state_.contact.setOnes();
        execution_state_.phase << 0.5, 0.5, 0.5, 0.5;
        foot_pos_goal_.setZero();
        foot_vel_goal_.setZero();
        rebuildFutureGaitSegments(0.0, 1.0);
        return;
    }

    // Drive the execution-side gait source.
    wave_generator_->status_ = execution_state_.requested_status;
    wave_generator_->update();

    execution_state_.active_status = wave_generator_->status_;
    execution_state_.contact = wave_generator_->contact_;
    execution_state_.phase = wave_generator_->phase_;

    planner_state_.current_contact = execution_state_.contact;
    planner_state_.current_mode = contactToMode(execution_state_.contact);
    planner_state_.is_full_stance = isFullStanceContact(execution_state_.contact);

    if (!execution_state_.zero_command_override) {
        gait_generator_.setGait(vxy_goal_, yaw_rate_goal_, gait_height_);
        gait_generator_.generate(foot_pos_goal_, foot_vel_goal_);
    } else {
        foot_pos_goal_.setZero();
        foot_vel_goal_.setZero();
    }

    rebuildFutureGaitSegments(0.0, 2.0 * (wave_generator_ ? wave_generator_->get_t() : 1.0));
}

const ExecutionGaitState& UnifiedGaitScheduler::getExecutionState() const {
    return execution_state_;
}

const PlannerGaitState& UnifiedGaitScheduler::getPlannerState() const {
    return planner_state_;
}


void UnifiedGaitScheduler::rebuildFutureGaitSegments(double start_time, double horizon_duration) {
    future_gait_segments_.clear();

    if (execution_state_.zero_command_override || !wave_generator_) {
        GaitPhaseSegment segment;
        segment.start_time = start_time;
        segment.end_time = start_time + horizon_duration;
        segment.mode = kModeStance;
        segment.contact = VecInt4::Ones();
        future_gait_segments_.push_back(segment);
        return;
    }

    const size_t current_mode = contactToMode(execution_state_.contact);
    const double phase_stance = 0.05;
    const double phase_swing = 0.25;

    const double current_segment_remaining_time =
        getCurrentSegmentRemainingTime(
            execution_state_.contact,
            execution_state_.phase,
            phase_stance,
            phase_swing);

    const double horizon_end = start_time + horizon_duration;
    double t = start_time;

    auto append_segment = [&](const VecInt4& contact, double duration, size_t mode) -> bool {
        if (t >= horizon_end) {
            return false;
        }

        GaitPhaseSegment segment;
        segment.start_time = t;
        segment.end_time = std::min(t + duration, horizon_end);
        segment.contact = contact;
        segment.mode = mode;
        future_gait_segments_.push_back(segment);
        t = segment.end_time;
        return t < horizon_end;
    };

    if (current_mode == kModeStance) {
        if (!append_segment(VecInt4::Ones(), current_segment_remaining_time, kModeStance)) return;
        if (!append_segment(makeLfRhContact(), phase_swing, kModeLfRh)) return;
        if (!append_segment(VecInt4::Ones(), phase_stance, kModeStance)) return;
        if (!append_segment(makeRfLhContact(), phase_swing, kModeRfLh)) return;
        append_segment(VecInt4::Ones(), phase_stance, kModeStance);
        return;
    }

    if (current_mode == kModeLfRh) {
        if (!append_segment(makeLfRhContact(), current_segment_remaining_time, kModeLfRh)) return;
        if (!append_segment(VecInt4::Ones(), phase_stance, kModeStance)) return;
        if (!append_segment(makeRfLhContact(), phase_swing, kModeRfLh)) return;
        append_segment(VecInt4::Ones(), phase_stance, kModeStance);
        return;
    }

    if (current_mode == kModeRfLh) {
        if (!append_segment(makeRfLhContact(), current_segment_remaining_time, kModeRfLh)) return;
        if (!append_segment(VecInt4::Ones(), phase_stance, kModeStance)) return;
        if (!append_segment(makeLfRhContact(), phase_swing, kModeLfRh)) return;
        append_segment(VecInt4::Ones(), phase_stance, kModeStance);
        return;
    }

    if (!append_segment(VecInt4::Ones(), current_segment_remaining_time, kModeStance)) return;
    if (!append_segment(makeLfRhContact(), phase_swing, kModeLfRh)) return;
    if (!append_segment(VecInt4::Ones(), phase_stance, kModeStance)) return;
    if (!append_segment(makeRfLhContact(), phase_swing, kModeRfLh)) return;
    append_segment(VecInt4::Ones(), phase_stance, kModeStance);
}



const std::vector<GaitPhaseSegment>& UnifiedGaitScheduler::getFutureGaitSegments() const {
    return future_gait_segments_;
}

ocs2::ModeSchedule UnifiedGaitScheduler::buildModeSchedule(double base_time) const {
    if (future_gait_segments_.empty()) {
        return ocs2::ModeSchedule(
            std::vector<ocs2::scalar_t>{},
            std::vector<size_t>{kModeStance});
    }

    std::vector<ocs2::scalar_t> event_times;
    std::vector<size_t> mode_sequence;
    mode_sequence.reserve(future_gait_segments_.size());

    for (size_t i = 0; i < future_gait_segments_.size(); ++i) {
        const auto& seg = future_gait_segments_[i];
        if (mode_sequence.empty() || mode_sequence.back() != seg.mode) {
            mode_sequence.push_back(seg.mode);
        }

        if (i + 1 < future_gait_segments_.size()) {
            const size_t next_mode = future_gait_segments_[i + 1].mode;
            if (next_mode != seg.mode) {
                event_times.push_back(base_time + seg.end_time);
            }
        }
    }

    return ocs2::ModeSchedule(std::move(event_times), std::move(mode_sequence));
}


const VecInt4& UnifiedGaitScheduler::getExecutionContact() const {
    return execution_state_.contact;
}

const Vec4& UnifiedGaitScheduler::getExecutionPhase() const {
    return execution_state_.phase;
}

const VecInt4& UnifiedGaitScheduler::getPlannerContact() const {
    return planner_state_.current_contact;
}

bool UnifiedGaitScheduler::isExecutionFullStance() const {
    return execution_state_.contact(0) == 1 &&
           execution_state_.contact(1) == 1 &&
           execution_state_.contact(2) == 1 &&
           execution_state_.contact(3) == 1;
}

bool UnifiedGaitScheduler::isPlannerFullStance() const {
    return planner_state_.is_full_stance;
}

void UnifiedGaitScheduler::setGaitCommand(const Vec2& vxy_goal, double yaw_rate_goal, double gait_height) {
    vxy_goal_ = vxy_goal;
    yaw_rate_goal_ = yaw_rate_goal;
    gait_height_ = gait_height;
}

const Vec34& UnifiedGaitScheduler::getFootPosGoal() const {
    return foot_pos_goal_;
}

const Vec34& UnifiedGaitScheduler::getFootVelGoal() const {
    return foot_vel_goal_;
}

void UnifiedGaitScheduler::restart() {
    gait_generator_.restart();
    foot_pos_goal_.setZero();
    foot_vel_goal_.setZero();
    future_gait_segments_.clear();
}
