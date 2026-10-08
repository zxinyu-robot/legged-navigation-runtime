#include "legged_navigation/relocalization_supervisor.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace legged_navigation {

RelocalizationSupervisor::RelocalizationSupervisor(RecoveryConfig config)
    : config_(std::move(config)) {
    if (config_.loss_confirmation_ms < 0 || config_.search_timeout_ms <= 0 ||
        config_.healthy_frames_to_resume == 0 ||
        config_.maximum_pose_jump_m < 0.0 ||
        config_.maximum_alignment_fitness < 0.0) {
        throw std::invalid_argument("invalid relocalization supervisor config");
    }
}

RecoveryDecision RelocalizationSupervisor::update(
    const LocalizationObservation& observation) {
    if (observation.timestamp_ms < last_timestamp_ms_) {
        throw std::invalid_argument("observation timestamps must be monotonic");
    }
    last_timestamp_ms_ = observation.timestamp_ms;

    switch (state_) {
        case RecoveryState::Tracking:
            if (observation.tracking_valid) {
                loss_started_ms_ = -1;
                return stay();
            }
            if (loss_started_ms_ < 0) {
                loss_started_ms_ = observation.timestamp_ms;
                return stay(RecoveryAction::HoldMotion);
            }
            if (observation.timestamp_ms - loss_started_ms_ >=
                config_.loss_confirmation_ms) {
                return transition(
                    RecoveryState::Searching,
                    RecoveryAction::SearchCandidate,
                    observation.timestamp_ms);
            }
            return stay(RecoveryAction::HoldMotion);

        case RecoveryState::Searching:
            if (observation.descriptor_candidate_found) {
                return transition(
                    RecoveryState::Aligning,
                    RecoveryAction::RunFineAlignment,
                    observation.timestamp_ms);
            }
            if (observation.timestamp_ms - state_entered_ms_ >=
                config_.search_timeout_ms) {
                return transition(
                    RecoveryState::SafeStop,
                    RecoveryAction::EnterSafeStop,
                    observation.timestamp_ms);
            }
            return stay(RecoveryAction::SearchCandidate);

        case RecoveryState::Aligning:
            if (!observation.alignment_complete) {
                if (observation.timestamp_ms - state_entered_ms_ >=
                    config_.search_timeout_ms) {
                    return transition(
                        RecoveryState::SafeStop,
                        RecoveryAction::EnterSafeStop,
                        observation.timestamp_ms);
                }
                return stay();
            }
            if (observation.alignment_success &&
                std::isfinite(observation.pose_jump_m) &&
                std::isfinite(observation.alignment_fitness) &&
                std::abs(observation.pose_jump_m) <=
                    config_.maximum_pose_jump_m &&
                observation.alignment_fitness <=
                    config_.maximum_alignment_fitness) {
                healthy_frames_ = 0;
                return transition(
                    RecoveryState::Verifying,
                    RecoveryAction::ResetLocalizer,
                    observation.timestamp_ms);
            }
            return transition(
                RecoveryState::Searching,
                RecoveryAction::SearchCandidate,
                observation.timestamp_ms);

        case RecoveryState::Verifying:
            if (!observation.tracking_valid) {
                healthy_frames_ = 0;
                return transition(
                    RecoveryState::Searching,
                    RecoveryAction::SearchCandidate,
                    observation.timestamp_ms);
            }
            ++healthy_frames_;
            if (healthy_frames_ >= config_.healthy_frames_to_resume) {
                loss_started_ms_ = -1;
                return transition(
                    RecoveryState::Tracking,
                    RecoveryAction::ResumeNavigation,
                    observation.timestamp_ms);
            }
            return stay();

        case RecoveryState::SafeStop:
            return stay();
    }

    throw std::logic_error("unhandled recovery state");
}

void RelocalizationSupervisor::reset(std::int64_t timestamp_ms) {
    state_ = RecoveryState::Tracking;
    state_entered_ms_ = timestamp_ms;
    last_timestamp_ms_ = timestamp_ms;
    loss_started_ms_ = -1;
    healthy_frames_ = 0;
}

RecoveryDecision RelocalizationSupervisor::transition(
    RecoveryState next,
    RecoveryAction action,
    std::int64_t timestamp_ms) {
    const RecoveryState previous = state_;
    state_ = next;
    state_entered_ms_ = timestamp_ms;
    return {previous, state_, action};
}

RecoveryDecision RelocalizationSupervisor::stay(
    RecoveryAction action) const {
    return {state_, state_, action};
}

std::string_view RelocalizationSupervisor::name(
    RecoveryState state) noexcept {
    switch (state) {
        case RecoveryState::Tracking:
            return "tracking";
        case RecoveryState::Searching:
            return "searching";
        case RecoveryState::Aligning:
            return "aligning";
        case RecoveryState::Verifying:
            return "verifying";
        case RecoveryState::SafeStop:
            return "safe_stop";
    }
    return "unknown";
}

std::string_view RelocalizationSupervisor::name(
    RecoveryAction action) noexcept {
    switch (action) {
        case RecoveryAction::None:
            return "none";
        case RecoveryAction::HoldMotion:
            return "hold_motion";
        case RecoveryAction::SearchCandidate:
            return "search_candidate";
        case RecoveryAction::RunFineAlignment:
            return "run_fine_alignment";
        case RecoveryAction::ResetLocalizer:
            return "reset_localizer";
        case RecoveryAction::ResumeNavigation:
            return "resume_navigation";
        case RecoveryAction::EnterSafeStop:
            return "enter_safe_stop";
    }
    return "unknown";
}

}  // namespace legged_navigation
