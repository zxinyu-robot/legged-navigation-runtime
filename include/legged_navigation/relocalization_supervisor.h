#ifndef LEGGED_NAVIGATION_RELOCALIZATION_SUPERVISOR_H
#define LEGGED_NAVIGATION_RELOCALIZATION_SUPERVISOR_H

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace legged_navigation {

enum class RecoveryState {
    Tracking,
    Searching,
    Aligning,
    Verifying,
    SafeStop,
};

enum class RecoveryAction {
    None,
    HoldMotion,
    SearchCandidate,
    RunFineAlignment,
    ResetLocalizer,
    ResumeNavigation,
    EnterSafeStop,
};

struct RecoveryConfig {
    std::int64_t loss_confirmation_ms{500};
    std::int64_t search_timeout_ms{2'500};
    std::size_t healthy_frames_to_resume{3};
    double maximum_pose_jump_m{0.5};
    double maximum_alignment_fitness{0.35};
};

struct LocalizationObservation {
    std::int64_t timestamp_ms{};
    bool tracking_valid{true};
    bool descriptor_candidate_found{};
    bool alignment_complete{};
    bool alignment_success{};
    double pose_jump_m{};
    double alignment_fitness{};
};

struct RecoveryDecision {
    RecoveryState previous{RecoveryState::Tracking};
    RecoveryState current{RecoveryState::Tracking};
    RecoveryAction action{RecoveryAction::None};

    [[nodiscard]] bool state_changed() const noexcept {
        return previous != current;
    }
};

class RelocalizationSupervisor {
public:
    explicit RelocalizationSupervisor(RecoveryConfig config = {});

    RecoveryDecision update(const LocalizationObservation& observation);
    void reset(std::int64_t timestamp_ms = 0);

    [[nodiscard]] RecoveryState state() const noexcept { return state_; }
    [[nodiscard]] static std::string_view name(RecoveryState state) noexcept;
    [[nodiscard]] static std::string_view name(RecoveryAction action) noexcept;

private:
    RecoveryDecision transition(
        RecoveryState next,
        RecoveryAction action,
        std::int64_t timestamp_ms);
    RecoveryDecision stay(RecoveryAction action = RecoveryAction::None) const;

    RecoveryConfig config_;
    RecoveryState state_{RecoveryState::Tracking};
    std::int64_t state_entered_ms_{};
    std::int64_t last_timestamp_ms_{};
    std::int64_t loss_started_ms_{-1};
    std::size_t healthy_frames_{};
};

}  // namespace legged_navigation

#endif
