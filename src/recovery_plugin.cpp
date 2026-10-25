#include "legged_navigation/recovery_plugin.h"

namespace legged_navigation {

RecoveryPlugin::RecoveryPlugin(RecoveryConfig config)
    : supervisor_(config), config_(config) {}

RecoveryOutput RecoveryPlugin::update(const RecoveryInput& input) {
    const LocalizationObservation observation{
        .timestamp_ms = input.timestamp_ms,
        .tracking_valid = input.tracking_valid,
        .descriptor_candidate_found = input.descriptor_candidate_found,
        .alignment_complete = input.alignment_complete,
        .alignment_success = input.alignment_success,
        .pose_jump_m = input.pose_jump_m,
        .alignment_fitness = input.alignment_fitness,
    };
    const RecoveryDecision decision = supervisor_.update(observation);

    RecoveryOutput output;
    output.previous = decision.previous;
    output.current = decision.current;
    output.action = decision.action;
    output.hints = input.hints;

    switch (decision.current) {
        case RecoveryState::Tracking:
            // 输入层声明退化通行模式时授权降级为 Restricted，其余为 Full。
            output.authority = input.localization_mode == 1
                                   ? RecoveryAuthority::Restricted
                                   : RecoveryAuthority::Full;
            break;
        case RecoveryState::Searching:
        case RecoveryState::Aligning:
        case RecoveryState::Verifying:
            output.authority = RecoveryAuthority::Suspended;
            break;
        case RecoveryState::SafeStop:
            output.authority = RecoveryAuthority::Revoked;
            break;
    }

    if (decision.current == RecoveryState::SafeStop) {
        // 进安全停：搜索/配准超时优先；跳变直接越限记 LocalizationJump。
        output.reason = input.pose_jump_m > config_.maximum_pose_jump_m
                            ? RecoveryReason::LocalizationJump
                            : RecoveryReason::RelocalizationTimeout;
    } else if (decision.previous == RecoveryState::Tracking &&
               decision.current == RecoveryState::Searching) {
        output.reason = input.pose_jump_m > config_.maximum_pose_jump_m
                            ? RecoveryReason::LocalizationJump
                            : RecoveryReason::LocalizationLost;
    } else if (decision.current == RecoveryState::Tracking) {
        output.reason = RecoveryReason::None;
    } else {
        // 恢复进行中：保留进入原因，由上一次输出携带；这里只给 None
        // 之外的最小语义——跳变越限贯穿全程可见。
        output.reason = input.pose_jump_m > config_.maximum_pose_jump_m
                            ? RecoveryReason::LocalizationJump
                            : RecoveryReason::None;
    }
    return output;
}

void RecoveryPlugin::reset(std::int64_t timestamp_ms) {
    supervisor_.reset(timestamp_ms);
}

RecoveryState RecoveryPlugin::state() const noexcept {
    return supervisor_.state();
}

std::string_view RecoveryPlugin::name(RecoveryAuthority authority) noexcept {
    switch (authority) {
        case RecoveryAuthority::Full: return "Full";
        case RecoveryAuthority::Restricted: return "Restricted";
        case RecoveryAuthority::Suspended: return "Suspended";
        case RecoveryAuthority::Revoked: return "Revoked";
    }
    return "Unknown";
}

std::string_view RecoveryPlugin::name(RecoveryReason reason) noexcept {
    switch (reason) {
        case RecoveryReason::None: return "None";
        case RecoveryReason::LocalizationLost: return "LocalizationLost";
        case RecoveryReason::LocalizationJump: return "LocalizationJump";
        case RecoveryReason::RelocalizationTimeout:
            return "RelocalizationTimeout";
    }
    return "Unknown";
}

}  // namespace legged_navigation
