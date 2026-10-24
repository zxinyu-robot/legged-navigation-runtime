#include "legged_navigation/recovery_plugin.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

legged_navigation::RecoveryInput frame(std::int64_t t,
                                       bool tracking_valid = true) {
    legged_navigation::RecoveryInput in;
    in.timestamp_ms = t;
    in.tracking_valid = tracking_valid;
    in.hints = {.gait = "trot", .environment = "stairs"};
    return in;
}

void test_healthy_frames_stay_full() {
    using namespace legged_navigation;
    RecoveryPlugin plugin;
    const auto out = plugin.update(frame(0));
    expect(out.authority == RecoveryAuthority::Full,
           "tracking frame should keep Full authority");
    expect(out.reason == RecoveryReason::None, "no reason on healthy frame");
    expect(out.hints.environment == "stairs", "hints pass through");
}

void test_loss_suspends_then_resumes_full() {
    using namespace legged_navigation;
    RecoveryPlugin plugin({.loss_confirmation_ms = 100,
                           .search_timeout_ms = 1'000,
                           .healthy_frames_to_resume = 2});

    auto out = plugin.update(frame(0, false));
    expect(out.action == RecoveryAction::HoldMotion, "invalid frame holds");
    out = plugin.update(frame(100, false));
    expect(out.current == RecoveryState::Searching,
           "confirmed loss starts searching");
    expect(out.authority == RecoveryAuthority::Suspended,
           "searching suspends authority");
    expect(out.reason == RecoveryReason::LocalizationLost,
           "Tracking->Searching reports LocalizationLost");

    // 配准成功 → Verifying → 恢复
    auto align = frame(150, false);
    align.descriptor_candidate_found = true;
    out = plugin.update(align);
    auto verify = frame(200, false);
    verify.alignment_complete = true;
    verify.alignment_success = true;
    out = plugin.update(verify);
    expect(out.authority == RecoveryAuthority::Suspended,
           "verifying still suspended");

    out = plugin.update(frame(300));
    out = plugin.update(frame(400));
    expect(out.current == RecoveryState::Tracking,
           "healthy frames resume tracking");
    expect(out.authority == RecoveryAuthority::Full,
           "resumed tracking restores Full");
}

void test_search_timeout_revokes() {
    using namespace legged_navigation;
    RecoveryPlugin plugin({.loss_confirmation_ms = 100,
                           .search_timeout_ms = 500});

    plugin.update(frame(0, false));
    auto out = plugin.update(frame(100, false));
    expect(out.current == RecoveryState::Searching, "searching");
    out = plugin.update(frame(700, false));
    expect(out.current == RecoveryState::SafeStop,
           "search timeout enters SafeStop");
    expect(out.authority == RecoveryAuthority::Revoked,
           "SafeStop revokes authority");
    expect(out.reason == RecoveryReason::RelocalizationTimeout,
           "SafeStop after search timeout reports RelocalizationTimeout");
}

void test_degraded_mode_restricts() {
    using namespace legged_navigation;
    RecoveryPlugin plugin;
    auto in = frame(0);
    in.localization_mode = 1;  // Degraded2D
    const auto out = plugin.update(in);
    expect(out.authority == RecoveryAuthority::Restricted,
           "Degraded2D mode restricts authority");
}

}  // namespace

int main() {
    test_healthy_frames_stay_full();
    test_loss_suspends_then_resumes_full();
    test_search_timeout_revokes();
    test_degraded_mode_restricts();
    return 0;
}
