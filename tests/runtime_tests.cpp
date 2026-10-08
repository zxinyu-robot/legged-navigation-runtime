#include "legged_navigation/latency_window.h"
#include "legged_navigation/relocalization_supervisor.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

void test_successful_recovery() {
    using namespace legged_navigation;

    RelocalizationSupervisor supervisor({
        .loss_confirmation_ms = 100,
        .search_timeout_ms = 1'000,
        .healthy_frames_to_resume = 2,
    });

    expect(
        supervisor.update({.timestamp_ms = 0, .tracking_valid = false}).action ==
            RecoveryAction::HoldMotion,
        "first invalid frame should hold motion");
    expect(
        supervisor.update({.timestamp_ms = 100, .tracking_valid = false}).current ==
            RecoveryState::Searching,
        "confirmed loss should start candidate search");
    expect(
        supervisor
                .update({
                    .timestamp_ms = 150,
                    .tracking_valid = false,
                    .descriptor_candidate_found = true,
                })
                .current == RecoveryState::Aligning,
        "descriptor candidate should start fine alignment");
    expect(
        supervisor
                .update({
                    .timestamp_ms = 250,
                    .tracking_valid = false,
                    .alignment_complete = true,
                    .alignment_success = true,
                    .pose_jump_m = 0.2,
                    .alignment_fitness = 0.1,
                })
                .current == RecoveryState::Verifying,
        "valid alignment should reset the localizer");
    supervisor.update({.timestamp_ms = 300, .tracking_valid = true});
    const RecoveryDecision resumed =
        supervisor.update({.timestamp_ms = 350, .tracking_valid = true});
    expect(
        resumed.current == RecoveryState::Tracking &&
            resumed.action == RecoveryAction::ResumeNavigation,
        "healthy verification frames should resume navigation");
}

void test_alignment_rejection_and_timeout() {
    using namespace legged_navigation;

    RelocalizationSupervisor supervisor({
        .loss_confirmation_ms = 0,
        .search_timeout_ms = 100,
        .healthy_frames_to_resume = 1,
        .maximum_pose_jump_m = 0.3,
        .maximum_alignment_fitness = 0.2,
    });

    supervisor.update({.timestamp_ms = 0, .tracking_valid = false});
    supervisor.update({.timestamp_ms = 0, .tracking_valid = false});
    supervisor.update({
        .timestamp_ms = 10,
        .tracking_valid = false,
        .descriptor_candidate_found = true,
    });
    const RecoveryDecision rejected = supervisor.update({
        .timestamp_ms = 20,
        .tracking_valid = false,
        .alignment_complete = true,
        .alignment_success = true,
        .pose_jump_m = 0.8,
        .alignment_fitness = 0.1,
    });
    expect(
        rejected.current == RecoveryState::Searching,
        "implausible pose jump should reject alignment");

    const RecoveryDecision stopped =
        supervisor.update({.timestamp_ms = 120, .tracking_valid = false});
    expect(
        stopped.current == RecoveryState::SafeStop &&
            stopped.action == RecoveryAction::EnterSafeStop,
        "search timeout should enter safe stop");
}

void test_latency_window() {
    using namespace legged_navigation;

    LatencyWindow window(4);
    window.observe(40.0);
    window.observe(10.0);
    window.observe(20.0);
    window.observe(30.0);
    window.observe(15.0);

    const LatencySnapshot snapshot = window.snapshot();
    expect(snapshot.samples == 4, "window should evict its oldest sample");
    expect(std::abs(snapshot.p50_ms - 15.0) < 1e-9, "p50 should use nearest rank");
    expect(std::abs(snapshot.p95_ms - 30.0) < 1e-9, "p95 should use nearest rank");
    expect(std::abs(snapshot.maximum_ms - 30.0) < 1e-9, "maximum should be tracked");

    bool threw = false;
    try {
        window.observe(-1.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "negative latency should be rejected");
}

}  // namespace

int main() {
    test_successful_recovery();
    test_alignment_rejection_and_timeout();
    test_latency_window();
    return 0;
}
