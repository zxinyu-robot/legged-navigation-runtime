#include "legged_navigation/latency_window.h"
#include "legged_navigation/relocalization_supervisor.h"

#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    using namespace legged_navigation;

    RelocalizationSupervisor supervisor;
    const std::vector<LocalizationObservation> replay{
        {.timestamp_ms = 0, .tracking_valid = true},
        {.timestamp_ms = 100, .tracking_valid = false},
        {.timestamp_ms = 600, .tracking_valid = false},
        {.timestamp_ms = 800,
         .tracking_valid = false,
         .descriptor_candidate_found = true},
        {.timestamp_ms = 1'100,
         .tracking_valid = false,
         .alignment_complete = true,
         .alignment_success = true,
         .pose_jump_m = 0.12,
         .alignment_fitness = 0.18},
        {.timestamp_ms = 1'200, .tracking_valid = true},
        {.timestamp_ms = 1'300, .tracking_valid = true},
        {.timestamp_ms = 1'400, .tracking_valid = true},
    };

    std::cout << "relocalization replay\n";
    for (const LocalizationObservation& observation : replay) {
        const RecoveryDecision decision = supervisor.update(observation);
        if (decision.action != RecoveryAction::None) {
            std::cout << "  t=" << std::setw(4) << observation.timestamp_ms
                      << " ms  " << RelocalizationSupervisor::name(decision.previous)
                      << " -> " << RelocalizationSupervisor::name(decision.current)
                      << "  action="
                      << RelocalizationSupervisor::name(decision.action) << '\n';
        }
    }

    LatencyWindow latency(10);
    for (double sample : {14.2, 16.1, 15.4, 18.7, 17.2,
                          20.1, 16.8, 19.5, 24.8, 17.9}) {
        latency.observe(sample);
    }
    const LatencySnapshot metrics = latency.snapshot();
    std::cout << "synthetic latency window: samples=" << metrics.samples
              << " p50=" << metrics.p50_ms << " ms"
              << " p95=" << metrics.p95_ms << " ms"
              << " max=" << metrics.maximum_ms << " ms\n";

    return supervisor.state() == RecoveryState::Tracking ? 0 : 1;
}
