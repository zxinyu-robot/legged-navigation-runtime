#include "legged_terrain_state/terrain_assessor.h"

#include <iostream>
#include <vector>

int main() {
    using namespace legged_terrain;

    TerrainAssessor assessor;
    const std::vector<TerrainObservation> replay{
        {{0, "base_link", 1}, 0.0, {0.0, 0.0, 1.0}, 0.01, 90, 100},
        {{100'000'000, "base_link", 2}, 0.04, {0.20, 0.0, 0.98}, 0.05, 80, 100},
        {{200'000'000, "base_link", 3}, 0.12, {0.50, 0.0, 0.86}, 0.12, 70, 100},
        {{300'000'000, "base_link", 4}, 0.0, {0.0, 0.0, 1.0}, 0.01, 10, 100},
    };

    std::cout << "terrain safety replay\n";
    for (const TerrainObservation& observation : replay) {
        const TerrainDecision decision = assessor.evaluate(observation);
        std::cout << "  seq=" << observation.header.sequence
                  << " slope=" << decision.terrain.slope_deg
                  << "deg roughness=" << decision.terrain.roughness_m
                  << "m confidence=" << decision.terrain.support_confidence
                  << " risk=" << TerrainAssessor::name(decision.risk)
                  << " action=" << TerrainAssessor::name(decision.safety_mode)
                  << '\n';
    }
    return 0;
}
