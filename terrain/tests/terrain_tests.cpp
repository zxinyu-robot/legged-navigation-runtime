#include "legged_terrain_state/terrain_assessor.h"

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

}  // namespace

int main() {
    using namespace legged_terrain;
    TerrainAssessor assessor;

    const TerrainDecision flat = assessor.evaluate(
        {{0, "base_link", 1}, 0.0, {0.0, 0.0, 1.0}, 0.01, 90, 100});
    expect(flat.risk == RiskLevel::Nominal, "flat terrain should be nominal");
    expect(flat.terrain.traversability > 0.75, "flat terrain should be traversable");

    const TerrainDecision slope = assessor.evaluate(
        {{1, "base_link", 2}, 0.1, {0.3, 0.0, 0.954}, 0.05, 80, 100});
    expect(slope.risk == RiskLevel::Caution, "moderate slope should reduce speed");

    const TerrainDecision sparse = assessor.evaluate(
        {{2, "base_link", 3}, 0.0, {0.0, 0.0, 1.0}, 0.01, 10, 100});
    expect(
        sparse.risk == RiskLevel::Critical &&
            sparse.safety_mode == SafetyMode::HoldPosition,
        "sparse support should hold position");
    expect(
        has_fault(sparse.terrain.header.faults, FaultFlag::SparseSupport),
        "sparse support fault should be explicit");

    const TerrainDecision invalid = assessor.evaluate(
        {{3, "base_link", 4}, 0.0, {0.0, 0.0, 0.0}, 0.01, 90, 100});
    expect(invalid.risk == RiskLevel::Critical, "invalid normal should be critical");
    return 0;
}
