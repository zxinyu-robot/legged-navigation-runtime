#include "legged_navigation/latency_window.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace legged_navigation {
namespace {

double percentile(const std::vector<double>& sorted, double value) {
    if (sorted.empty()) {
        return 0.0;
    }
    const double rank = std::ceil(value * static_cast<double>(sorted.size()));
    const std::size_t index =
        static_cast<std::size_t>(std::max(1.0, rank)) - 1;
    return sorted[index];
}

}  // namespace

LatencyWindow::LatencyWindow(std::size_t capacity) : capacity_(capacity) {
    if (capacity_ == 0) {
        throw std::invalid_argument("latency window capacity must be positive");
    }
}

void LatencyWindow::observe(double latency_ms) {
    if (!std::isfinite(latency_ms) || latency_ms < 0.0) {
        throw std::invalid_argument("latency sample must be finite and non-negative");
    }
    if (samples_.size() == capacity_) {
        samples_.pop_front();
    }
    samples_.push_back(latency_ms);
}

LatencySnapshot LatencyWindow::snapshot() const {
    if (samples_.empty()) {
        return {};
    }
    std::vector<double> sorted(samples_.begin(), samples_.end());
    std::sort(sorted.begin(), sorted.end());
    return {
        sorted.size(),
        percentile(sorted, 0.50),
        percentile(sorted, 0.95),
        sorted.back(),
    };
}

}  // namespace legged_navigation
