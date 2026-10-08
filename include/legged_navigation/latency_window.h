#ifndef LEGGED_NAVIGATION_LATENCY_WINDOW_H
#define LEGGED_NAVIGATION_LATENCY_WINDOW_H

#include <cstddef>
#include <deque>

namespace legged_navigation {

struct LatencySnapshot {
    std::size_t samples{};
    double p50_ms{};
    double p95_ms{};
    double maximum_ms{};
};

class LatencyWindow {
public:
    explicit LatencyWindow(std::size_t capacity);

    void observe(double latency_ms);
    [[nodiscard]] LatencySnapshot snapshot() const;
    [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }

private:
    std::size_t capacity_;
    std::deque<double> samples_;
};

}  // namespace legged_navigation

#endif
