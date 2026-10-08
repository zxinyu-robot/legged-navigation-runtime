# Benchmark Methodology

## Metric definitions

### Localization frontend latency

Measure wall-clock time from the moment a synchronized LiDAR/IMU bundle becomes eligible for processing to the publication of the corresponding pose. Report P50, P95, P99 and maximum values after warm-up.

Do not mix sensor transport delay with algorithm processing time unless the report explicitly calls the metric end-to-end latency.

### LOST detection latency

Measure from the first observation that violates the localization-health policy to the supervisor's first `SearchCandidate` action. The configured confirmation interval is part of this metric.

### Relocalization latency

Measure from `SearchCandidate` to the accepted localizer reset. Mission verification and controller resume should be reported separately.

## Reproducible protocol

1. Pin CPU/GPU power mode and record device temperature.
2. Replay the same rosbag at real-time rate after a warm-up interval.
3. Record at least 1,000 frontend samples per platform and scenario.
4. Inject deterministic localization failures at recorded timestamps.
5. Report the hardware, ROS 2 distribution, middleware, map size and point count.
6. Publish raw samples or a checksum-addressed result file.
7. Separate synthetic tests, bag replay and live-robot measurements.

## Historical engineering figures

The associated private engineering project reported:

- G1 frontend P95: approximately 25.9 ms.
- H2 frontend P95: approximately 16.3 ms.
- automatic relocalization: approximately 1.0–1.2 s.
- LOST-path trigger/recovery response: approximately 0.5 s.

These figures are included as project context. They cannot be independently reproduced from this repository because the original hardware configuration, maps, bags and integration code are not public.

## Public-repository policy

CI validates state-machine behavior and latency-statistics correctness only. It must not be presented as a reproduction of robot performance. Future public benchmarks should be checked into a versioned `benchmarks/` directory with raw inputs and environment metadata.
