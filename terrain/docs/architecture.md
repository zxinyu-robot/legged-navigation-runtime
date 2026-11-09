# Architecture

The public core defines a narrow contract between terrain perception and motion control. It does not implement FAST-LIO2, point-cloud segmentation, elevation mapping, locomotion or ROS 2 transport.

```text
LiDAR + IMU + contacts
  → LIO / terrain perception
  → BaseState / TerrainState / ContactState
  → TerrainAssessor
  → traversability + risk + safety mode
  → navigation / whole-body controller
```

## Contract rules

- Every state carries timestamp, frame, sequence and machine-readable fault flags.
- Confidence is separate from the estimated value.
- Sparse support and invalid normals fail safe.
- Perception produces constraints and risk; it does not command joints.
- Controllers may apply stricter limits but must not silently ignore critical faults.

## ROS 2 integration boundary

A production adapter should publish typed messages for base, terrain and contact state. QoS should favor latest-value delivery for high-rate pose/terrain streams and reliable delivery for fault transitions. TF, sensor calibration and clock health must be validated before the state is accepted.

## Evaluation

Trajectory quality, map consistency and end-to-end delay should be measured with rosbag2, evo, Foxglove and PlotJuggler. Terrain-state evaluation additionally needs labeled slope, support and roughness data plus false-safe/false-stop rates.
