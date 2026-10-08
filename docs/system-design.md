# System Design

## Safety boundary

The recovery runtime coordinates localization components but does not command joint torque, body velocity, or foot placement. Every recovery action must pass through the robot's motion-safety layer.

The runtime assumes these integration ports:

- localization health input from FAST-LIO2 or a localizer watchdog;
- global descriptor candidates from Scan Context;
- coarse/fine alignment results from NDT and ICP;
- localizer reset service or lifecycle transition;
- Nav2 mission pause/resume action;
- safe-stop command accepted by the platform controller.

## Recovery states

### Tracking

Normal localization and navigation. A single invalid observation holds motion but does not immediately rebuild localization. Failure is confirmed only after the configured duration.

### Searching

Navigation remains paused while the global descriptor service searches cached maps or keyframes. A timeout enters `SafeStop`.

### Aligning

The selected candidate is refined with NDT/ICP. The result is accepted only when the solver reports success and both pose-jump and fitness gates pass.

### Verifying

The localizer has been reset but navigation remains paused. Consecutive healthy observations are required before task execution resumes. Any invalid observation returns to candidate search.

### SafeStop

Automatic recovery is exhausted or timed out. The supervisor remains latched until an operator or a higher-level recovery manager explicitly calls `reset()`.

## ROS 2 mapping

A production adapter can map actions as follows:

- `HoldMotion`: cancel or pause the active Nav2 action and request a stationary controller mode.
- `SearchCandidate`: invoke a Scan Context candidate service.
- `RunFineAlignment`: invoke an NDT/ICP alignment action with a deadline.
- `ResetLocalizer`: atomically update the map pose and reset the localizer state.
- `ResumeNavigation`: resume the mission from its last idempotent checkpoint.
- `EnterSafeStop`: publish a latched fault and invoke the platform safe-stop service.

The adapter should use ROS time consistently, reject stale responses with request IDs, and keep state-machine updates on one mutually exclusive callback group.

## Multi-floor navigation boundary

Multi-floor planning is modeled above the recovery runtime:

1. A topological planner selects a floor, staircase, or transition edge.
2. SCAN-Planner/PCT-Planner provides a traversable 3D segment.
3. A dedicated stair controller executes the transition.
4. The localizer switches map partitions only after transition validation.
5. The recovery runtime remains active throughout and can hold or abort the mission.

This separation prevents a localization reset from silently changing the semantic floor or resuming an obsolete path.
