# Two-hertz telemetry

Application log writes and input serialization go through `RateLimitedTelemetry`.
Each key may be written at most once per 500,000 microseconds. Intermediate
updates replace one pending value; `Robot.robotPeriodic()` flushes pending values,
including a final command stop after the command is no longer running. The three
caches (outputs, inputs, dashboard) each have a fixed maximum of 2,048 keys and do
not retain a history of suppressed samples.

All AdvantageKit receivers, including WPILOGWriter and NT4Publisher, are wrapped
in `RateLimitedReceiver`. Entire frames are forwarded at most once per 500,000
microseconds, so AdvantageKit's automatic DriverStation, Console, SystemStats,
performance, and other fields are also recorded/transmitted at no more than 2 Hz.
This cap uses the frame's robot timestamp, including during accelerated replay.

On hardware, scalar dashboard diagnostics use the same per-key cap. Desktop
simulation retains immediate dashboard state for control tests and interactive
simulation, but its recorded log frames and application log writes are still capped.
Chooser/command sendables, operator tuning requests, camera-control NetworkTables
messages, and Driver Station errors are control/error interfaces, not sampled
telemetry, and are not delayed by this limiter.

Removed logging work:

- Indexer mechanism animation and Flywheel/Mechanism serialization.
- Repeated LED color strings (the physical LED animation still runs normally).
- Bulk BLine trajectory/pose graphics and the duplicate Fuel hub/release poses.
- YAGSL's independent telemetry and the legacy Swerve/Field dashboard graphic.
- Duplicate scalar swerve pose dashboard values; use the sampled Swerve/Pose log.

Swerve raw/adjusted absolute encoder dashboard updates and encoder read-error
telemetry are disabled. Encoder feedback remains active for drivetrain control.

Sensors, scheduler execution, vision fusion, shooting readiness checks, motor
commands and physical LED updates retain their normal control-loop frequency.
Replay still consumes logged inputs every replay cycle. A sparse 2 Hz log cannot
reproduce the original 50 Hz control decisions with full fidelity.

On hardware, `SampledLoggedRobot` runs AdvantageKit's before/after capture hooks
only once per 500,000 microseconds, before table cloning and queue admission.
The scheduler and all control callbacks still execute every 20 ms. Both WPILOG
and NT4 receivers remain enabled. Desktop simulation/replay retain every-cycle
capture. Startup and exception frames are additional diagnostic exceptions.

The framework is adapted from AdvantageKit 26.0.2's BSD-licensed LoggedRobot in
the same package to access package-private capture hooks. Review this adaptation
when upgrading AdvantageKit. This reduces queue admission from 50 to 2 frames/s;
it does not establish that logging caused the memory exhaustion.

Brief faults between samples, including final console output before a crash, may be
absent from the sampled log; retain the Driver Station console when diagnosing a failure.

Production queue follow-up: hardware now retains at most eight queued snapshots
and evicts oldest frames under receiver backpressure. See `log-queue.md` for
queue metrics, the pinned AdvantageKit adaptation and packaging requirements.
