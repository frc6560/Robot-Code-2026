# Runtime allocation review

The review covers all application subsystems (intake, shooter, indexer, vision,
swerve, LEDs), their shot/aim callers, current monitoring and telemetry. The goal
is reusable owner-specific storage, not static mutable objects shared by threads.
Objects needed by hardware controls remain distinct per motor/owner.

## Changes

- Intake: one shared bus wrapper; reusable CurrentLimitsConfigs for limit
  transitions; four preallocated refresh arrays. Safety enable flags and limit
  values are retained. Successful status messages use constants, and the fixed
  configuration summary is generated once.
- Indexer/shooter hardware: preallocated status-refresh arrays, existing reusable
  control requests retained. Indexer configuration summary cached at initialization.
- Shot solver: command and dashboard each own a result, updated with solveInto;
  diagnostics updated in place. Table scans and measured-RPM checks do not create
  objects or integrate physics. Convenience solve() still returns a fresh result
  for offline callers; it is no longer called by robot execution paths.
- Shooter characterization: sample lists replaced by five fixed running totals
  for the same least-squares fit. Sample history cannot grow with command duration.
- Vision: parse only changed pose frames; reuse the covariance and orientation
  buffers; iterate a fixed camera array without allocating list iterators.
- Swerve/aim: reuse alignment and aim command velocity buffers; avoid redundant
  hub-difference translations and release translations for the current zero
  offset. Driver clamping and disabled driving reuse owned speed objects. Distance
  command creates its requested velocity during command construction.
- LED hardware already owns and reuses its output buffer. Desktop flywheel colors
  now use initialization-time objects. Simulation inspection getters retain their
  snapshot semantics.
- Telemetry: primitive numeric pending values avoid per-call Double boxing; the
  pending-slot array avoids map iterators in every flush. First occurrence of a
  new key still creates its bounded map slot. File/NT logging remains at 2 Hz.
- Current monitoring: rebuild motor metadata and bus arrays only at registration;
  fixed top-three buffers replace runtime streams, records, lists and sorting.
  Dashboard rankings now show device name and the existing separate numeric value,
  rather than allocating a formatted combined sentence each update.
- Memory monitoring: paths, field names and telemetry keys cached. Linux file
  reading remains a once-per-second diagnostic operation.

## Allocations that remain

This code is not an end-to-end allocation-free robot. WPILib/YAGSL have immutable
pose/rotation types and allocate kinematics, pose-estimator and profiled-controller
state. Phoenix applies/serializes configurations and uses native buffers internally.
Fresh camera frames still allocate NetworkTables arrays, pose/tag data, and vision
estimation state. Startup, operator resets, autonomous path construction, error
messages, console formatting, logging snapshots/serialization, memory-file readers,
and initialization of previously unseen telemetry keys also allocate.

Do not mutate a pose object that an estimator retains, reuse an asynchronous log
snapshot as scratch, remove safety diagnostics, or add System.gc() in the control
loop simply to hide allocations. Eliminating these remaining allocations requires
changing library APIs/implementations or reducing functionality, not just moving
application constructors into fields. No claim of global zero allocations is made.

## Validation

The allocation regression test measures the warmed table-solver path with JVM
per-thread allocation counters, using 100,000 calls to each runtime operation.
Reusable result/invalid reset semantics, configuration identity and safety values,
ranking/tie ordering, primitive telemetry and normal subsystem behavior have tests.
The earlier full-suite run had six failures matching the six previously recorded
failing test names. Targeted checks and a headless autonomous simulation are used
to check these changes; desktop measurements do not replace roboRIO profiling.
