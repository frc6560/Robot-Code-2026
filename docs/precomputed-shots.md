# Precomputed shooter trajectories

The robot loads `src/main/deploy/shooting/trajectories.csv` rather than integrating
shot physics at startup or while shooting. The table was generated with the fixed
launch-speed coefficient 1.025 and the current ideal calibration and physics constants.
This changes where the calculation runs, not the model's physical accuracy.

The table includes 561 flights from 1000 to 3800 RPM in 5 RPM steps. The solver
still scans its original 10 RPM grid; every possible band midpoint lies on the
5 RPM grid. It looks up the midpoint exactly, without interpolation or integration.
For measured motor RPM between grid points, release readiness conservatively requires
both neighboring grid predictions to score. This can slightly narrow the ready band
by up to one 5 RPM interval, and avoids integrations and temporary prediction objects.
The immutable table entries are shared by lookups. Shot results and diagnostics
are still small immutable objects, to avoid mutable snapshots changing under logging.

After changing calibration, coefficient, or physics constants, run:

```
./gradlew generateShotTable
./gradlew test --tests '*PrecomputedShotsTest' --tests '*FuelShotModelTest'
```

Commit the regenerated CSV with the constants. A missing or stale table is rejected
at startup, rather than silently running stale predictions or falling back to on-robot
simulation. If the numerical algorithm itself changes, bump `rk4-v1` in the table
signature and regenerate. Hardware coefficient changes require regeneration.

Allocation reductions also include reusable camera orientation arrays, current-monitor
CAN bus groups and scratch lists, and integrator scratch arrays for offline calculation.
Scratch objects are owned by their subsystem/model rather than shared across threads.
This does not eliminate allocations inside WPILib, NetworkTables, or vendor libraries.

Runtime allocation update: command and dashboard now each own a reusable `Shot`.
They call `solveInto(range, result)`, which updates it in place. The diagnostic
view is also reused. `solve(range)` remains an allocating convenience method for
offline tests/tools. These mutable views are scheduler-thread owned and must not
be retained as historical snapshots or passed to asynchronous logging; telemetry
records their scalar values. The warmed desktop allocation test requires zero
bytes allocated by 100,000 solveInto calls and 100,000 scores calls.
