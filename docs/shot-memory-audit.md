# Shot memory audit — October 31, 2026

Historical baseline before the runtime reuse changes: see `runtime-allocations.md`
and `precomputed-shots.md` for the current solveInto API and fixed-size characterization.

## Version and limits

Audited the working tree based on 9278df5 and the older calculator implementations
in 2933557, be9839d, and c59b459. No live roboRIO, crash message, heap dump, kernel
OOM report, or deployed build identity was available. The user's observation that
pre-calculator code works is evidence of a regression, but does not isolate the
calculator from other changes introduced with it. This audit cannot certify every
allocation or leak in native vendor libraries.

## Current runtime computation

RobotContainer loads one PrecomputedShots model from the deploy CSV. The only
ordinary constructor that integrates physics is called by the offline generator
and software tests. There is no automatic runtime fallback to physics.

- Startup parses 561 CSV flights at 5 RPM spacing. Reader closes via try-with-resources.
  Row strings, split arrays, numeric-parser temporaries and signature reflection
  objects become eligible for collection. The model retains the table.
- The model also retains a 281-element boxed RPM list and 281 flight references,
  pointing into the same table rather than duplicating those Flight objects.
  Its six scratch arrays are fixed size. Estimated table/list storage is 47,144
  bytes with the existing approximate sizing method; this is not a heap-dump measurement.
- Dashboard solve runs at 2 Hz. Shot-command solve runs when range changes by more
  than 2 cm, limited to 10 Hz. Each scans 281 entries and returns a Shot record,
  replacing the previous diagnostic record. There is no solve-history collection.
- Release readiness runs each command iteration. scores() uses neighboring table
  entries for measured RPM. No integrations or temporary Flight objects.
- The coefficient setter is called each iteration, but unchanged coefficient 1.025
  returns immediately. A different coefficient on a table model is rejected; it
  does not trigger simulations. The offline numerical model clears/replaces its
  bounded cache when its coefficient changes.

## Measured allocation test

ShotMemoryAuditTest warms up 10,000 solves, then runs 100,000 solves and 100,000
readiness checks on the saved model. The desktop JVM's per-thread allocation
counter reported:

- solve: 9,600,000 bytes total, 96 bytes per call (Shot plus diagnostics).
- scores: zero bytes during the measured loop.
- physics integrations: zero.
- cache count and estimated bytes: unchanged.

The result objects are deliberately allowed to escape to avoid benchmarking only
an optimized-away result. At the current combined maximum 12 solves/sec, this
corresponds to about 1,152 bytes/sec for these records, excluding all callers and
framework work. JVM object sizes/optimizations can differ on the roboRIO. This is
an allocation measurement, not a full retained-heap or native-memory profile.

## Historical calculator pressure

The earlier calculator did run integrations at startup, while solving, and for
measured-RPM scoring. Each flight allocated six double arrays plus launch/result
objects. A successful solve in 2933557 integrated the midpoint twice, once directly
and once through scores(). Earlier versions also refined band edges through
repeated integrations; moving-shot code had further seven-element RK4 arrays.
These are temporary allocations and CPU work, not evidence of an unbounded leak.
They can still increase GC pauses and worsen a controller already near its limit.
No full state history was retained for each integration timestep.

## Other memory paths found in the shooting feature

1. Geometry/aiming: releasePosition constructs immutable translations; aim uses
   additional translations, chassis-speed objects and profiled-controller states.
   Temporary allocation continues even with a precomputed shot table.
2. Diagnostics: numeric boxing and some status strings are constructed before
   telemetry's limiter. Cached latest values keep the most recent object for each
   key; superseded objects are eligible for collection. Fixed-key use is bounded.
3. Logging: the latest-value caches each cap at 2,048 keys. This bounds key count,
   not the size of an arbitrary array/string value. AdvantageKit also retains its
   current log table and a bounded receiver backlog. Hardware frame admission now
   runs at 2 Hz; a slow receiver can still retain up to its 500-frame queue capacity.
   A bounded queue can exhaust a small heap without being an infinite leak.
4. Characterization: velocitySamples and voltageSamples append for as long as the
   optional feedforward characterization command runs. They are cleared at its next
   initialization, but retained by the dashboard command afterward. This is the
   clear unbounded-duration collection found in the shooter subsystem; normal shots
   do not run it.
5. Commands/controllers: model and command objects are retained for reuse. No
   command creation or button-binding registration was found inside the normal shot
   execution loop. WPILib 2026.2.1 caches controller trigger objects; repeated calls
   to rightTrigger() may allocate lookup keys but do not register a new trigger each time.
6. Hardware and vision: CTRE motor/CAN objects and NetworkTables use native memory;
   vision creates pose/tag arrays every update. Their allocations and driver state
   are outside the measured model loop. The flywheel visualizer/physics simulator
   is desktop-only and is not the real robot's shot predictor.

## Garbage collection and crash diagnosis

Java collects unreachable Java objects automatically. It cannot free a referenced
cache, retained command history, or native allocations. It can reuse freed heap
space without returning the committed heap pages to Linux. A Linux OOM kill can
therefore happen even with a modest live Java heap. Adding System.gc() to the robot
loop is not a fix and could cause control pauses; no explicit GC was added.

Collect the exact failure message and these readings on the failing deployed build:
HeapUsedMiB, HeapCommittedMiB, HeapMaxMiB, ProcessResidentMiB, SystemAvailableMiB,
and Logger/QueuedCycles. Heap growth suggests Java retention; stable heap with
process growth suggests native memory or JVM overhead; system-wide pressure can
also come from other processes. Distinguish Java OutOfMemoryError, Linux OOM kill,
Watchdog overruns, brownout, and ordinary exceptions instead of inferring the cause
from a free-memory number alone.

The audit found and corrected an escaped-regex bug in MemoryMonitor: valid Linux
memory lines could fail parsing and be silently ignored. Its parser now has tests.
This diagnostic fix and the audit tests are local changes, not deployed evidence.
