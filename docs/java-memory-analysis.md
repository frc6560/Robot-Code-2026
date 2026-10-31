# Java memory allocation and retention analysis

Revision: `1565d7befc3601df984704d473a85a7c022f907e` (`tidle_tumble`).
Analysis date: October 31, 2026. The analysis itself did not change robot behavior.
Follow-up: the Limelight count/length guard described in risk #1 has since been
implemented locally; see `memory-analysis/limelight-hypothesis.md`. Findings below
describe the stated baseline revision. A subsequent local queue adaptation also
reduces hardware backlog to eight frames with oldest-frame eviction; see
`log-queue.md`. The 500-frame estimates below remain baseline scenarios.

## Result

No confirmed unbounded retained-Java-heap leak was found in the normal shooter,
intake, indexer, LED, vision, or drivetrain application loops reviewed. This is
not a proof that the entire process cannot leak. Native vendor code, VM state,
input sizes and the running deployed build were not profiled on the roboRIO.

The largest concrete Java-heap growth mechanism is AdvantageKit's receiver
backlog. It is bounded at 500 frames but can still consume multiple MiB. A
separate concrete OOM vulnerability exists in LimelightHelpers: an unchecked
received tag count determines an array allocation before frame validation.

The current shot calculator is a small fixed cache. Runtime table solving and
scoring use reusable results; they do not perform integrations. Old calculator
versions did produce integration arrays and more GC/CPU pressure.

## Scope and interpretation

Reviewed 71 application Java files and relevant installed source for AdvantageKit
26.0.2, BLine v0.9.1, YAGSL 2026.1.12 and WPILib 2026.2.1. The supporting
`memory-analysis/allocation-sites.txt` lists 685 textual allocation/growth
candidates. This includes initialization, desktop tooling and inactive APIs;
it is not a list of 685 runtime leaks. Hidden allocations through library calls,
boxing, concatenation and varargs are discussed separately below.

- Fixed retained memory remains reachable at steady state.
- Bounded growth fills a queue/cache/history and then stops; it can still cause OOM.
- Allocation churn creates objects repeatedly but releases old references. Cumulative
  allocated bytes can grow indefinitely while live heap remains approximately flat.
- A leak retains objects indefinitely with a reachable owner and no effective bound.
- A burst can request an enormous object once and OOM without leaking anything.

Byte estimates are not measurements of the deployed ARM VM. Object headers,
reference widths, alignment, string storage and collector settings change sizes.
The sorted lists below rank the largest plausible or known Java-heap consumers;
unmeasured library object graphs cannot be honestly placed in an exact byte order.

## 1. Growth and burst risks, ranked by plausible peak heap impact

| Rank | Owner / allocation | Growth bound and release | Assessment |
|---|---|---|---|
| 1 | LimelightHelpers `getBotPoseEstimate`, lines 751–775 | `new RawFiducial[tagCount]` trusts an incoming double cast to int, before checking array length | Concrete input-dependent OOM risk. A huge positive count can request a multi-MiB/GB array; negative values can throw. Not a normal-duration leak. No evidence that actual camera packets are malformed. |
| 2 | AdvantageKit static receiver queue | 500 cloned LogTables; drained on receiver thread | Concrete bounded retained growth. File I/O or a receiver stall can allow backlog. This is the first heap growth metric to inspect. |
| 3 | LogTable data/subtables, NT4 publishers, WPILOG entry metadata | Grow with unique keys/types, not elapsed time alone | Library maps have no application-style MAX_KEYS cap. Current application/BLine names appear stable, so normal use should plateau. Future timestamp/sequence-number keys would change that. |
| 4 | NetworkTables/camera arrays and native buffers | Incoming payload sizes and connection state; library-dependent | Network-supplied arrays are size-dependent. Java arrays are visible to GC; native buffers are not. Total cannot be statically bounded from this application alone. |
| 5 | Application telemetry maps | 3 caches, max 2,048 keys each; one latest value per key | Bounded key warmup. Values remain referenced after transmission, until replaced or program exit. Number of keys is capped; size of a String/array value is not. |
| 6 | BLine robotTranslations | Adds one point every third execute; trims >300 entries to 250 | Bounded history, not a leak. Initialization clears it; backing capacity can remain at its high-water mark. Graphics callbacks do not prevent internal history/array creation. |
| 7 | WPILib odometry/vision history | Time window, 1.5 seconds in PoseEstimator; normal monotonic updates prune old entries | Bounded under normal timestamps/rates. Current application rejects stale/future camera timestamps. No ever-growing pose history found. |
| 8 | Offline custom path planner / desktop diagnostics | A* scales with grid size; trace StringBuilder grows with simulation duration | Large possible temporary arrays, but no active normal robot caller of the custom A* planner was found. Desktop diagnostic trace is simulation-only and finishes after its configured run. Do not attribute these to real-robot shooting. |

### Log-backlog size example

AdvantageKit `LogTable.clone` copies the HashMap structure and shares existing
LogValue references. It does NOT deep-copy every unchanged payload per queued
frame. However, new readings replace values, so older queued frames can retain
previous LogValues and payload arrays.

Let F = field count and Q = queued frames. Under a common 64-bit JVM layout with
4-byte references, the cloned map structure is roughly 40 bytes per field,
including average bucket overhead, plus per-table overhead. Thus:

- F=500, Q=500: roughly 9.5 MiB of map structure alone.
- F=1,000, Q=500: roughly 19 MiB of map structure alone.
- Distinct retained values, copied arrays/strings, publisher metadata and serialization
  add to this. These F values are scenarios, not measured robot field counts.

Current hardware admission is 2 Hz, so an undrained queue would take roughly
250 seconds to fill, ignoring startup/exception frames. Previously 50 Hz admission
could fill the same queue in about 10 seconds. Slower admission reduces the rate,
not its 500-frame capacity. Even after the queue is full, clone construction occurs
before `add()` rejects the frame, causing additional temporary allocation.

## 2. Fixed / steady-state retained allocations

| Likely size tier | Retained objects | Static size evidence |
|---|---|---|
| Largest unknown graphs | Phoenix controllers/status signals, YAGSL drivetrain, WPILib math/estimator/controller state, NetworkTables/AdvantageKit schemas and publisher metadata | 16 motor controllers configured: 8 drive/steering + 3 intake + 3 indexer + 2 shooter; associated encoders/IMU, cached signals and native handles. Exact Java retained sizes cannot be derived from constructor counts. Native allocations are additional. |
| Potentially MiB with growth | Logger current/last tables, queue snapshots, NT publisher maps | See backlog/key formulas above. Queued frames are not all allocated at startup; the 500-slot queue array is small compared with its eventual contents. |
| Tens of KiB baseline, potentially hundreds of KiB or more at key cap | Three LatestValueLimiter instances | Their three fixed Pending[2048] arrays require 24 KiB of references at 4-byte refs, 48 KiB at 8-byte refs, excluding array headers. Pending objects, map entries and keys are added lazily. At all 6,144 slots filled, metadata alone is on the order of 0.5–1 MiB, with key strings and retained values additional. Current occupancy was not measured. |
| Approximately 40–64 KiB | Shooter precomputed model/table | 561 Flight objects, one table reference array, 281 boxed RPMs, two ArrayList backing arrays and fixed scratch/results. Existing rough estimator reports 47,144 bytes. This is not a growing shot history. |
| Typically tens of KiB; depends on update rate/frame size | Odometry history, three latest camera estimates, BLine history | Estimator sample count approximately rate × 1.5 s; BLine <=300 retained point references. A huge camera frame invalidates any small-size assumption. |
| Small fixed objects/buffers | Current-limit configuration, motor-control requests, sensor-refresh arrays, LED buffer, characterization accumulator, memory monitor buffers | One reusable intake CurrentLimitsConfigs. Sensor arrays proportional to installed devices. LED hardware buffer has 65 pixels. Characterization keeps five doubles (40 bytes of primitive payload), not sample lists. |
| Small fixed duplication | SampledLoggedRobot plus its LoggedRobot superclass | Both initialize their own notifier/GC-stat fields. Superclass loop is unused; extra native handle and small fixed stats state remain. Wasteful fixed duplication, not growth per loop. |

Calibration: four double fields ×561 =17,952 bytes of primitive trajectory data,
plus 561 boolean fields. Everything above this is headers, padding, references,
boxed RPMs, list capacity and other fixed objects. The CSV's disk size and the
application JAR's size do not directly equal retained heap; not all class files or
backup sources inside the JAR are live Java objects at once.

## 3. Runtime allocation churn, largest likely candidates first

The ranking is qualitative; an allocation profiler is required for exact bytes/sec.

1. Drivetrain math and estimation (50 Hz control plus the YAGSL odometry notifier):
   module state/position arrays and elements, chassis conversions, rotations,
   pose/estimator snapshots and matrix intermediates. Internal library allocations
   remain despite reusable application input buffers. Estimator history is pruned.
2. Fresh vision frames (up to one parse/camera/control iteration): timestamped array,
   primitive pose array, Pose2d/Translation2d/Rotation2d, PoseEstimate, RawFiducial[]
   and one RawFiducial per tag, followed by estimator computations. Normal primitive
   input array payload is 8*(11+7*T) bytes for T tags, before wrappers/tag objects.
   No parse on an unchanged NT timestamp. Only the newest estimate is retained by
   each camera interface, apart from downstream estimator history.
3. BLine path following: every scalar log call constructs a Pair and boxes its value
   inside BLine; our callback also constructs `"BLine/" + key` before rate limiting.
   Pose graphics construct Pose2d/Pair even though our callback is a no-op. Every
   third execute creates a history reference array before passing it to the disabled
   graphics callback. At a full 300 points this array is ~1.2 KiB with 4-byte refs,
   ~2.4 KiB with 8-byte refs; at 50 Hz/3 that's ~20–40 KiB/sec of reference-array
   allocation alone while following. Array copies share point objects. These are
   churn, not an unbounded retained trajectory list.
4. AdvantageKit (hardware 2 Hz): each recorded scalar creates a LogValue; structured
   values serialize/copy arrays, full tables clone map nodes, receiver comparisons
   and serialization do additional work. The application primitive limiter removes
   upstream Double boxing, not all allocations inside Logger.
5. Controller APIs and status/error paths: `rightTrigger()` uses a cached Trigger
   but the lookup creates a Pair key/boxed threshold in CommandGenericHID. It does
   not continually register new button events. ProfiledPIDController creates profile
   states. Fault strings, exceptions, Driver Station reporting and stack traces
   allocate, especially if a fault repeats every loop.
6. Memory diagnostics (1 Hz): two reader/decoder/buffer object graphs, line strings,
   numeric substrings, final console String and console/Logger capture. Cached
   StringBuilder does not make printing allocation-free: `toString()` creates the
   output string. Reader resources close each sample. Live application state stays
   fixed except the bounded logging consequences.
7. Mode/autonomous changes: deferred command suppliers build paths/controllers and
   commands when scheduled. JSON parsing/rotation creates transient graphs. The
   default six competition chooser entries are deferred wrappers, not six fully
   loaded competition paths at startup. DeferredCommand clears its current command
   at end; WPILib composed-command tracking uses weak keys. No ordinary once-per-loop
   autonomous graph construction was found.
8. Current monitoring: fixed motor metadata, refresh arrays and top-three storage
   are reused. Register() rebuilds arrays/streams during initialization; repeated
   registration with new names would grow its map, but no periodic caller was found.
9. Shot solveInto/scores: no object construction in the current table-backed normal
   path. Convenience solve() creates a Shot and offline numerical models still
   allocate Flight/Launch results; those are not the robot's selected construction path.

## 4. Reachability and leak checks

- Shooter: PrecomputedShots.load is the sole normal RobotContainer construction
  path. One shared model, 561 fixed entries. solveInto uses counted loops and reuses
  diagnostics/result objects. Coefficient remains fixed at 1.025; setter returns
  immediately when unchanged. It rejects changes on table models instead of growing
  caches or falling back to physics. No stored list of prior shots.
- Intake/indexer/shooter current and motor requests: constructor-owned objects
  reused. Repeated hardware apply may allocate internally, but current-limit code
  only applies on transitions/failures, not every identical request.
- Characterization: former unbounded lists have been replaced with fixed sums.
- Telemetry: latest pending value overwrites predecessor. Pending.value stays
  reachable even after a successful write, retaining one last object per key;
  previous objects are released. Primitive offers clear old Object references.
- Limelight static entry cache: no eviction, but keys are fixed configured camera/
  entry combinations in normal execution. Dynamic camera/entry names could leak
  native entry handles and grow Java metadata. No such normal caller was found.
- BLine registry: static event registry overwrites the same six names on restarts.
  Event-index set is bounded by the path and clears on initialization. Retained
  movement history caps at 300; its export array is allocated before our callback.
- `NtValueDisplay.add`: schedules a fresh never-ending AlwaysRunCommand per call;
  repeated runtime use would retain commands/suppliers/widgets indefinitely. No
  robot caller was found, so this is a latent API risk, not a demonstrated live leak.
- Custom A*/AdaptiveSampler grids: large arrays and boxed samples scale with grid/
  recursion bounds. No active main-robot caller; GUI/offline APIs remain allocation
  heavy. Simulation-only trace growth is not a real-controller allocation path.
- Shared mutable results are not recorded by reference into asynchronous logs;
  application logging extracts numeric fields. No growing thread/timer creation
  loop or repeated periodic subscription registration was found in application code.

## 5. Recommended next actions

1. Guard Limelight tag count before allocation: require finite, integral, nonnegative
   count, sensible upper bound and compatible payload length before creating objects.
   Use length-safe arithmetic; reject malformed packets. This report leaves code unchanged.
2. Capture Logger/QueuedCycles and the memory fields together on the failing build.
   A queue trending upward and a rising post-GC heap strongly implicate backlog.
   Consider a substantially smaller bounded queue/latest-frame policy in an approved
   AdvantageKit adaptation if lost diagnostic frames are acceptable.
3. Change BLine logging at the producer, not just its callbacks: suppress unneeded
   Pair/array/history generation before construction, and cache fixed scalar keys.
4. Check the effective JVM heap limit and GC on the roboRIO. No explicit heap sizing
   override is present in this project's build.gradle; deployment defaults/runtime
   settings must be checked, not inferred from the desktop -Xmx64m experiment.
5. Distinguish Java OutOfMemoryError (heap, native thread, direct buffer, etc.) from
   a Linux OOM kill, native malloc failure, watchdog timeout or an ordinary exception.
   Stable Java live heap does not rule out excessive process RSS or native leaks.

Repeated allocations alone are not evidence of a leak. GC automatically reclaims
unreachable Java objects but cannot free live queue/cache contents, native memory,
or force the OS to preserve the process until collection. Committed heap pages
may remain resident after Java objects are reclaimed. No System.gc() loop is advised.

## 6. Evidence and validation

- Installed dependency excerpts: `memory-analysis/dependency-evidence.txt`.
- Application candidate inventory: `memory-analysis/allocation-sites.txt`.
- Re-ran ShotMemoryAuditTest and PrecomputedShotsTest against this revision: passed.
  Warmed desktop measurement: 100,000 solveInto calls allocated 0 bytes; 100,000
  scores calls allocated 0 bytes; cache estimate remained 47,144 bytes; integrations=0.
  This is supporting dynamic evidence limited to the model, not a full process profile.
- No native memory audit or roboRIO heap dump was available. Fixed library graphs,
  actual log field count, packet sizes and queue depth remain unmeasured.
