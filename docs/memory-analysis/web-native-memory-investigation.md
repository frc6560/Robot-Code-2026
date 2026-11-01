# Native-memory crash investigation — 2026-10-31

## Conclusion and confidence

The photo proves that HotSpot failed a native malloc of 32,756 bytes for
ChunkPool::allocate. It does not identify which component consumed the budget.
The leading explanation is system/native-memory pressure or Linux commitment
limits on a roboRIO 1, rather than a growing collection of shot trajectories.
Strict commit accounting is a specific, documented candidate, not a measured
fact about this controller. Physical pressure, other processes, native-library
growth and process/address-space limits cannot yet be separated. The controller
was disconnected during this investigation. No claim that the crash is fixed,
or that a particular native library leaks, is justified yet.

Subsequent user observation: the logged free-memory reading declined every second
while teleop had never been enabled. The exact metric, decline rate and running
build are not yet established. This shows that a shooting-button press is not
required for the observed decline. It does not exclude startup allocations,
background native work, disabled subsystem periodic updates, file/cache growth,
or this branch's unconditional dashboard solver. A falling free-memory reading
alone cannot distinguish those cases from normal heap/cache growth.

Oracle's [native allocation failure guide](https://ops.java/performance/jvm/articles/native-mem-allocation-fail/)
distinguishes physical/swap shortage from process address-space exhaustion and
uses the full hs_err report to investigate. A Java heap can be small while a
native allocation still fails. GC handles unreachable Java objects; it cannot
reclaim arbitrary native allocations or RAM owned by other processes. Creating
temporary Java objects is allocation pressure, not by itself evidence of a leak.

## Most useful new finding

[WPILib's system-memory guidance](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-gc.html#system-memory-tuning)
documents the constrained RAM of roboRIO 1 and recommends considering
vm.overcommit_memory=1 instead of the device's documented default of 2.
Linux can refuse an allocation based on promised memory before the physical
pages have been used. This can explain a fatal allocation with apparently free
RAM. It is not evidence that this particular robot still uses that default.

The [kernel documentation](https://docs.kernel.org/mm/overcommit-accounting.html)
defines CommitLimit and Committed_AS as system-wide commitment accounting.
In mode 2, commitment is constrained by swap and configured RAM allowance.
The allowance comes from overcommit_ratio or a nonzero overcommit_kbytes.
Mode 1 changes admission policy; it does not create physical RAM and does not
repair a real leak. CommitLimit - Committed_AS is useful evidence, not a
guaranteed malloc allowance because reserves and other constraints also apply.

The [Chief Delphi Memory Leaks thread](https://www.chiefdelphi.com/t/memory-leaks/439549)
reports small native-allocation failures despite substantial MemAvailable and
a small Java heap. The author's final correction says disabling pre-touch alone
was insufficient with the web server enabled; allowing overcommit was also
needed. This is a closely related failure pattern, not proof of the same cause:
this checkout does not request AlwaysPreTouch and GradleRIO already uses Serial
GC. The thread also identifies console files on volatile storage as another
potential RAM consumer.

## Code/deployment evidence

- The upstream default main at f010577a01b82ee63563bee052c905fa23d9a701 has
  Turret and Feeder, matching the photo. This checkout's tidle_tumble branch does
  not. Exact running-build identity is still unverified.
- That main still starts WPILOGWriter and the normal AdvantageKit queue and has
  no MemoryMonitor or this branch's 2 Hz snapshot policy. Downloading the default
  archive does not include the fixes on this branch. A full GradleRIO deploy of
  that archive would still deploy the older logging policy.
- Main's ShotCalculator has three fixed nine-entry interpolation maps. Its
  moving-target correction is bounded (20 iterations), creates temporary
  geometry, and replaces current state. No append-only trajectory history was
  found. Its SuperstructureCommand is a default command without a
  runsWhenDisabled override; it does not execute while disabled. The photo
  includes disabledPeriodic overruns, which further weakens the claim that an
  actively running shot solver is required to trigger this failure.
- This checkout's separate precomputed solver loads a fixed table. The allocation
  audit repeatedly exercises 100,000 solves and 100,000 score checks without
  growing the cache or running numerical flight integrations. This only tests
  the solver path, not all robot/native-library behavior.
- The hs_err/replay output follows the fatal allocation failure. These are
  JVM crash diagnostics; their existence does not establish that normal log
  recording caused the crash. Current branch flags disable replay-file creation
  and send the fatal report directly to stderr. The NI console capture remains
  outside AdvantageKit's file-logging policy.

## Ranked next investigations

| Candidate | Supporting evidence | Observation that separates it |
|---|---|---|
| Native/system budget or strict commit limit | Actual fatal native malloc; constrained controller; documented failure pattern | Overcommit mode plus CommitLimit/Committed_AS, MemAvailable and full fatal report |
| Running source lacks fixes | Subsystem names match default main; main still has file receiver and original queue | Startup build identifier and JVM arguments from the controller |
| Console/tmpfs growth or another process | Repeated pose/CAN/overrun output; volatile files can consume RAM | Filesystem growth, Shmem, and process RSS snapshots over time |
| Robot native-library/JVM growth | Native crash can occur with stable heap | Rising process RSS/anonymous pages with stable heap; full report/maps and subsequent targeted native profiling |
| Java retention/backlog in deployed build | Original receiver queue can retain up to 500 snapshots | Sustained rising heap floor after collections and receiver depth; class-retention profile if indicated |
| Growing shot cache | No such growth found in either inspected shooter implementation | Current solver allocation/cache audit; verify actual deployed source |

These are investigation priorities, not measured byte distributions or assigned
probabilities. The photo alone cannot establish a leak owner or leak rate.

## Other web reports checked against this code

- [2026 YAGSL report](https://www.chiefdelphi.com/t/excessive-ram-usage-on-roborio-causing-low-responsiveness-and-data-loss/512263):
  the reporter improved behavior by removing duplicate odometry updates. No
  corresponding extra explicit updateOdometry call was found in inspected main.
- [roboRIO native OOM discussion](https://www.chiefdelphi.com/t/roborio-out-of-memory/495564):
  similar controller-memory failures exist without an obvious desktop Java leak;
  this does not identify our library or justify enabling swap blindly.
- [CTRE changelog](https://api.ctr-electronics.com/changelog.html): Phoenix 26.1.2
  fixes unconditional SignalLogger startup on roboRIO 1 and a crash when deleting
  hoot logs on full storage. Both inspected branches specify 26.1.2. That known
  older-version bug is therefore not enough to explain this crash, assuming the
  matching JNI libraries were actually deployed.

## Changes made for decisive evidence

MemoryMonitor now reads CommitLimit, Committed_AS, overcommit mode, ratio and
kbytes every second without modifying Linux. Console output includes signed
commit headroom; NT includes the raw fields and commitment growth. This closes
an actual gap in the previous monitor: physical RAM and Java heap alone could
not detect strict commitment pressure. Missing readings remain unavailable.

tools/capture-rio-memory.sh streams a read-only snapshot from the controller:
memory, policy, process RSS, Java limits/command line, filesystems and recent
kernel messages. It writes no files on the controller. Run it while the robot
program is alive; capture a second sample later to distinguish growth from a
large fixed footprint. On Windows PowerShell, from the project directory:

```powershell
Get-Content -Raw tools/capture-rio-memory.sh | ssh admin@roborio-TEAM-frc.local sh | Set-Content rio-memory-before.txt
```

Replace TEAM with the team number; output is saved on the laptop. Preserve the
existing hs_err report before rebooting if one still exists. If the current
stderr-only policy is deployed, save the full fatal console output externally.

If mode is 2 and commitment approaches its limit while physical availability
remains healthy, the targeted next experiment is WPILib's documented temporary
setting, `sysctl -w vm.overcommit_memory=1`, followed by the same workload and
memory measurements. That requires an admin connection; it was not executed.
If mode is already 1, or physical RAM keeps declining, that hypothesis loses
support and the process/tmpfs/native-growth readings become the next lead.
Do not persist policy changes or reimage the controller before collecting this
evidence; neither would identify a retained allocation.

## Validation

Java 17 compilation and MemoryMonitor/MemoryTrend regression tests pass,
including strict commitment exhaustion with 64 MiB physically available,
negative headroom, and missing/malformed sysctl readings. This is synthetic
accounting input, not a reproduction of the roboRIO crash. No live controller
measurement or hardware validation was possible while disconnected.

The Java 17 shooter audit initially counted 168 bytes across 100,000 readiness
checks because that path was not warmed before measuring. The audit now warms
both measured paths and retains its strict zero-allocation assertions. It passes:
100,000 solves allocated 0 measured Java bytes, 100,000 readiness checks allocated
0, cache estimate remained 47,144 bytes, and flight integrations remained 0.
This establishes the tested steady-state result, not allocation-free startup,
all robot code, or zero native/JIT memory growth.
