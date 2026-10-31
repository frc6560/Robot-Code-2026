# CPU and /tmp filesystem monitoring

Both monitors run once per second from robotPeriodic, alongside MemoryMonitor.
They print console summaries and publish scalar fields to the dashboard and
AdvantageKit. Hardware log capture remains 2 Hz; these readings refresh at 1 Hz.

## CPU

AdvantageKit already records CPU temperature and sampled framework timings. The
new CPU/ fields add utilization, trends and timing statistics over every control
cycle rather than only the sampled logging cycles.

- ProcessMachinePercent: change in total robot process CPU time / monotonic elapsed
  time / JVM-visible logical processor count. 100% means the whole available
  machine capacity. Includes Java, native vendor code, logging and odometry threads.
- ProcessOneCorePercent: the same process CPU time without dividing by processor
  count. 100% is one fully occupied logical core; 200% is two. The normalized
  machine field is clamped to 100% for sampling/processor-count changes; raw one-core
  load is also available. ProcessHandle CPU duration is optional; unsupported
  readings are NaN, not zero. First sample establishes a baseline.
- MainThreadOneCorePercent: CPU time used by the calling robot thread, divided by
  elapsed time. Excludes background threads and time spent blocked/asleep. Standard
  ThreadMXBean CPU timing is enabled once if supported.
- SystemBusyPercent and Core0BusyPercent etc.: Linux /proc/stat tick deltas. Up to
  eight logical cores are monitored to bound telemetry on large desktop hosts;
  the roboRIO's cores are covered. System user includes nice; kernel includes IRQ
  and softIRQ. Idle, I/O-wait and steal are separate fields. Busy=user+kernel+steal;
  optional guest fields are not added again because they are included in user/nice.
  I/O-wait is a kernel estimate with limitations; a counter rollback invalidates
  the interval and reestablishes a baseline. No USER_HZ assumption is needed for
  tick ratios. Missing/invalid sources never leave stale utilization values.
- SystemLoadAverage1m is an OS load-average metric, NOT a percentage. It may include
  runnable/waiting tasks. System totals may describe an entire host rather than a
  container quota; the dedicated roboRIO does not have that desktop ambiguity.
- Process/System WindowChangePercentagePoints, ChangePercentagePointsPerMinute,
  and WindowSeconds show the actual change between endpoint samples retained over
  at most 60 seconds. Storage is fixed. They are not cumulative CPU consumption
  or predictions. Warmup and short windows can have noisy rates.

CycleWorkMeanMillis and CycleWorkPeakMillis measure wall time from before robot
callbacks through logging capture, excluding notifier sleep. CpuCycleStats records
primitive counters at 50 Hz without constructing objects per cycle. The one-second
summary resets interval counts/peaks, preserving WorkOverrunsTotal. A work overrun
means wall work strictly exceeded the configured 20 ms period. CycleIntervalPeakMillis
also detects delayed starts, including scheduler starvation; it is not the same as
work time. The current cycle containing the monitor report enters the next window.

Interpretation:

| Observation | Candidate to inspect |
|---|---|
| High process CPU and high main-thread CPU | Robot control, callbacks, logging capture or diagnostics |
| High process CPU but modest main-thread CPU | Background odometry, receivers, GC or native worker threads |
| High system load but modest robot process CPU | Other processes/kernel work, or different sampling denominators |
| Long cycle work with low thread CPU | Blocking I/O, locks, waiting or scheduling rather than pure computation |
| One core saturated but moderate machine percentage | A single-thread bottleneck; averaging cores hides it |

CPU observations identify categories, not which individual subsystem/function is
responsible. The monitor itself consumes CPU and allocates once-per-second reader,
ProcessHandle.Info and console objects. No external shell command or per-thread
scan is run. On macOS/Windows, Linux system/core counters show n/a while supported
process and thread CPU measurements remain available.

## /tmp equivalent of df -h /tmp

StorageMonitor queries the FileStore containing /tmp using Java's filesystem APIs.
It reports Storage/Tmp/TotalMiB, UsedMiB, FreeMiB, AvailableMiB, UsedPercent,
FileSystemType, FileStoreName, ReadOK and KnownRamBackedType. Console sizes use
binary KiB/MiB/GiB. The reported numbers describe the filesystem containing /tmp,
NOT the recursive size of just /tmp and not the JVM heap.

Used = total - unallocated free. Available = usable space reported to this process.
The percentage denominator is used+available, matching df-style treatment of
reserved space; human-readable rounding can differ from the df command's integer
Use%. Statistics are not one atomic snapshot, and quotas/provider behavior can
also affect comparisons. Zero/unknown capacity produces n/a percentage rather
than divide-by-zero or a false 0%.

KnownRamBackedType is true for tmpfs or ramfs. On tmpfs, the displayed total is a
filesystem limit, not RAM preallocated at mount. A large reported available space
does not guarantee equivalent free physical RAM. Watch Memory/SystemAvailableMiB
and Memory/SystemSharedMiB too. Other types (including overlay) are not claimed to
be definitely disk-backed; the flag only recognizes explicit RAM-backed types.
The filesystem can fill from other directories sharing the mount. A simulation
run describes the simulation host's /tmp, not the real roboRIO.

UsedSinceStartMiB, UsedWindowChangeMiB, AvailableWindowChangeMiB, UsedMiBPerMinute
and WindowSeconds track growth with fixed storage. Changes can also come from
other writers, mount changes/resizing or quota changes; they do not identify a file
or prove a leak. No directory walk is performed. Failed reads publish NaN/n/a and
clear the window instead of retaining stale space readings. The runtime uses no
external `df` process. Java FileStore calls and console output have small periodic
allocation/I/O costs.

## Validation and sources

Tests cover process normalization, first/missing/reset samples, Linux CPU
percentages and guest exclusion, malformed counters, missing cores/files, cycle
peak/overrun accounting, df-style reserved capacity and RAM-backed classification.
Headless simulation checks integration with the existing robot loop and console.
Linux readings need live roboRIO verification.

Primary references:
- https://kernel.org/doc/html/v5.19/admin-guide/cpu-load.html
- https://kernel.org/doc/html/v5.16/filesystems/proc.html
- https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/lang/ProcessHandle.Info.html
- https://docs.oracle.com/en/java/javase/17/docs/api/java.management/java/lang/management/ThreadMXBean.html
- https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/nio/file/FileStore.html
