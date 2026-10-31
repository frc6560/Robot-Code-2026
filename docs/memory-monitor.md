# Reading the memory monitor

MemoryMonitor samples once per second in all robot modes. It publishes numeric
fields under Memory/ to the dashboard and AdvantageKit, and prints a distribution
line plus an actual-change line to the console. Hardware file/NT logging remains
2 Hz but these values refresh at 1 Hz. The console also works when log receivers
are backlogged. Sampling reads the controller's /proc, not the Windows laptop.

## Three separate views — do not add them together

1. System RAM: SystemTotalMiB, SystemFreeMiB, SystemAvailableMiB, raw Cached/Buffers/
   Shared/ReclaimableSlab/UnreclaimableSlab/KernelStacks/PageTables, plus swap.
   SystemTotal is kernel-reported usable RAM, which can be below the board's advertised RAM.
   MemAvailable is the kernel estimate of headroom without swapping; it is not
   identical to MemFree. The console additionally splits total into free,
   approximate reclaimable cache, and the residual other used category:
   cache estimate = clamp(Cached + Buffers + SReclaimable - Shmem, 0, Total - Free).
   Other estimate = Total - Free - cache estimate. These three derived buckets
   sum to total but are approximations, not exact allocation ownership. Shared/tmpfs
   pages are excluded from the estimated reclaimable cache. The other bucket
   includes anonymous/process memory, shared/tmpfs, kernel use and other accounting.
   Available is a separate kernel estimate; it is NOT a fourth additive bucket.
2. Robot-process RSS: ProcessResidentMiB, split by RssAnon, RssFile and RssShmem
   where supported. Anonymous memory includes Java heap and native allocations;
   file-backed pages include code/libraries and file mappings; shared covers resident
   shared-memory mappings. Values can be approximate on Linux and field snapshots
   can differ slightly. Missing older-kernel fields show unavailable. VmSize is
   virtual address space, NOT RAM. VmData is not a native-memory measurement.
   VmStk is the main-thread virtual stack mapping, not the sum of all thread stacks.
3. JVM accounting: heap used, committed and maximum, plus JVM-managed non-heap
   used/committed, tracked direct/mapped buffer allocations and Java thread count.
   Heap used + free committed = committed; committed + uncommitted limit = maximum
   when max is known. Headroom = maximum - used is Java heap-limit headroom, NOT
   controller RAM available. Non-heap covers VM-managed pools such as class metadata/
   code caches, not every native allocation. Buffer metrics report tracked allocation,
   not residency, and mapped capacity is not the physical mapped RAM footprint.

RSS, system cache and JVM usage overlap. Never sum all three views, subtract heap
used from RSS to call the difference native memory, or charge all system memory
outside the robot RSS to Linux (other processes and shared pages also contribute).
This monitor cannot attribute heap bytes to individual subsystems/classes or see
all native allocations. A heap/native-memory profile is needed for that.

## What is growing

Each of Heap, HeapCommitted, ProcessResident, ProcessAnonymous, ProcessFileBacked,
ProcessShared, SystemAvailable, NonHeap, DirectBuffers, MappedBuffers, SystemShared,
SystemUnreclaimableSlab and SystemOtherUsedEstimate has:

- Memory/Growth/<name>SinceStartMiB: change since its first valid sample.
- Memory/Growth/<name>WindowDeltaMiB: newest minus oldest retained valid sample.
- Memory/Growth/<name>MiBPerMinute: window delta / actual elapsed seconds * 60.
- Memory/Growth/<name>WindowSeconds: actual interval used for that category.

Trend storage is fixed: 61 samples per category, at most 60 seconds old. Missing
values clear the current window and publish NaN, not stale values or a fake zero.
A resumed source starts a fresh window; its session baseline is retained unless
the clock goes backward. Console shows actual delta and actual interval for each
category (for example 1.5MiB/59s). Rates in the dashboard/log are normalized rates,
not predictions, and are noisy during short startup windows. A decline in
SystemAvailable is pressure, while a positive RSS/heap delta is increasing usage.

Use several windows, not one increasing sample:

| Pattern | Interpretation to investigate |
|---|---|
| Heap repeatedly rises and falls after GC; RSS roughly stable | Normal allocation/collection may explain it; not proof of a leak |
| Heap window minimum and observations after GC keep rising | Java retention, an expanding working set, warmup or backlog |
| Queue depth/dropped count grows with heap | Receiver backlog; queue remains capped at eight on hardware |
| RSS/anonymous grows while sampled heap is stable | Native allocations, stacks, JVM commitment or another anonymous mapping; not a subsystem identification |
| NonHeap grows then levels off | Class loading/JIT warmup is possible; sustained growth needs inspection |
| Direct-buffer allocation/count rises persistently | Tracked direct buffers are a candidate; not all native memory is tracked here |
| File-backed RSS grows | File mappings/page residency, including loaded libraries; not automatically a leak |
| SystemShared grows while process RSS is stable | System shared/tmpfs use outside this process is a candidate |
| SystemUnreclaimableSlab grows | Kernel allocations are a candidate |
| SystemAvailable falls but robot RSS is stable | Other processes, system allocations or accounting changes need investigation |

## Garbage collection and logging

GCCount and GCCountSinceSample sum supported collector counters.
GCTimeMillisSinceSample is aggregate reported collection time over the interval,
not a precise measurement of the longest pause. Unsupported counters are NaN.
HeapPeakMiB is the largest sampled heap usage, not the exact between-sample peak.
HeapWindowMinimumMiB is the lowest one-second observation in the current window;
collections can be missed between samples. HeapUsedAtGcObservationMiB is sampled
heap usage when the monitor detects a new collection. It includes any allocations
since that collection and is NOT exact post-GC retained bytes. Its observation age
is published as HeapGcObservationAgeSeconds. No explicit GC is invoked.

LogQueueFrames, LogQueueCapacity, LogDroppedFrames and LogDroppedFramesSinceSample
show receiver pressure. A missing reading is NaN/n/a. ProcessProcReadOK and
SystemProcReadOK mean the source file was read successfully; individual optional
or malformed fields can still be unavailable. No failed read leaves stale old
numbers published as current.

## Correctness changes and overhead

JVM usage now comes from a single MemoryUsage snapshot instead of separate
Runtime.totalMemory/freeMemory calls that could straddle a collection. /proc
values require kB units and checked integer conversion to bytes; malformed fields
are isolated so one failure does not hide unrelated fields. Metrics and trend
buffers/keys are created during initialization. Readers and MemoryUsage snapshots,
line strings and the final console string still allocate at 1 Hz; this is a
monitoring cost, not an allocation-free profiler. Missing /proc on desktop is
handled without constructing repeated file-open exceptions. Trend storage stays
fixed with runtime. The monitor itself adds telemetry fields and log/console work.

## Sources

- Linux /proc memory definitions: https://cdn.kernel.org/doc/html/latest/filesystems/proc.html
- JVM heap/non-heap accounting: https://docs.oracle.com/en/java/javase/17/docs/api/java.management/java/lang/management/MemoryMXBean.html
- Buffer-pool estimates: https://docs.oracle.com/en/java/javase/17/docs/api/java.management/java/lang/management/BufferPoolMXBean.html
