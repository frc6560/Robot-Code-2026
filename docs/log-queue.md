# Production log queue

## What is queued

An AdvantageKit log frame is a snapshot of the current table. recordOutput and
processInputs update fields; they do not enqueue one event per call. A frame
contains application readings plus built-in Driver Station, system/conduit,
performance, alerts, radio and captured console data. SampledLoggedRobot invokes
the hardware capture hooks at most once per 500 ms, alongside the 50 Hz control
loop. Startup and exception capture are additional frames.

The receiver thread drains frames and calls file/NT receivers sequentially. A slow
file writer can therefore delay live AdvantageScope delivery too. If robot code
stalls, it produces no frames during that stall. If a receiver stalls while control
continues, frames accumulate. The 2 Hz gate is a timestamp check, not Thread.sleep;
it schedules the next capture relative to now, so it does not create catch-up bursts.

## Capacity and loss policy

- Real hardware: eight queued frames, roughly four seconds at 2 Hz.
- Desktop simulation/replay: original 500-slot capacity and overflow behavior.
- Hardware saturation: evict the oldest queued frame, admit the newest snapshot.
  No wait for space or receiver I/O and no warning/stacktrace storm on overflow.
- The eight-frame limit excludes the receiver's in-flight frame, a snapshot under
  construction, the current log table and receivers' last-table references. It is
  a frame-count limit, not a total process byte limit. Oversized payloads still matter.
- Losing intermediate frames affects diagnostic history, not control outputs.
  Retained frame maps are independent; they do not reference a mutable live table.

Relative to a full 500-frame backlog, at most 1.6% as many frames are queued (62.5x
fewer). New snapshots still allocate at 2 Hz, and old snapshots become eligible
for GC when evicted. This does not fix native-memory leaks or eliminate every log
allocation. At a hypothetical 500 fields/frame, cloned map structure is roughly
0.15 MiB for eight frames versus 9.5 MiB for 500, excluding retained payloads.

## Observability

Recorded fields: Logger/QueuedCycles, Logger/QueueCapacity, Logger/DroppedFrames.
Queue metrics in a frame reflect state before that frame's own enqueue, so a new
eviction is visible in the next sample. getReceiverQueueFault indicates whether
the last hardware capture evicted an old frame, rather than dropping the new one.

The dashboard and one-second console memory summary also show current queue depth,
capacity and cumulative drops. These remain useful when the log receiver is stalled.
Console transport itself can still be delayed by a separate communication problem.

## Reducing work further

Hardware frame admission is already limited before cloning/capture to 2 Hz. We
preserve that rate and AdvantageScope/file logging. Application latest-value
caches merge suppressed updates instead of retaining a sensor-event history.
Reducing recordOutput call count alone does not reduce frame count. To reduce
bytes per frame, remove redundant fields or large payloads; to reduce frames,
lower the capture rate or skip selected captures. When a receiver is blocked,
keeping the bounded newest window is more useful than accumulating minutes of
old snapshots. BLine's internal Pair/history-array generation is a separate
producer-side allocation issue, not additional AdvantageKit queue entries.

## Dependency adaptation and packaging

The installed AdvantageKit 26.0.2 has a private hard-coded 500-slot queue and no
public capacity setter. Logger.java is a local BSD-licensed adaptation of the
26.0.2 source, retaining its public API and upstream copyright. Changes are confined
to queue construction, hardware overflow handling and queue observability methods.
LatestFrameQueue implements the single-producer newest-frame policy.

The build rejects an AdvantageKit version upgrade until this adaptation is reviewed.
Dependency Logger.class and Logger$*.class entries are excluded from the fat JAR;
our compiled versions are included normally. Do not replace the queue by reflection
or edit a machine-local dependency cache. The upstream-relative diff is included
in memory-analysis/logger-queue-26.0.2.patch for review.

## Validation

Tests cover 10,000 saturated offers with an eight-frame cap, a blocked actual
ReceiverThread with 1,000 additional frames and recovery order, snapshot isolation,
and desktop patched-API/capacity compatibility. Existing rate-limit, memory and
Limelight guard tests are run alongside these. Artifact inspection verifies that
exactly one Logger.class and its nested AdvancedHooks class are deployed.
