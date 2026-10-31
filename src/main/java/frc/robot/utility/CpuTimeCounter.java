package frc.robot.utility;

/** Deltas of cumulative process/thread CPU time, independent of sampling jitter. */
final class CpuTimeCounter {
    private boolean initialized;
    private long lastWall, lastCpu;
    double oneCorePercent = Double.NaN;

    void sample(long wallNanos, long cpuNanos) {
        oneCorePercent = Double.NaN;
        if (cpuNanos < 0) { initialized = false; return; }
        if (initialized && wallNanos > lastWall && cpuNanos >= lastCpu)
            oneCorePercent = 100.0 * (cpuNanos - lastCpu) / (wallNanos - lastWall);
        lastWall = wallNanos;
        lastCpu = cpuNanos;
        initialized = true;
    }
    double machinePercent(int processors) {
        return processors > 0 && Double.isFinite(oneCorePercent)
            ? Math.min(100, oneCorePercent / processors) : Double.NaN;
    }
}
