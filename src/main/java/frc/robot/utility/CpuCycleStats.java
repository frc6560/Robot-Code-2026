package frc.robot.utility;

/** Main robot-thread cycle wall-time statistics with no per-cycle object allocation. */
final class CpuCycleStats {
    long cycles, workMicros, peakWorkMicros, peakIntervalMicros, overruns, totalOverruns;
    void record(long work, long period, long interval) {
        if (work < 0 || period <= 0) return;
        cycles++;
        workMicros += work;
        peakWorkMicros = Math.max(peakWorkMicros, work);
        if (interval >= 0) peakIntervalMicros = Math.max(peakIntervalMicros, interval);
        if (work > period) { overruns++; totalOverruns++; }
    }
    void clearWindow() { cycles = workMicros = peakWorkMicros = peakIntervalMicros = overruns = 0; }
}
