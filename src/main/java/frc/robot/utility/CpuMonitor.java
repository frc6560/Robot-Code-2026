package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.nio.file.Files;
import java.nio.file.Path;

/** One-second CPU snapshots and trends; cycle recording uses primitive counters only. */
public final class CpuMonitor {
    private static final ThreadMXBean THREADS = ManagementFactory.getThreadMXBean();
    private static final boolean THREAD_CPU_ENABLED = enableThreadCpuTime();
    private static final ProcessHandle PROCESS = ProcessHandle.current();
    private static final Path PROC_STAT = Path.of("/proc/stat");
    private static final CpuTimeCounter process = new CpuTimeCounter(), mainThread = new CpuTimeCounter();
    private static final CpuTicks system = new CpuTicks();
    // Avoid hundreds of diagnostic fields on desktop machines. Both roboRIO cores are included.
    private static final CpuTicks[] cores = new CpuTicks[Math.min(8, Runtime.getRuntime().availableProcessors())];
    private static final String[] coreKeys = new String[cores.length];
    private static final MemoryTrend processTrend = new MemoryTrend(), systemTrend = new MemoryTrend();
    private static final StringBuilder console = new StringBuilder(768);
    private static double nextSample, lastSample = Double.NaN;
    private static final CpuCycleStats cycleStats = new CpuCycleStats();
    static {
        for (int i = 0; i < cores.length; i++) {
            cores[i] = new CpuTicks();
            coreKeys[i] = "CPU/Core" + i + "BusyPercent";
        }
    }
    private CpuMonitor() {}

    private static boolean enableThreadCpuTime() {
        try {
            if (!THREADS.isCurrentThreadCpuTimeSupported()) return false;
            if (!THREADS.isThreadCpuTimeEnabled()) THREADS.setThreadCpuTimeEnabled(true);
            return true;
        } catch (UnsupportedOperationException | SecurityException ignored) { return false; }
    }

    /** Robot-thread only. Counts wall work (including capture), not notifier sleep. */
    public static void recordCycle(long work, long period, long interval) {
        cycleStats.record(work, period, interval);
    }

    public static void update() {
        double now = Timer.getFPGATimestamp();
        if (now >= lastSample && now < nextSample) return;
        lastSample = now;
        nextSample = now + 1;
        int processors = Runtime.getRuntime().availableProcessors();
        long processWall = System.nanoTime();
        long processCpu = -1;
        try {
            var duration = PROCESS.info().totalCpuDuration();
            if (duration.isPresent()) processCpu = duration.get().toNanos();
        } catch (UnsupportedOperationException | SecurityException ignored) { /* Unavailable, not zero. */ }
        process.sample(processWall, processCpu);
        long threadWall = System.nanoTime();
        mainThread.sample(threadWall, THREAD_CPU_ENABLED ? THREADS.getCurrentThreadCpuTime() : -1);
        double processPercent = process.machinePercent(processors);
        boolean procOK = readSystem(PROC_STAT, system, cores);
        number("CPU/LogicalProcessors", processors);
        number("CPU/ProcessMachinePercent", processPercent);
        number("CPU/ProcessOneCorePercent", process.oneCorePercent);
        number("CPU/MainThreadOneCorePercent", mainThread.oneCorePercent);
        flag("CPU/SystemProcReadOK", procOK);
        number("CPU/SystemBusyPercent", system.busy);
        number("CPU/SystemUserPercent", system.user);
        number("CPU/SystemKernelPercent", system.kernel);
        number("CPU/SystemIdlePercent", system.idle);
        number("CPU/SystemIoWaitPercent", system.ioWait);
        number("CPU/SystemStealPercent", system.steal);
        for (int i = 0; i < cores.length; i++) number(coreKeys[i], cores[i].busy);
        double load = ManagementFactory.getOperatingSystemMXBean().getSystemLoadAverage();
        number("CPU/SystemLoadAverage1m", load >= 0 ? load : Double.NaN);
        processTrend.add(now, processPercent);
        systemTrend.add(now, system.busy);
        number("CPU/ProcessWindowChangePercentagePoints", processTrend.delta());
        number("CPU/ProcessChangePercentagePointsPerMinute", processTrend.changePerMinute());
        number("CPU/ProcessWindowSeconds", processTrend.seconds());
        number("CPU/SystemWindowChangePercentagePoints", systemTrend.delta());
        number("CPU/SystemChangePercentagePointsPerMinute", systemTrend.changePerMinute());
        number("CPU/SystemWindowSeconds", systemTrend.seconds());
        double meanWork = cycleStats.cycles > 0 ? (double) cycleStats.workMicros / cycleStats.cycles / 1000 : Double.NaN;
        double peakWork = cycleStats.cycles > 0 ? cycleStats.peakWorkMicros / 1000.0 : Double.NaN;
        double peakInterval = cycleStats.cycles > 0 && cycleStats.peakIntervalMicros > 0 ? cycleStats.peakIntervalMicros / 1000.0 : Double.NaN;
        number("CPU/CycleWorkMeanMillis", meanWork);
        number("CPU/CycleWorkPeakMillis", peakWork);
        number("CPU/CycleIntervalPeakMillis", peakInterval);
        number("CPU/WorkOverrunsSinceSample", cycleStats.overruns);
        number("CPU/WorkOverrunsTotal", cycleStats.totalOverruns);
        number("CPU/CyclesSinceSample", cycleStats.cycles);
        printSummary(processPercent, meanWork, peakWork, peakInterval);
        cycleStats.clearWindow();
    }

    static boolean readSystem(Path path, CpuTicks aggregate, CpuTicks[] perCore) {
        aggregate.seen = false;
        for (CpuTicks core : perCore) core.seen = false;
        try {
            if (!Files.isReadable(path)) { reset(aggregate, perCore); return false; }
            try (var reader = Files.newBufferedReader(path)) {
                String line;
                while ((line = reader.readLine()) != null && line.startsWith("cpu")) {
                    CpuTicks target = null;
                    if (line.length() > 3 && Character.isWhitespace(line.charAt(3))) target = aggregate;
                    else {
                        int i = 3, index = 0;
                        while (i < line.length() && line.charAt(i) >= '0' && line.charAt(i) <= '9') {
                            index = index * 10 + line.charAt(i++) - '0';
                            if (index >= perCore.length) break;
                        }
                        if (i > 3 && i < line.length() && Character.isWhitespace(line.charAt(i)) && index < perCore.length)
                            target = perCore[index];
                    }
                    if (target != null) {
                        if (CpuTicks.parse(line, target.incoming)) target.accept();
                        else target.reset();
                    }
                }
            }
            if (!aggregate.seen) aggregate.reset();
            for (CpuTicks core : perCore) if (!core.seen) core.reset();
            return aggregate.seen;
        } catch (IOException | SecurityException ignored) { reset(aggregate, perCore); return false; }
    }
    private static void reset(CpuTicks aggregate, CpuTicks[] perCore) {
        aggregate.reset();
        for (CpuTicks core : perCore) core.reset();
    }
    private static void number(String key, double value) {
        RateLimitedTelemetry.putNumber(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void flag(String key, boolean value) {
        RateLimitedTelemetry.putBoolean(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void printSummary(double processPercent, double mean, double peak, double interval) {
        console.setLength(0);
        console.append("[CPU] system busy="); append(system.busy); console.append("% (user/kernel/idle/io-wait=");
        append(system.user); console.append('/'); append(system.kernel); console.append('/'); append(system.idle);
        console.append('/'); append(system.ioWait); console.append("%); robot="); append(processPercent);
        console.append("% machine / "); append(process.oneCorePercent); console.append("% one-core; main thread=");
        append(mainThread.oneCorePercent); console.append("% one-core; cycle mean/peak="); append(mean);
        console.append('/'); append(peak); console.append("ms; peak interval="); append(interval);
        console.append("ms; work overruns=").append(cycleStats.overruns).append(" total=").append(cycleStats.totalOverruns);
        console.append("; rolling change robot/system="); appendChange(processTrend);
        console.append(" / "); appendChange(systemTrend);
        System.out.println(console.toString());
    }
    private static void appendChange(MemoryTrend trend) {
        if (!Double.isFinite(trend.delta())) { console.append("n/a"); return; }
        append(trend.delta());
        console.append("pp/").append((long) trend.seconds()).append('s');
    }
    private static void append(double value) {
        if (!Double.isFinite(value)) { console.append("n/a"); return; }
        long tenths = Math.round(value * 10);
        if (tenths < 0) console.append('-');
        long absolute = Math.abs(tenths);
        console.append(absolute / 10).append('.').append(absolute % 10);
    }
}
