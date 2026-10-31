package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import java.io.IOException;
import java.lang.management.BufferPoolMXBean;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/** One-second memory snapshots and fixed-storage trends; never forces garbage collection. */
public final class MemoryMonitor {
    private static final double MIB = 1048576.0;
    private static final MemoryMXBean JVM = ManagementFactory.getMemoryMXBean();
    private static final java.lang.management.ThreadMXBean THREADS = ManagementFactory.getThreadMXBean();
    private static final GarbageCollectorMXBean[] COLLECTORS = ManagementFactory.getGarbageCollectorMXBeans()
        .toArray(GarbageCollectorMXBean[]::new);
    private static final BufferPoolMXBean[] BUFFER_POOLS = ManagementFactory.getPlatformMXBeans(BufferPoolMXBean.class)
        .toArray(BufferPoolMXBean[]::new);
    // /proc units are KiB. Optional fields can be absent on older kernels.
    private static final String[] PROCESS_FIELDS = {"VmRSS:", "VmSize:", "VmData:", "RssAnon:", "RssFile:",
        "RssShmem:", "VmSwap:", "VmHWM:", "VmStk:"};
    private static final String[] PROCESS_NAMES = {"Memory/ProcessResidentMiB", "Memory/ProcessVirtualMiB",
        "Memory/ProcessDataMiB", "Memory/ProcessAnonymousMiB", "Memory/ProcessFileBackedMiB",
        "Memory/ProcessSharedMiB", "Memory/ProcessSwapMiB", "Memory/ProcessResidentPeakMiB", "Memory/MainThreadStackVirtualMiB"};
    private static final String[] SYSTEM_FIELDS = {"MemTotal:", "MemFree:", "MemAvailable:", "Cached:", "Buffers:",
        "Shmem:", "SReclaimable:", "SUnreclaim:", "KernelStack:", "PageTables:", "SwapTotal:", "SwapFree:"};
    private static final String[] SYSTEM_NAMES = {"Memory/SystemTotalMiB", "Memory/SystemFreeMiB",
        "Memory/SystemAvailableMiB", "Memory/SystemCachedMiB", "Memory/SystemBuffersMiB",
        "Memory/SystemSharedMiB", "Memory/SystemReclaimableSlabMiB", "Memory/SystemUnreclaimableSlabMiB",
        "Memory/SystemKernelStacksMiB", "Memory/SystemPageTablesMiB", "Memory/SystemSwapTotalMiB", "Memory/SystemSwapFreeMiB"};
    private static final Path PROCESS_STATUS = Path.of("/proc/self/status"), SYSTEM_STATUS = Path.of("/proc/meminfo");
    private static final long[] processBytes = new long[PROCESS_FIELDS.length], systemBytes = new long[SYSTEM_FIELDS.length];
    private static final String[] TREND_NAMES = {"Heap", "ProcessResident", "ProcessAnonymous", "ProcessFileBacked",
        "ProcessShared", "SystemAvailable", "NonHeap", "DirectBuffers", "SystemShared",
        "SystemUnreclaimableSlab", "HeapCommitted", "MappedBuffers", "SystemOtherUsedEstimate"};
    private static final MemoryTrend[] trends = new MemoryTrend[TREND_NAMES.length];
    private static final String[][] trendKeys = new String[TREND_NAMES.length][4];
    private static final StringBuilder consoleLine = new StringBuilder(1024);
    private static double nextSample, lastSample = Double.NaN;
    private static long peak, lastGcCount = -1, lastGcMillis = -1, lastDropped = -1;
    private static double heapAtGcObservation = Double.NaN, gcObservationTime = Double.NaN;
    static {
        for (int i = 0; i < trends.length; i++) {
            trends[i] = new MemoryTrend();
            trendKeys[i][0] = "Memory/Growth/" + TREND_NAMES[i] + "SinceStartMiB";
            trendKeys[i][1] = "Memory/Growth/" + TREND_NAMES[i] + "WindowDeltaMiB";
            trendKeys[i][2] = "Memory/Growth/" + TREND_NAMES[i] + "MiBPerMinute";
            trendKeys[i][3] = "Memory/Growth/" + TREND_NAMES[i] + "WindowSeconds";
        }
    }
    private MemoryMonitor() {}

    public static void update() {
        double now = Timer.getFPGATimestamp();
        if (now >= lastSample && now < nextSample) return;
        lastSample = now;
        nextSample = now + 1;
        // Use a single coherent JVM snapshot, rather than separate total/free calls across a GC.
        var heap = JVM.getHeapMemoryUsage();
        var nonHeap = JVM.getNonHeapMemoryUsage();
        long used = heap.getUsed(), committed = heap.getCommitted(), max = heap.getMax();
        peak = Math.max(peak, used);
        publishBytes("Memory/HeapUsedMiB", used);
        publishBytes("Memory/HeapCommittedMiB", committed);
        publishBytes("Memory/HeapMaxMiB", max);
        publishBytes("Memory/HeapHeadroomMiB", max >= 0 ? max - used : Double.NaN);
        publishBytes("Memory/HeapFreeCommittedMiB", committed - used);
        publishBytes("Memory/HeapUncommittedLimitMiB", max >= 0 ? max - committed : Double.NaN);
        publishBytes("Memory/HeapPeakMiB", peak);
        publishBytes("Memory/NonHeapUsedMiB", nonHeap.getUsed());
        publishBytes("Memory/NonHeapCommittedMiB", nonHeap.getCommitted());
        number("Memory/JavaThreadCount", THREADS.getThreadCount());

        boolean processOK = readProc(PROCESS_STATUS, PROCESS_FIELDS, processBytes);
        boolean systemOK = readProc(SYSTEM_STATUS, SYSTEM_FIELDS, systemBytes);
        publishProc(PROCESS_NAMES, processBytes);
        publishProc(SYSTEM_NAMES, systemBytes);
        flag("Memory/ProcessProcReadOK", processOK);
        flag("Memory/SystemProcReadOK", systemOK);
        double cache = reclaimableEstimate(systemBytes);
        publishBytes("Memory/SystemReclaimableCacheEstimateMiB", cache);
        double otherUsed = Double.isFinite(cache) ? systemBytes[0] - systemBytes[1] - cache : Double.NaN;
        publishBytes("Memory/SystemOtherUsedEstimateMiB", otherUsed);
        number("Memory/SystemAvailablePercent", systemBytes[0] > 0 && systemBytes[2] >= 0
            && systemBytes[2] <= systemBytes[0] ? 100.0 * systemBytes[2] / systemBytes[0] : Double.NaN);

        double direct = Double.NaN, mapped = Double.NaN, directCount = Double.NaN, mappedCount = Double.NaN;
        for (BufferPoolMXBean pool : BUFFER_POOLS) {
            if (pool.getName().equals("direct")) {
                direct = pool.getMemoryUsed(); directCount = pool.getCount();
            } else if (pool.getName().equals("mapped")) {
                mapped = pool.getMemoryUsed(); mappedCount = pool.getCount();
            }
        }
        publishBytes("Memory/DirectBufferAllocatedMiB", direct);
        publishBytes("Memory/MappedBufferAllocatedMiB", mapped);
        number("Memory/DirectBufferCount", directCount >= 0 ? directCount : Double.NaN);
        number("Memory/MappedBufferCount", mappedCount >= 0 ? mappedCount : Double.NaN);
        updateGc(now, used);
        trend(0, now, used);
        trend(1, now, processBytes[0]);
        trend(2, now, processBytes[3]);
        trend(3, now, processBytes[4]);
        trend(4, now, processBytes[5]);
        trend(5, now, systemBytes[2]);
        trend(6, now, nonHeap.getUsed());
        trend(7, now, direct);
        trend(8, now, systemBytes[5]);
        trend(9, now, systemBytes[7]);
        trend(10, now, committed);
        trend(11, now, mapped);
        trend(12, now, otherUsed);
        publishBytes("Memory/HeapWindowMinimumMiB", trends[0].minimum());
        number("Memory/Growth/WindowSeconds", trends[0].seconds());

        int queued = Logger.getReceiverQueueSize(), capacity = Logger.getReceiverQueueCapacity();
        long dropped = Logger.getDroppedFrameCount();
        number("Memory/LogQueueFrames", queued);
        number("Memory/LogQueueCapacity", capacity);
        number("Memory/LogDroppedFrames", dropped);
        number("Memory/LogDroppedFramesSinceSample", lastDropped >= 0 && dropped >= lastDropped ? dropped - lastDropped : 0);
        lastDropped = dropped;
        printSummary(used, committed, max, nonHeap.getUsed(), cache, otherUsed, queued, capacity, dropped);
    }

    private static void updateGc(double now, long used) {
        long count = 0, millis = 0;
        boolean countKnown = false, timeKnown = false;
        for (GarbageCollectorMXBean gc : COLLECTORS) {
            long c = gc.getCollectionCount(), t = gc.getCollectionTime();
            if (c >= 0) { count += c; countKnown = true; }
            if (t >= 0) { millis += t; timeKnown = true; }
        }
        double delta = countKnown && lastGcCount >= 0 && count >= lastGcCount ? count - lastGcCount : Double.NaN;
        number("Memory/GCCount", countKnown ? count : Double.NaN);
        number("Memory/GCCountSinceSample", delta);
        number("Memory/GCTimeMillisSinceSample", timeKnown && lastGcMillis >= 0 && millis >= lastGcMillis
            ? millis - lastGcMillis : Double.NaN);
        // A sampled observation after a detected GC, NOT exact retained/live bytes at GC completion.
        if (delta > 0) { heapAtGcObservation = used; gcObservationTime = now; }
        publishBytes("Memory/HeapUsedAtGcObservationMiB", heapAtGcObservation);
        number("Memory/HeapGcObservationAgeSeconds", now >= gcObservationTime ? now - gcObservationTime : Double.NaN);
        lastGcCount = countKnown ? count : -1;
        lastGcMillis = timeKnown ? millis : -1;
    }

    private static void trend(int i, double now, double bytes) {
        trends[i].add(now, bytes);
        publishSignedBytes(trendKeys[i][0], trends[i].sinceStart());
        publishSignedBytes(trendKeys[i][1], trends[i].delta());
        publishSignedBytes(trendKeys[i][2], trends[i].bytesPerMinute());
        number(trendKeys[i][3], trends[i].seconds());
    }
    private static void publishProc(String[] names, long[] bytes) {
        for (int i = 0; i < names.length; i++) publishBytes(names[i], bytes[i]);
    }
    private static void flag(String key, boolean value) {
        RateLimitedTelemetry.putBoolean(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void number(String key, double value) {
        RateLimitedTelemetry.putNumber(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void publishBytes(String key, double bytes) { number(key, bytes >= 0 ? bytes / MIB : Double.NaN); }
    private static void publishSignedBytes(String key, double bytes) { number(key, bytes / MIB); }

    static boolean readProc(Path path, String[] fields, long[] bytes) {
        Arrays.fill(bytes, -1);
        try {
            // Missing /proc is normal on desktop; avoid constructing exceptions every sample.
            if (!Files.isReadable(path)) return false;
            try (var reader = Files.newBufferedReader(path)) {
                String line;
                while ((line = reader.readLine()) != null) {
                    for (int i = 0; i < fields.length; i++) {
                        if (line.startsWith(fields[i])) {
                            try { bytes[i] = parseProcBytes(line); }
                            catch (NumberFormatException ignored) { bytes[i] = -1; }
                        }
                    }
                }
            }
            return true;
        } catch (IOException | SecurityException ignored) {
            Arrays.fill(bytes, -1);
            return false;
        }
    }

    static long parseProcBytes(String line) {
        int colon = line.indexOf(':');
        if (colon < 0) throw new NumberFormatException("Missing field separator");
        int i = colon + 1;
        while (i < line.length() && Character.isWhitespace(line.charAt(i))) i++;
        int start = i;
        long kib = 0;
        while (i < line.length() && line.charAt(i) >= '0' && line.charAt(i) <= '9') {
            int digit = line.charAt(i++) - '0';
            if (kib > (Long.MAX_VALUE / 1024 - digit) / 10)
                throw new NumberFormatException("Memory value overflow");
            kib = kib * 10 + digit;
        }
        if (i == start || i >= line.length() || !Character.isWhitespace(line.charAt(i)))
            throw new NumberFormatException("Invalid memory value");
        while (i < line.length() && Character.isWhitespace(line.charAt(i))) i++;
        if (i + 2 > line.length() || line.charAt(i) != 'k' || line.charAt(i + 1) != 'B')
            throw new NumberFormatException("Expected kB units");
        i += 2;
        while (i < line.length() && Character.isWhitespace(line.charAt(i))) i++;
        if (i != line.length()) throw new NumberFormatException("Trailing memory data");
        return kib * 1024;
    }

    /** Approximate reclaimable file-cache/buffers/slab, excluding shared/tmpfs pages. */
    static double reclaimableEstimate(long[] values) {
        if (values[0] < 0 || values[1] < 0 || values[3] < 0 || values[4] < 0 || values[5] < 0 || values[6] < 0)
            return Double.NaN;
        if (values[1] > values[0]) return Double.NaN;
        double estimate = (double) values[3] + values[4] + values[6] - values[5];
        return Math.max(0, Math.min(values[0] - values[1], estimate));
    }

    private static void printSummary(long used, long committed, long max, long nonHeap, double cache, double otherUsed, int queued, int capacity, long dropped) {
        consoleLine.setLength(0);
        consoleLine.append("[Memory] RAM available="); appendMiB(systemBytes[2]);
        consoleLine.append('/'); appendMiB(systemBytes[0]);
        consoleLine.append(" (free/cache-est/other-est="); appendMiB(systemBytes[1]); consoleLine.append('/');
        appendMiB(cache); consoleLine.append('/'); appendMiB(otherUsed); consoleLine.append(')');
        consoleLine.append("; RSS="); appendMiB(processBytes[0]);
        consoleLine.append(" (anon/file/shared="); appendMiB(processBytes[3]); consoleLine.append('/');
        appendMiB(processBytes[4]); consoleLine.append('/'); appendMiB(processBytes[5]); consoleLine.append(')');
        consoleLine.append("; heap used/committed/max="); appendMiB(used); consoleLine.append('/');
        appendMiB(committed); consoleLine.append('/'); appendMiB(max);
        consoleLine.append("; nonheap="); appendMiB(nonHeap);
        consoleLine.append("; log queue=").append(queued).append('/').append(capacity).append(" dropped=").append(dropped);
        consoleLine.append("\n[Memory change, rolling <=60s] heap="); appendTrend(0);
        consoleLine.append(" RSS="); appendTrend(1);
        consoleLine.append(" anon="); appendTrend(2);
        consoleLine.append(" file="); appendTrend(3);
        consoleLine.append(" shared="); appendTrend(4);
        consoleLine.append(" available="); appendTrend(5);
        consoleLine.append(" nonheap="); appendTrend(6);
        consoleLine.append(" direct="); appendTrend(7);
        consoleLine.append(" system-shared="); appendTrend(8);
        consoleLine.append(" kernel-slab="); appendTrend(9);
        consoleLine.append("; heap window minimum="); appendMiB(trends[0].minimum());
        System.out.println(consoleLine.toString());
    }
    private static void appendTrend(int i) {
        double change = trends[i].delta();
        if (!Double.isFinite(change)) { consoleLine.append("n/a"); return; }
        appendNumber(change / MIB);
        consoleLine.append("MiB/").append((long) trends[i].seconds()).append('s');
    }
    private static void appendMiB(double bytes) {
        if (bytes < 0 || !Double.isFinite(bytes)) consoleLine.append("n/a");
        else { appendNumber(bytes / MIB); consoleLine.append("MiB"); }
    }
    private static void appendNumber(double value) {
        if (!Double.isFinite(value)) { consoleLine.append("n/a"); return; }
        long tenths = Math.round(value * 10);
        if (tenths < 0) consoleLine.append('-');
        long absolute = Math.abs(tenths);
        consoleLine.append(absolute / 10).append('.').append(absolute % 10);
    }
}
