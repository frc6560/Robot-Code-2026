package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utility.telemetry.RateLimitedTelemetry;

/** Samples JVM heap once a second without forcing garbage collection. */
public final class MemoryMonitor {
    private static final java.nio.file.Path PROCESS_STATUS = java.nio.file.Path.of("/proc/self/status");
    private static final java.nio.file.Path SYSTEM_STATUS = java.nio.file.Path.of("/proc/meminfo");
    private static final String[] PROCESS_FIELDS = {"VmRSS:", "VmSize:", "VmData:"};
    private static final String[] PROCESS_NAMES = {"Memory/ProcessResidentMiB", "Memory/ProcessVirtualMiB", "Memory/ProcessDataMiB"};
    private static final String[] SYSTEM_FIELDS = {"MemTotal:", "MemFree:", "MemAvailable:", "Cached:", "Buffers:"};
    private static final String[] SYSTEM_NAMES = {"Memory/SystemTotalMiB", "Memory/SystemFreeMiB", "Memory/SystemAvailableMiB", "Memory/SystemCachedMiB", "Memory/SystemBuffersMiB"};
    private static final long[] processBytes = new long[PROCESS_FIELDS.length];
    private static final long[] systemBytes = new long[SYSTEM_FIELDS.length];
    private static final StringBuilder consoleLine = new StringBuilder(256);
    private static double nextConsole;
    private static double nextSample;
    private static long peak;
    private MemoryMonitor() {}
    public static void update() {
        double now = Timer.getFPGATimestamp();
        if (now < nextSample) return;
        nextSample = now + 1;
        Runtime runtime = Runtime.getRuntime();
        long used = runtime.totalMemory() - runtime.freeMemory();
        peak = Math.max(peak, used);
        publish("Memory/HeapUsedMiB", used);
        publish("Memory/HeapCommittedMiB", runtime.totalMemory());
        publish("Memory/HeapMaxMiB", runtime.maxMemory());
        publish("Memory/HeapHeadroomMiB", runtime.maxMemory() - used);
        publish("Memory/HeapPeakMiB", peak);
        // Linux-only: distinguish whole-process memory from heap and reclaimable OS cache.
        sampleProc(PROCESS_STATUS, PROCESS_FIELDS, PROCESS_NAMES, processBytes);
        sampleProc(SYSTEM_STATUS, SYSTEM_FIELDS, SYSTEM_NAMES, systemBytes);
        if (now >= nextConsole) {
            nextConsole = now + 1;
            consoleLine.setLength(0);
            consoleLine.append("[Memory] Controller RAM available=");
            appendMiB(consoleLine, systemBytes[2]);
            consoleLine.append(" / ");
            appendMiB(consoleLine, systemBytes[0]);
            consoleLine.append("; robot process=");
            appendMiB(consoleLine, processBytes[0]);
            consoleLine.append("; Java heap used=");
            appendMiB(consoleLine, used);
            consoleLine.append(" / ");
            appendMiB(consoleLine, runtime.maxMemory());
            consoleLine.append("; heap headroom=");
            appendMiB(consoleLine, runtime.maxMemory() - used);
            System.out.println(consoleLine.toString());
        }
    }
    private static void sampleProc(java.nio.file.Path path, String[] fields, String[] names, long[] bytes) {
        java.util.Arrays.fill(bytes, -1);
        if (!java.nio.file.Files.isReadable(path)) return;
        try (var reader = java.nio.file.Files.newBufferedReader(path)) {
            String line;
            while ((line = reader.readLine()) != null) {
                for (int i = 0; i < fields.length; i++) {
                    if (line.startsWith(fields[i])) {
                        bytes[i] = parseProcBytes(line);
                        publish(names[i], bytes[i]);
                    }
                }
            }
        } catch (java.io.IOException | NumberFormatException ignored) {
            // Diagnostics must never interrupt robot control.
        }
    }
    private static void appendMiB(StringBuilder line, long bytes) {
        if (bytes < 0) line.append("unavailable");
        else line.append(bytes / 1048576L).append(" MiB");
    }
    static long parseProcBytes(String line) {
        int start = line.indexOf(':') + 1;
        while (start < line.length() && Character.isWhitespace(line.charAt(start))) start++;
        int end = start;
        while (end < line.length() && Character.isDigit(line.charAt(end))) end++;
        return Long.parseLong(line.substring(start, end)) * 1024L;
    }
    private static void publish(String name, long bytes) {
        double mib = bytes / 1048576.0;
        RateLimitedTelemetry.putNumber(name, mib);
        RateLimitedTelemetry.recordOutput(name, mib);
    }
}
