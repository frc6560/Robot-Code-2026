package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utility.telemetry.RateLimitedTelemetry;

/** Samples JVM heap once a second without forcing garbage collection. */
public final class MemoryMonitor {
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
        publish("HeapUsedMiB", used);
        publish("HeapCommittedMiB", runtime.totalMemory());
        publish("HeapMaxMiB", runtime.maxMemory());
        publish("HeapHeadroomMiB", runtime.maxMemory() - used);
        publish("HeapPeakMiB", peak);
        // Linux-only: distinguish whole-process memory from heap and reclaimable OS cache.
        sampleProc("/proc/self/status", new String[] {"VmRSS", "VmSize", "VmData"},
            new String[] {"ProcessResidentMiB", "ProcessVirtualMiB", "ProcessDataMiB"});
        sampleProc("/proc/meminfo", new String[] {"MemTotal", "MemFree", "MemAvailable", "Cached", "Buffers"},
            new String[] {"SystemTotalMiB", "SystemFreeMiB", "SystemAvailableMiB", "SystemCachedMiB", "SystemBuffersMiB"});
    }
    private static void sampleProc(String path, String[] fields, String[] names) {
        if (!java.nio.file.Files.isReadable(java.nio.file.Path.of(path))) return;
        try (var reader = java.nio.file.Files.newBufferedReader(java.nio.file.Path.of(path))) {
            String line;
            while ((line = reader.readLine()) != null) {
                for (int i = 0; i < fields.length; i++) {
                    if (line.startsWith(fields[i] + ":")) {
                        String value = line.substring(line.indexOf(':') + 1).trim().split("\\s+")[0];
                        publish(names[i], Long.parseLong(value) * 1024L);
                    }
                }
            }
        } catch (java.io.IOException | NumberFormatException ignored) {
            // Diagnostics must never interrupt robot control.
        }
    }
    private static void publish(String name, long bytes) {
        double mib = bytes / 1048576.0;
        RateLimitedTelemetry.putNumber("Memory/" + name, mib);
        RateLimitedTelemetry.recordOutput("Memory/" + name, mib);
    }
}
