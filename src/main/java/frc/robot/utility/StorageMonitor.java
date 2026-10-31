package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

/** Filesystem-wide /tmp capacity (df-style), not directory size or physical RAM available. */
public final class StorageMonitor {
    private static final Path TMP = Path.of("/tmp");
    private static final MemoryTrend usedTrend = new MemoryTrend(), availableTrend = new MemoryTrend();
    private static final StringBuilder console = new StringBuilder(384);
    private static double nextSample, lastSample = Double.NaN;
    private StorageMonitor() {}

    public static void update() {
        double now = Timer.getFPGATimestamp();
        if (now >= lastSample && now < nextSample) return;
        lastSample = now;
        nextSample = now + 1;
        double total = Double.NaN, free = Double.NaN, available = Double.NaN;
        String type = "unavailable", name = "unavailable";
        boolean ok = false;
        try {
            var store = Files.getFileStore(TMP);
            total = store.getTotalSpace();
            free = store.getUnallocatedSpace();
            available = store.getUsableSpace();
            type = store.type();
            name = store.name();
            ok = total >= 0 && free >= 0 && available >= 0 && free <= total && available <= total;
        } catch (IOException | SecurityException | UnsupportedOperationException ignored) {
            // Missing /tmp or filesystem statistics must not interrupt controls.
        }
        if (!ok) { total = free = available = Double.NaN; type = name = "unavailable"; }
        double used = total - free;
        double percentage = usedPercent(total, free, available);
        number("Storage/Tmp/TotalMiB", total / 1048576);
        number("Storage/Tmp/UsedMiB", used / 1048576);
        number("Storage/Tmp/FreeMiB", free / 1048576);
        number("Storage/Tmp/AvailableMiB", available / 1048576);
        number("Storage/Tmp/UsedPercent", percentage);
        text("Storage/Tmp/FileSystemType", type);
        text("Storage/Tmp/FileStoreName", name);
        flag("Storage/Tmp/ReadOK", ok);
        flag("Storage/Tmp/KnownRamBackedType", isRamBacked(type));
        usedTrend.add(now, used);
        availableTrend.add(now, available);
        number("Storage/Tmp/UsedSinceStartMiB", usedTrend.sinceStart() / 1048576);
        number("Storage/Tmp/UsedWindowChangeMiB", usedTrend.delta() / 1048576);
        number("Storage/Tmp/UsedMiBPerMinute", usedTrend.changePerMinute() / 1048576);
        number("Storage/Tmp/AvailableWindowChangeMiB", availableTrend.delta() / 1048576);
        number("Storage/Tmp/WindowSeconds", usedTrend.seconds());
        console.setLength(0);
        console.append("[Storage /tmp filesystem] total="); appendSize(total);
        console.append(" used="); appendSize(used);
        console.append(" available="); appendSize(available);
        console.append(" use="); appendNumber(percentage); console.append("% type=").append(type);
        if (isRamBacked(type)) console.append(" (RAM-backed)");
        console.append(" store=").append(name).append("; used change=");
        if (Double.isFinite(usedTrend.delta())) {
            appendNumber(usedTrend.delta() / 1048576);
            console.append("MiB/").append((long) usedTrend.seconds()).append('s');
        } else console.append("n/a");
        System.out.println(console.toString());
    }

    /** df-style denominator excludes reserved/unavailable free space. */
    static double usedPercent(double total, double free, double available) {
        if (!Double.isFinite(total) || !Double.isFinite(free) || !Double.isFinite(available)
                || total <= 0 || free < 0 || available < 0 || free > total || available > total)
            return Double.NaN;
        double used = total - free;
        return used + available > 0 ? 100 * used / (used + available) : Double.NaN;
    }
    static boolean isRamBacked(String type) { return "tmpfs".equals(type) || "ramfs".equals(type); }
    private static void number(String key, double value) {
        RateLimitedTelemetry.putNumber(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void text(String key, String value) {
        RateLimitedTelemetry.putString(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void flag(String key, boolean value) {
        RateLimitedTelemetry.putBoolean(key, value);
        RateLimitedTelemetry.recordOutput(key, value);
    }
    private static void appendSize(double bytes) {
        if (!Double.isFinite(bytes)) { console.append("n/a"); return; }
        if (bytes >= 1073741824) { appendNumber(bytes / 1073741824); console.append("GiB"); }
        else if (bytes >= 1048576) { appendNumber(bytes / 1048576); console.append("MiB"); }
        else if (bytes >= 1024) { appendNumber(bytes / 1024); console.append("KiB"); }
        else { appendNumber(bytes); console.append('B'); }
    }
    private static void appendNumber(double value) {
        if (!Double.isFinite(value)) { console.append("n/a"); return; }
        long tenths = Math.round(value * 10);
        if (tenths < 0) console.append('-');
        long absolute = Math.abs(tenths);
        console.append(absolute / 10).append('.').append(absolute % 10);
    }
}
