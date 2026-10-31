package frc.robot.utility.telemetry;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.LongSupplier;

/** Bounded storage, no history queue: a suppressed update replaces the pending value. */
final class LatestValueLimiter {
    static final int MAX_KEYS = 2048;
    private final Map<String, Pending> values = new LinkedHashMap<>();
    private final LongSupplier clock;
    private final BiConsumer<String, Object> writer;
    private static final class Pending {
        Object value;
        boolean dirty;
        long nextWrite = Long.MIN_VALUE;
    }
    LatestValueLimiter(LongSupplier clock, BiConsumer<String, Object> writer) {
        this.clock = clock;
        this.writer = writer;
    }
    void offer(String key, Object value) {
        Pending pending = values.get(key);
        if (pending == null) {
            if (values.size() >= MAX_KEYS) return;
            pending = new Pending();
            values.put(key, pending);
        }
        pending.value = value;
        pending.dirty = true;
        writeIfDue(key, pending, clock.getAsLong());
    }
    void flush() {
        long now = clock.getAsLong();
        for (var entry : values.entrySet()) writeIfDue(entry.getKey(), entry.getValue(), now);
    }
    int keyCount() { return values.size(); }
    private void writeIfDue(String key, Pending pending, long now) {
        if (!pending.dirty || now < pending.nextWrite) return;
        // Advance first so even a failing writer cannot generate a high-frequency retry storm.
        pending.dirty = false;
        pending.nextWrite = now + RateLimitedTelemetry.PERIOD_MICROS;
        writer.accept(key, pending.value);
    }
}
