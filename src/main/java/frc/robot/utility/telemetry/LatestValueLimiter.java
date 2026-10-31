package frc.robot.utility.telemetry;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.LongSupplier;

/** Bounded storage, no history queue: a suppressed update replaces the pending value. */
final class LatestValueLimiter {
    static final int MAX_KEYS = 2048;
    private final Map<String, Pending> values = new LinkedHashMap<>();
    private final Pending[] slots = new Pending[MAX_KEYS];
    private int size;
    private final LongSupplier clock;
    private final BiConsumer<String, Object> writer;
    private final java.util.function.ObjDoubleConsumer<String> doubleWriter;
    private static final class Pending {
        String key;
        Object value;
        double number;
        boolean numeric;
        boolean dirty;
        long nextWrite = Long.MIN_VALUE;
    }
    LatestValueLimiter(LongSupplier clock, BiConsumer<String, Object> writer) {
        this(clock, writer, (key, value) -> writer.accept(key, value));
    }
    LatestValueLimiter(LongSupplier clock, BiConsumer<String, Object> writer,
            java.util.function.ObjDoubleConsumer<String> doubleWriter) {
        this.clock = clock;
        this.writer = writer;
        this.doubleWriter = doubleWriter;
    }
    void offer(String key, Object value) {
        Pending pending = values.get(key);
        if (pending == null) {
            if (values.size() >= MAX_KEYS) return;
            pending = new Pending();
            pending.key = key;
            slots[size++] = pending;
            values.put(key, pending);
        }
        pending.numeric = false;
        pending.value = value;
        pending.dirty = true;
        writeIfDue(key, pending, clock.getAsLong());
    }
    void offerDouble(String key, double value) {
        Pending pending = values.get(key);
        if (pending == null) {
            if (values.size() >= MAX_KEYS) return;
            pending = new Pending();
            pending.key = key;
            slots[size++] = pending;
            values.put(key, pending);
        }
        pending.numeric = true;
        pending.number = value;
        pending.value = null;
        pending.dirty = true;
        writeIfDue(key, pending, clock.getAsLong());
    }
    void flush() {
        long now = clock.getAsLong();
        for (int i = 0; i < size; i++) {
            Pending pending = slots[i];
            writeIfDue(pending.key, pending, now);
        }
    }
    int keyCount() { return values.size(); }
    private void writeIfDue(String key, Pending pending, long now) {
        if (!pending.dirty || now < pending.nextWrite) return;
        // Advance first so even a failing writer cannot generate a high-frequency retry storm.
        pending.dirty = false;
        pending.nextWrite = now + RateLimitedTelemetry.PERIOD_MICROS;
        if (pending.numeric) doubleWriter.accept(key, pending.number);
        else writer.accept(key, pending.value);
    }
}
