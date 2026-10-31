package frc.robot.utility.telemetry;

import static org.junit.jupiter.api.Assertions.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

class TelemetryRateLimitTest {
    @Test void fiftyHertzCallsOnlySerializeAtTwoHertz() {
        AtomicLong time = new AtomicLong();
        List<Long> writtenAt = new ArrayList<>();
        var limiter = new LatestValueLimiter(time::get, (key, value) -> writtenAt.add(time.get()));
        for (int cycle = 0; cycle < 1000; cycle++) {
            time.set(cycle * 20_000L);
            limiter.offer("Shooter/RPM", cycle);
            limiter.flush();
        }
        assertEquals(40, writtenAt.size());
        for (int i = 1; i < writtenAt.size(); i++)
            assertTrue(writtenAt.get(i) - writtenAt.get(i - 1) >= 500_000);
        assertEquals(1, limiter.keyCount());
    }
    @Test void finalStopIsFlushedWithoutAnotherCommandExecution() {
        AtomicLong time = new AtomicLong();
        List<Object> values = new ArrayList<>();
        var limiter = new LatestValueLimiter(time::get, (key, value) -> values.add(value));
        limiter.offer("Feeding", true);
        time.set(20_000);
        limiter.offer("Feeding", false);
        time.set(499_999);
        limiter.flush();
        assertEquals(List.of(true), values);
        time.set(500_000);
        limiter.flush();
        assertEquals(List.of(true, false), values);
    }
    @Test void pendingStorageStaysBoundedEvenWithChangingKeys() {
        var limiter = new LatestValueLimiter(() -> 0, (key, value) -> {});
        for (int i = 0; i < 20_000; i++) limiter.offer("Diagnostic/" + i, i);
        assertEquals(LatestValueLimiter.MAX_KEYS, limiter.keyCount());
    }
    @Test void receiverCapsAllFramesIncludingBuiltInFieldsAndForwardsLifecycle() throws Exception {
        List<Long> timestamps = new ArrayList<>();
        int[] lifecycle = new int[2];
        var receiver = new RateLimitedReceiver(new LogDataReceiver() {
            @Override public void start() { lifecycle[0]++; }
            @Override public void end() { lifecycle[1]++; }
            @Override public void putTable(LogTable table) {
                timestamps.add(table.getTimestamp());
                assertEquals(123.0, table.getSubtable("SystemStats").get("Test", 0.0));
            }
        });
        receiver.start();
        for (int i = 0; i < 100; i++) {
            var table = new LogTable(i * 20_000L);
            table.getSubtable("SystemStats").put("Test", 123.0);
            receiver.putTable(table);
        }
        receiver.end();
        assertEquals(List.of(0L, 500_000L, 1_000_000L, 1_500_000L), timestamps);
        assertArrayEquals(new int[] {1, 1}, lifecycle);
    }
}
