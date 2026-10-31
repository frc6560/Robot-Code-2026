package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;
import java.lang.management.ManagementFactory;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/** Desktop allocation audit: intentionally exercises many more solves than a match. */
class ShotMemoryAuditTest {
    private static volatile Object sink;
    @Test void repeatedShotsKeepFixedCacheAndNeverIntegrate() {
        var model = PrecomputedShots.load(Path.of("src/main/deploy/shooting/trajectories.csv"));
        var bean = ManagementFactory.getThreadMXBean();
        assertInstanceOf(com.sun.management.ThreadMXBean.class, bean);
        var allocations = (com.sun.management.ThreadMXBean) bean;
        allocations.setThreadAllocatedMemoryEnabled(true);
        long thread = Thread.currentThread().getId();
        var result = FuelShotModel.Shot.invalid();
        for (int i = 0; i < 10000; i++) model.solveInto(2 + (i % 500) * 0.01, result);
        int count = model.cachedFlightCount();
        long bytes = model.estimatedCacheBytes();
        int iterations = 100000;
        long before = allocations.getThreadAllocatedBytes(thread);
        for (int i = 0; i < iterations; i++) model.solveInto(2 + (i % 500) * 0.01, result);
        long solveBytes = allocations.getThreadAllocatedBytes(thread) - before;
        before = allocations.getThreadAllocatedBytes(thread);
        int ready = 0;
        for (int i = 0; i < iterations; i++) {
            if (model.scores(2 + (i % 500) * 0.01, 1000.1 + i % 2799)) ready++;
        }
        long scoreBytes = allocations.getThreadAllocatedBytes(thread) - before;
        assertEquals(0, solveBytes, "The runtime solveInto path must not allocate");
        assertEquals(0, scoreBytes, "The runtime scoring check must not allocate");
        assertEquals(count, model.cachedFlightCount());
        assertEquals(bytes, model.estimatedCacheBytes());
        assertEquals(0, model.flightCalculations());
        System.out.printf("SHOT_MEMORY_AUDIT iterations=%d solveAllocatedBytes=%d bytesPerSolve=%.1f "
            + "scoreAllocatedBytes=%d cacheEstimateBytes=%d integrations=%d ready=%d%n",
            iterations, solveBytes, (double) solveBytes / iterations, scoreBytes, bytes,
            model.flightCalculations(), ready);
        sink = null;
    }
}
