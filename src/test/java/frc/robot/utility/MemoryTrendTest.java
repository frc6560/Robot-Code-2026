package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class MemoryTrendTest {
    @Test void usesActualElapsedTimeAndSignedGrowth() {
        var trend = new MemoryTrend();
        assertTrue(Double.isNaN(trend.bytesPerMinute()));
        trend.add(10, 100);
        assertEquals(0, trend.sinceStart());
        assertTrue(Double.isNaN(trend.bytesPerMinute()));
        trend.add(20, 160);
        assertEquals(60, trend.delta());
        assertEquals(360, trend.bytesPerMinute());
        assertEquals(100, trend.minimum());
        trend.add(30, 80);
        assertEquals(-20, trend.sinceStart());
        assertEquals(-60, trend.bytesPerMinute());
    }
    @Test void oldValuesExpireButSessionBaselineIsRetained() {
        var trend = new MemoryTrend();
        for (int i = 0; i <= 1000; i++) trend.add(i, i * 100);
        assertEquals(60, trend.seconds());
        assertEquals(6000, trend.delta());
        assertEquals(100000, trend.sinceStart());
        assertEquals(94000, trend.minimum());
        trend.add(2000, 200000);
        assertEquals(0, trend.seconds());
        assertTrue(Double.isNaN(trend.bytesPerMinute()));
    }
    @Test void unavailableSamplesDoNotProduceFalseGrowthAndClockResetRestartsBaseline() {
        var trend = new MemoryTrend();
        trend.add(10, 100);
        trend.add(11, -1);
        assertTrue(Double.isNaN(trend.sinceStart()));
        trend.add(12, 200);
        assertEquals(100, trend.sinceStart());
        assertTrue(Double.isNaN(trend.delta()));
        trend.add(0, 50);
        assertEquals(0, trend.sinceStart());
        trend.add(1, Double.NaN);
        assertTrue(Double.isNaN(trend.minimum()));
    }
}
