package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import org.junit.jupiter.api.Test;

class CurrentUsageMonitorTest {
    @Test void ranksTemperaturesIndependentlyAndOmitsMissingSamples() {
        var hottest = CurrentUsageMonitor.rankTemperatures(List.of(
            new CurrentUsageMonitor.TemperatureReading("Intake", 68),
            new CurrentUsageMonitor.TemperatureReading("Shooter", 42),
            new CurrentUsageMonitor.TemperatureReading("Drive", 55),
            new CurrentUsageMonitor.TemperatureReading("Disconnected", Double.NaN)));
        assertEquals(List.of("Intake", "Drive", "Shooter"),
            hottest.stream().map(CurrentUsageMonitor.TemperatureReading::name).toList());
        assertEquals(68, hottest.get(0).celsius());
        assertTrue(CurrentUsageMonitor.rankTemperatures(List.of()).isEmpty());
    }
    @Test void ranksActualSupplyDrawAndIgnoresInvalidReadings() {
        var readings = CurrentUsageMonitor.rank(List.of(
            new CurrentUsageMonitor.Reading("Shooter", 15),
            new CurrentUsageMonitor.Reading("Intake", 39),
            new CurrentUsageMonitor.Reading("Drive", 27),
            new CurrentUsageMonitor.Reading("Floor", 8),
            new CurrentUsageMonitor.Reading("Missing", Double.NaN),
            new CurrentUsageMonitor.Reading("Invalid", Double.POSITIVE_INFINITY)));
        assertEquals(List.of("Intake", "Drive", "Shooter", "Floor"),
            readings.stream().map(CurrentUsageMonitor.Reading::name).toList());
        assertEquals(39, readings.get(0).supplyAmps());
    }

    @Test void handlesNoValidDataAndBreaksTiesConsistently() {
        assertTrue(CurrentUsageMonitor.rank(List.of()).isEmpty());
        assertEquals("A", CurrentUsageMonitor.rank(List.of(
            new CurrentUsageMonitor.Reading("B", 0),
            new CurrentUsageMonitor.Reading("A", 0))).get(0).name());
    }
}
