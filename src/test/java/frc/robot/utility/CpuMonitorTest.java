package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Files;
import org.junit.jupiter.api.Test;

class CpuMonitorTest {
    @Test void cycleStatsPreservePeaksAndCumulativeOverrunsAcrossWindows() {
        var stats = new CpuCycleStats();
        stats.record(10000, 20000, -1);
        stats.record(20000, 20000, 20000);
        stats.record(40000, 20000, 25000);
        assertEquals(3, stats.cycles);
        assertEquals(70000, stats.workMicros);
        assertEquals(40000, stats.peakWorkMicros);
        assertEquals(25000, stats.peakIntervalMicros);
        assertEquals(1, stats.overruns);
        stats.clearWindow();
        assertEquals(0, stats.cycles);
        assertEquals(0, stats.overruns);
        assertEquals(1, stats.totalOverruns);
        stats.record(-1, 20000, 20000);
        stats.record(100, 0, 20000);
        assertEquals(0, stats.cycles);
    }
    @Test void processPercentDistinguishesOneCoreFromMachineCapacity() {
        var counter = new CpuTimeCounter();
        counter.sample(0, 0);
        assertTrue(Double.isNaN(counter.oneCorePercent));
        counter.sample(1_000_000_000, 1_500_000_000);
        assertEquals(150, counter.oneCorePercent);
        assertEquals(75, counter.machinePercent(2));
        assertTrue(Double.isNaN(counter.machinePercent(0)));
        counter.sample(2_000_000_000, -1);
        assertTrue(Double.isNaN(counter.oneCorePercent));
        counter.sample(3_000_000_000L, 2_000_000_000);
        assertTrue(Double.isNaN(counter.oneCorePercent));
        counter.sample(4_000_000_000L, 1);
        assertTrue(Double.isNaN(counter.oneCorePercent));
    }
    @Test void systemDistributionUsesDeltasAndDoesNotDoubleCountGuests() {
        var ticks = new CpuTicks();
        assertTrue(CpuTicks.parse("cpu 100 10 20 200 5 1 4 0 900 900", ticks.incoming));
        ticks.accept();
        assertTrue(Double.isNaN(ticks.busy));
        assertTrue(CpuTicks.parse("cpu 120 15 30 250 10 3 7 5 1900 1900", ticks.incoming));
        ticks.accept();
        assertEquals(25, ticks.user);
        assertEquals(15, ticks.kernel);
        assertEquals(50, ticks.idle);
        assertEquals(5, ticks.ioWait);
        assertEquals(5, ticks.steal);
        assertEquals(45, ticks.busy);
        assertEquals(100, ticks.user + ticks.kernel + ticks.idle + ticks.ioWait + ticks.steal);
    }
    @Test void invalidUnchangedAndRolledBackCountersAreUnavailable() {
        long[] values = new long[8];
        for (String line : new String[] {"intr 1 2 3", "cpu 1 2", "cpu -1 2 3 4 5 6 7 8",
                "cpu 1.5 2 3 4 5 6 7 8", "cpu 99999999999999999999 2 3 4 5 6 7 8", "cpux 1 2 3 4 5 6 7 8"})
            assertFalse(CpuTicks.parse(line, values), line);
        var ticks = new CpuTicks();
        CpuTicks.parse("cpu0 10 10 10 10 10 10 10 10", ticks.incoming); ticks.accept(); ticks.accept();
        assertTrue(Double.isNaN(ticks.busy));
        CpuTicks.parse("cpu0 9 11 11 11 11 11 11 11", ticks.incoming); ticks.accept();
        assertTrue(Double.isNaN(ticks.busy));
    }
    @Test void missingProcAndMissingCoresNeverLeaveOldValues() throws Exception {
        var file = Files.createTempFile("cpu-monitor", ".txt");
        var system = new CpuTicks();
        var cores = new CpuTicks[] {new CpuTicks(), new CpuTicks()};
        try {
            Files.writeString(file, "cpu 1 0 1 8 0 0 0 0\ncpu0 1 0 1 8 0 0 0 0\ncpu1 0 0 0 10 0 0 0 0\nintr 5\n");
            assertTrue(CpuMonitor.readSystem(file, system, cores));
            Files.writeString(file, "cpu 2 0 2 16 0 0 0 0\ncpu0 2 0 2 16 0 0 0 0\nintr 9\n");
            assertTrue(CpuMonitor.readSystem(file, system, cores));
            assertEquals(20, system.busy);
            assertEquals(20, cores[0].busy);
            assertTrue(Double.isNaN(cores[1].busy));
            Files.delete(file);
            assertFalse(CpuMonitor.readSystem(file, system, cores));
            assertTrue(Double.isNaN(system.busy));
        } finally { Files.deleteIfExists(file); }
    }
}
