package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class MemoryMonitorTest {
    @Test void rejectsWrongUnitsTrailingDataAndOverflow() {
        for (String line : new String[] {"VmRSS: -1 kB", "VmRSS: 5 MB", "VmRSS: 5 kB junk",
                "VmRSS: 5kB", "VmRSS 5 kB", "VmRSS: 9007199254740992 kB", "VmRSS: 1.5 kB"})
            assertThrows(NumberFormatException.class, () -> MemoryMonitor.parseProcBytes(line), line);
        assertEquals((Long.MAX_VALUE / 1024) * 1024,
            MemoryMonitor.parseProcBytes("VmRSS: " + (Long.MAX_VALUE / 1024) + " kB"));
    }
    @Test void sourceFailuresClearOldValuesAndMalformedFieldsDoNotHideOtherReadings() throws Exception {
        var path = java.nio.file.Files.createTempFile("memory-monitor", ".txt");
        String[] fields = {"VmRSS:", "VmData:", "RssAnon:"};
        long[] bytes = {999, 999, 999};
        try {
            java.nio.file.Files.writeString(path, "VmRSS: broken\nVmData: 123 kB\n");
            assertTrue(MemoryMonitor.readProc(path, fields, bytes));
            assertArrayEquals(new long[] {-1, 123 * 1024, -1}, bytes);
            java.nio.file.Files.delete(path);
            assertFalse(MemoryMonitor.readProc(path, fields, bytes));
            assertArrayEquals(new long[] {-1, -1, -1}, bytes);
        } finally { java.nio.file.Files.deleteIfExists(path); }
    }
    @Test void cacheEstimateExcludesSharedPagesAndNeverDoubleCountsAsAvailableMemory() {
        long[] values = {1000, 100, 400, 250, 50, 75, 25};
        double cache = MemoryMonitor.reclaimableEstimate(values);
        assertEquals(250, cache);
        double other = values[0] - values[1] - cache;
        assertEquals(1000, values[1] + cache + other);
        values[5] = 1000;
        assertEquals(0, MemoryMonitor.reclaimableEstimate(values));
        values[6] = -1;
        assertTrue(Double.isNaN(MemoryMonitor.reclaimableEstimate(values)));
    }
    @Test void parsesLinuxMemoryReadingsWithSpacesOrTabs() {
        assertEquals(65536L * 1024, MemoryMonitor.parseProcBytes("MemAvailable:     65536 kB"));
        assertEquals(12345L * 1024, MemoryMonitor.parseProcBytes("VmRSS:\t   12345 kB"));
        assertEquals(0, MemoryMonitor.parseProcBytes("Buffers: 0 kB"));
        assertThrows(NumberFormatException.class, () -> MemoryMonitor.parseProcBytes("VmRSS: unknown"));
    }
}
