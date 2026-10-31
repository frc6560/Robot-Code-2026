package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class MemoryMonitorTest {
    @Test void parsesLinuxMemoryReadingsWithSpacesOrTabs() {
        assertEquals(65536L * 1024, MemoryMonitor.parseProcBytes("MemAvailable:     65536 kB"));
        assertEquals(12345L * 1024, MemoryMonitor.parseProcBytes("VmRSS:\t   12345 kB"));
        assertEquals(0, MemoryMonitor.parseProcBytes("Buffers: 0 kB"));
        assertThrows(NumberFormatException.class, () -> MemoryMonitor.parseProcBytes("VmRSS: unknown"));
    }
}
