package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class StorageMonitorTest {
    @Test void dfPercentageUsesAvailableSpaceAndExcludesReservedSpace() {
        assertEquals(80, StorageMonitor.usedPercent(1000, 200, 200));
        assertEquals(80000.0 / 900, StorageMonitor.usedPercent(1000, 200, 100), 1e-9);
        assertEquals(100, StorageMonitor.usedPercent(1000, 0, 0));
        assertEquals(0, StorageMonitor.usedPercent(1000, 1000, 1000));
        assertTrue(Double.isNaN(StorageMonitor.usedPercent(0, 0, 0)));
        assertTrue(Double.isNaN(StorageMonitor.usedPercent(1000, -1, 100)));
        assertTrue(Double.isNaN(StorageMonitor.usedPercent(1000, 1001, 100)));
        assertTrue(Double.isNaN(StorageMonitor.usedPercent(Double.NaN, 0, 0)));
    }
    @Test void ramBackedClassificationDoesNotGuessFromDirectoryName() {
        assertTrue(StorageMonitor.isRamBacked("tmpfs"));
        assertTrue(StorageMonitor.isRamBacked("ramfs"));
        assertFalse(StorageMonitor.isRamBacked("ext4"));
        assertFalse(StorageMonitor.isRamBacked("overlay"));
        assertFalse(StorageMonitor.isRamBacked("apfs"));
    }
}
