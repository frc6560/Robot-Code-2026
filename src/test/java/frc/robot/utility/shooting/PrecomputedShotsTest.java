package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class PrecomputedShotsTest {
    @Test void savedTableMatchesIntegratorAndNeverIntegratesOnLoadOrSolve() {
        var saved = PrecomputedShots.load(Path.of("src/main/deploy/shooting/trajectories.csv"));
        var reference = new FuelShotModel(LaunchCalibration.ideal());
        for (double rpm = 1000; rpm <= 3800; rpm += 5) {
            assertEquals(reference.flight(rpm), saved.flight(rpm), "RPM " + rpm);
        }
        for (double range = 0.75; range <= 8; range += 0.025) {
            assertEquals(reference.solve(range), saved.solve(range), "Range " + range);
            assertEquals(reference.diagnostics(), saved.diagnostics());
        }
        for (double rpm = 1000.1; rpm < 3800; rpm += 7.3) {
            final double measured = rpm;
            assertDoesNotThrow(() -> saved.scores(4, measured));
            int lower = (int) Math.floor((rpm - 1000) / 5);
            double lowRPM = 1000 + lower * 5;
            assertEquals(reference.scores(4, lowRPM) && reference.scores(4, lowRPM + 5),
                saved.scores(4, rpm));
        }
        assertEquals(0, saved.flightCalculations());
        assertFalse(saved.setLaunchSpeedCoefficient(LaunchSpeedTuning.DEFAULT_COEFFICIENT));
        assertThrows(IllegalStateException.class, () -> saved.setLaunchSpeedCoefficient(1.1));
    }
    @Test void reusableResultMatchesSnapshotAndClearsInvalidShots() {
        var model = PrecomputedShots.load(Path.of("src/main/deploy/shooting/trajectories.csv"));
        var result = FuelShotModel.Shot.invalid();
        for (double range = 0.75; range <= 8; range += 0.025) {
            var snapshot = model.solve(range);
            model.solveInto(range, result);
            assertEquals(snapshot, result);
            assertNotSame(snapshot, result);
        }
        model.solveInto(Double.NaN, result);
        assertFalse(result.valid());
        assertEquals(0, result.rpm());
        assertEquals(0, result.flightSeconds());
        assertEquals(0, model.flightCalculations());
    }
    @Test void staleTableIsRejected() throws Exception {
        var path = java.nio.file.Files.createTempFile("stale-shot-table", ".csv");
        try {
            java.nio.file.Files.writeString(path, "wrong coefficient\n");
            assertThrows(IllegalStateException.class, () -> PrecomputedShots.load(path));
        } finally { java.nio.file.Files.delete(path); }
    }
}
