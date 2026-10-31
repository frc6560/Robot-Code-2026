package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.Test;

class FuelShotModelTest {
    // Synthetic data for software tests only; never installed on the robot.
    public static LaunchCalibration fixture() {
        return new LaunchCalibration(List.of(new LaunchCalibration.Point(1000, 2.66, Double.NaN),
            new LaunchCalibration.Point(2500, 6.65, Double.NaN),
            new LaunchCalibration.Point(3800, 10.11, Double.NaN)), 1500);
    }

    @Test void zeroTranslation3DMatchesOriginalFlightAtAnyHeading() {
        var model = new FuelShotModel(fixture());
        model.setLaunchSpeedCoefficient(1.05);
        for (double rpm : new double[] {1000, 2400, 2800, 3500}) {
            var stationary = model.flight(rpm);
            for (double heading : new double[] {0, 0.7, Math.PI / 2, -2.4}) {
                var moving = model.movingFlight(rpm, heading, 0, 0);
                assertEquals(stationary.reached(), moving.reached());
                if (!stationary.reached()) continue;
                assertEquals(stationary.crossingX() * Math.cos(heading), moving.crossingX(), 1e-7);
                assertEquals(stationary.crossingX() * Math.sin(heading), moving.crossingY(), 1e-7);
                assertEquals(stationary.entryDegrees(), moving.entryDegrees(), 1e-7);
                assertEquals(stationary.time(), moving.time(), 1e-7);
            }
        }
    }

    @Test void inheritedVelocityUses3DAerodynamicsAndRotatesWithTheField() {
        var model = new FuelShotModel(fixture());
        var stationary = model.flight(3000);
        var moving = model.movingFlight(3000, 0, 0.3, 0.8);
        var rotated = model.movingFlight(3000, Math.PI / 2, -0.8, 0.3);
        assertTrue(moving.reached());
        assertTrue(rotated.reached());
        assertEquals(-moving.crossingY(), rotated.crossingX(), 1e-7);
        assertEquals(moving.crossingX(), rotated.crossingY(), 1e-7);
        assertEquals(moving.time(), rotated.time(), 1e-7);
        assertEquals(moving.entryDegrees(), rotated.entryDegrees(), 1e-7);
        // Drag/lift must act on inherited motion; a simple displacement shift is insufficient.
        assertNotEquals(stationary.crossingX() + 0.3 * stationary.time(), moving.crossingX(), 0.01);
        assertTrue(moving.crossingY() < 0.8 * moving.time());
        model.setLaunchSpeedCoefficient(1.05);
        assertNotEquals(moving.crossingX(), model.movingFlight(3000, 0, 0.3, 0.8).crossingX());
    }

    @Test void liveCoefficientUpdatesFlightAndRejectsInvalidValues() {
        var model = new FuelShotModel(fixture());
        var baseline = model.flight(3000);
        assertTrue(model.setLaunchSpeedCoefficient(1.05));
        assertNotEquals(baseline.crossingX(), model.flight(3000).crossingX());
        assertFalse(model.setLaunchSpeedCoefficient(Double.NaN));
        assertFalse(model.setLaunchSpeedCoefficient(0));
        assertFalse(model.setLaunchSpeedCoefficient(1.05));
        model.setLaunchSpeedCoefficient(1.0);
        assertEquals(baseline.crossingX(), model.flight(3000).crossingX(), 1e-9);
    }

    @Test void interpolatesMeasuredSpeedAndNeverExtrapolates() {
        var calibration = fixture();
        assertEquals(6.65, calibration.at(2500).speed(), 1e-9);
        assertEquals(4.655, calibration.at(1750).speed(), 1e-9);
        assertTrue(Double.isNaN(calibration.at(2500).backspinRps()));
        assertThrows(IllegalArgumentException.class, () -> calibration.at(999));
        assertThrows(IllegalArgumentException.class, () -> calibration.at(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new LaunchCalibration(List.of(
            new LaunchCalibration.Point(1000, 3, 10), new LaunchCalibration.Point(2000, 2, 20),
            new LaunchCalibration.Point(3000, 8, 30)), 1500));
    }

    @Test void rejectsMixedTunnelSpeedsAndMissingData() throws Exception {
        var path = Files.createTempFile("fuel-calibration", ".csv");
        try {
            Files.writeString(path, "rpm,exit_speed_mps,backspin_rps,tunnel_rpm\n"
                + "1000,3,,1500\n2000,6,,1700\n3000,9,,1500\n");
            assertThrows(IllegalArgumentException.class, () -> LaunchCalibration.load(path));
        } finally { Files.delete(path); }
    }

    @Test void selectsInteriorOfReachableBandAndRejectsUnreachableShots() {
        var model = new FuelShotModel(fixture());
        int reachable = 0;
        for (double range = 2; range <= 7; range += 0.1) {
            var shot = model.solve(range);
            if (!shot.valid()) continue;
            var diagnostic = model.diagnostics();
            assertEquals("Reachable", diagnostic.reason());
            assertTrue(diagnostic.sampled() >= diagnostic.reachedHeight());
            assertTrue(diagnostic.reachedHeight() >= diagnostic.clearedNearRim());
            assertTrue(diagnostic.clearedNearRim() >= diagnostic.landedInOpening());
            assertTrue(diagnostic.landedInOpening() >= diagnostic.passedEntryAngle());
            assertTrue(diagnostic.passedEntryAngle() > 0);
            reachable++;
            assertTrue(model.scores(range, shot.rpm()));
            assertTrue(shot.minimumRPM() <= shot.rpm() && shot.rpm() <= shot.maximumRPM());
            assertTrue(model.flight(shot.rpm()).entryDegrees() >= FuelShotConfig.MIN_ENTRY_DEGREES);
            assertTrue(shot.rpm() >= 1000 && shot.rpm() <= 3800);
        }
        assertTrue(reachable > 10, "Synthetic shooter should reach several ranges");
        assertFalse(model.solve(Double.NaN).valid());
        assertFalse(model.solve(20).valid());
        assertEquals(20, model.diagnostics().rangeMeters());
        assertEquals(0, model.diagnostics().sampled());
        assertTrue(model.diagnostics().reason().startsWith("Range outside solver limits"));
        assertFalse(model.scores(4, Double.NaN));
        assertFalse(model.scores(4, 4000));
    }

    @Test void measuredSpinChangesPredictedFlightAndLowSpeedCannotReachRim() {
        var rolling = new FuelShotModel(fixture());
        var noSpin = new FuelShotModel(new LaunchCalibration(List.of(
            new LaunchCalibration.Point(1000, 2.66, 0), new LaunchCalibration.Point(2500, 6.65, 0),
            new LaunchCalibration.Point(3800, 10.11, 0)), 1500));
        assertFalse(rolling.flight(1000).reached());
        assertTrue(rolling.flight(3500).reached());
        assertNotEquals(rolling.flight(3500).crossingX(), noSpin.flight(3500).crossingX(), 0.01);
    }
}
