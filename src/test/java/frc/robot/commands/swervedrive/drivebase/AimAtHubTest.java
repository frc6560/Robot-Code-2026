package frc.robot.commands.swervedrive.drivebase;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class AimAtHubTest {
    @Test void releaseAlignmentUsesActualErrorAndMeasuredTurnRate() {
        assertTrue(AimAtHub.isSettled(Math.toRadians(1), Math.toRadians(4)));
        assertTrue(AimAtHub.isSettled(Math.toRadians(-1), Math.toRadians(-4)));
        assertTrue(AimAtHub.isSettled(Math.toRadians(20), 0));
        assertTrue(AimAtHub.isSettled(Math.toRadians(-20), 0));
        assertFalse(AimAtHub.isSettled(Math.toRadians(20.1), 0));
        assertFalse(AimAtHub.isSettled(Math.toRadians(-20.1), 0));
        assertFalse(AimAtHub.isSettled(0, Math.toRadians(6)));
        assertFalse(AimAtHub.isSettled(Double.NaN, 0));
        assertFalse(AimAtHub.isSettled(0, Double.NaN));
    }
    @Test void acquiredAlignmentSurvivesTurnRateNoiseButStillRequiresInitialSettling() {
        double tolerance = Math.toRadians(20);
        assertFalse(AimAtHub.isSettled(0, Math.toRadians(6), tolerance, false));
        boolean aligned = AimAtHub.isSettled(0, Math.toRadians(4), tolerance, false);
        assertTrue(aligned);
        for (double speed : new double[] {5.1, -5.5, 4.9, 6, -7.9}) {
            aligned = AimAtHub.isSettled(0, Math.toRadians(speed), tolerance, aligned);
            assertTrue(aligned, "Small turn-rate noise should not restart the feed dwell");
        }
        aligned = AimAtHub.isSettled(0, Math.toRadians(8.1), tolerance, aligned);
        assertFalse(aligned);
        assertFalse(AimAtHub.isSettled(0, Math.toRadians(6), tolerance, aligned));
        assertTrue(AimAtHub.isSettled(0, Math.toRadians(4), tolerance, aligned));
    }

    @Test void heldAlignmentStillDropsImmediatelyForBadHeadingOrFeedback() {
        double tolerance = Math.toRadians(20);
        assertFalse(AimAtHub.isSettled(Math.toRadians(20.1), 0, tolerance, true));
        assertFalse(AimAtHub.isSettled(Math.toRadians(-20.1), 0, tolerance, true));
        assertFalse(AimAtHub.isSettled(0, Math.toRadians(-8.1), tolerance, true));
        assertFalse(AimAtHub.isSettled(Double.NaN, 0, tolerance, true));
        assertFalse(AimAtHub.isSettled(0, Double.NaN, tolerance, true));
        // Passing applies its own heading limit through the same stateful gate.
        assertFalse(AimAtHub.isSettled(Math.toRadians(11), 0, Math.toRadians(10), true));
    }
}
