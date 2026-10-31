package frc.robot.commands.swervedrive.drivebase;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class AimAtHubTest {
    @Test void translatingAimTracksBearingWhileStationaryAimHasNoFeedforward() {
        var target = new edu.wpi.first.math.geometry.Translation2d(4, 0);
        assertEquals(0, AimAtHub.trackingOmega(target,
            new edu.wpi.first.math.geometry.Translation2d()));
        double tracking = AimAtHub.trackingOmega(target,
            new edu.wpi.first.math.geometry.Translation2d(0, 1));
        assertEquals(-0.25, tracking, 1e-9);
        assertTrue(AimAtHub.isSettled(0, tracking - tracking));
        assertFalse(AimAtHub.isSettled(0, tracking));
    }

    @Test void releaseAlignmentUsesActualErrorAndMeasuredTurnRate() {
        assertTrue(AimAtHub.isSettled(Math.toRadians(1), Math.toRadians(4)));
        assertTrue(AimAtHub.isSettled(Math.toRadians(-1), Math.toRadians(-4)));
        assertTrue(AimAtHub.isSettled(Math.toRadians(7.5), 0));
        assertTrue(AimAtHub.isSettled(Math.toRadians(-7.5), 0));
        assertFalse(AimAtHub.isSettled(Math.toRadians(7.6), 0));
        assertFalse(AimAtHub.isSettled(Math.toRadians(-7.6), 0));
        assertFalse(AimAtHub.isSettled(0, Math.toRadians(6)));
        assertFalse(AimAtHub.isSettled(Double.NaN, 0));
        assertFalse(AimAtHub.isSettled(0, Double.NaN));
    }
}
