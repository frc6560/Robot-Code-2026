package frc.robot.commands.swervedrive.drivebase;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class AimAtHubTest {
    @Test void releaseAlignmentUsesActualErrorAndMeasuredTurnRate() {
        assertTrue(AimAtHub.isSettled(Math.toRadians(1), Math.toRadians(4)));
        assertTrue(AimAtHub.isSettled(Math.toRadians(-1), Math.toRadians(-4)));
        assertFalse(AimAtHub.isSettled(Math.toRadians(2), 0));
        assertFalse(AimAtHub.isSettled(0, Math.toRadians(6)));
        assertFalse(AimAtHub.isSettled(Double.NaN, 0));
        assertFalse(AimAtHub.isSettled(0, Double.NaN));
    }
}
