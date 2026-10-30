package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.LimelightConstants;
import java.util.Set;
import org.junit.jupiter.api.Test;

class LimelightConfigurationTest {
    @Test void onlyTheThreeInstalledNetworkNamesAreConfigured() {
        assertEquals(3, LimelightConstants.LIMELIGHT_NAMES.length);
        assertEquals(Set.of("limelight-cl", "limelight-cr", "limelight-back"),
            Set.of(LimelightConstants.LIMELIGHT_NAMES));
        assertThrows(IllegalArgumentException.class,
            () -> LimelightConstants.getLimelightPose("limelight-br"));
        assertThrows(IllegalArgumentException.class,
            () -> LimelightConstants.getLimelightPose("limelight"));
    }

    @Test void mountingValuesMatchCADInLimelightForwardRightUpCoordinates() {
        assertMount(LimelightConstants.LEFT_CAMERA, -0.21325, -0.32168, 0.42430, 15, 90);
        assertMount(LimelightConstants.RIGHT_CAMERA, -0.21325, 0.32168, 0.42430, 15, -90);
        assertMount(LimelightConstants.FRONT_CAMERA, -0.00918, 0, 0.73553, 22, 0);
    }

    private void assertMount(String name, double forward, double right, double up,
            double pitchDeg, double yawDeg) {
        Pose3d pose = LimelightConstants.getLimelightPose(name);
        assertEquals(forward, pose.getX(), 1e-9);
        assertEquals(right, pose.getY(), 1e-9);
        assertEquals(up, pose.getZ(), 1e-9);
        // Check the Euler values actually sent by the camera constructor.
        assertEquals(0, Math.toDegrees(pose.getRotation().getX()), 1e-9);
        assertEquals(pitchDeg, Math.toDegrees(pose.getRotation().getY()), 1e-9);
        assertEquals(yawDeg, Math.toDegrees(pose.getRotation().getZ()), 1e-9);
    }
}
