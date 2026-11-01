package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import org.junit.jupiter.api.Test;

/** Regression coverage for the calculation/integration rules in glendale-working-branch. */
class LimelightVisionTest {
    private final Pose2d robotPose = new Pose2d(3, 4, new Rotation2d());

    private PoseEstimate frame() {
        PoseEstimate frame = new PoseEstimate();
        frame.pose = robotPose;
        frame.tagCount = 1;
        frame.avgTagDist = 2;
        frame.timestampSeconds = 9.9;
        frame.latency = 20;
        return frame;
    }

    private String check(PoseEstimate frame) {
        return LimelightVision.rejectionReason(frame, robotPose, 0);
    }

    @Test void rejectsMissingNullAndZeroPosesLikeGlendale() {
        assertEquals("No pose received", check(null));
        PoseEstimate frame = frame();
        frame.pose = null;
        assertEquals("Null or zero pose", check(frame));
        frame.pose = new Pose2d();
        assertEquals("Null or zero pose", check(frame));
    }

    @Test void jumpLimitIsTwoMetersAndTurnLimitIs360DegreesPerSecond() {
        assertEquals(2, LimelightConstants.JUMP_TOLERANCE);
        PoseEstimate frame = frame();
        frame.pose = new Pose2d(5, 4, new Rotation2d());
        assertEquals("Ready", check(frame));
        frame.pose = new Pose2d(5.001, 4, new Rotation2d());
        assertEquals("Pose differs by more than 2 m", check(frame));
        for (double sign : new double[] {-1, 1}) {
            assertEquals("Ready", LimelightVision.rejectionReason(frame(), robotPose,
                sign * Math.toRadians(360)));
            assertEquals("Robot turning too fast", LimelightVision.rejectionReason(frame(), robotPose,
                sign * Math.toRadians(360.001)));
        }
    }

    @Test void latestEstimateCanBeReusedAndNoFrameAgeOrDistanceFilterIsApplied() {
        PoseEstimate frame = frame();
        frame.avgTagDist = 9;
        frame.timestampSeconds = 1;
        assertEquals("Ready", check(frame));
        assertEquals("Ready", check(frame));
        frame.timestampSeconds = 100;
        assertEquals("Ready", check(frame));
        // Glendale does not require per-tag distance details or independently check tag count.
        frame.tagCount = 0;
        assertEquals("Ready", check(frame));
    }

    @Test void xyTrustMatchesGlendaleDistanceSquaredPerTagWithFloor() {
        assertEquals(0.05, LimelightConstants.kStdvXYBase);
        assertEquals(0.30, LimelightConstants.kStdvXYFloor);
        assertEquals(9999, LimelightConstants.kStdvThetaBase);
        PoseEstimate frame = frame();
        assertEquals(0.2, LimelightVision.xyStdDevScale(frame) * LimelightConstants.kStdvXYBase, 1e-12);
        frame.tagCount = 2;
        assertEquals(0.1, LimelightVision.xyStdDevScale(frame) * LimelightConstants.kStdvXYBase, 1e-12);
        frame.avgTagDist = 0.1;
        assertEquals(0.015, LimelightVision.xyStdDevScale(frame) * LimelightConstants.kStdvXYBase, 1e-12);
    }

    @Test void integrationUsesCurrentTimeMinusLatencyRatherThanNetworkTimestamp() {
        PoseEstimate frame = frame();
        assertEquals(9.98, LimelightVision.fusionTimestamp(10, frame.latency / 1000.0), 1e-12);
        assertEquals(10.08, LimelightVision.fusionTimestamp(10.1, frame.latency / 1000.0), 1e-12);
    }

    @Test void manualResetAcceptsAnyNonzeroMt1PoseLikeGlendale() {
        assertFalse(LimelightVision.canHardReset(null));
        PoseEstimate frame = frame();
        frame.pose = null;
        assertFalse(LimelightVision.canHardReset(frame));
        frame.pose = new Pose2d();
        assertFalse(LimelightVision.canHardReset(frame));
        frame.pose = new Pose2d(12, 7, Rotation2d.fromDegrees(90));
        frame.avgTagDist = 9;
        frame.timestampSeconds = 1;
        assertTrue(LimelightVision.canHardReset(frame));
    }
}
