package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import org.junit.jupiter.api.Test;

class LimelightVisionTest {
    private PoseEstimate frame() {
        PoseEstimate frame = new PoseEstimate();
        frame.pose = new Pose2d(3, 4, new Rotation2d());
        frame.tagCount = 1;
        frame.avgTagDist = 2;
        frame.timestampSeconds = 9.9;
        frame.latency = 20;
        return frame;
    }
    private String check(PoseEstimate frame) {
        return LimelightVision.rejectionReason(frame, new Pose2d(3, 4, new Rotation2d()), 0, 10, 9.8);
    }
    @Test void missingAndZeroTagFramesCannotBeAccepted() {
        assertEquals("No pose received", check(null));
        PoseEstimate frame = frame(); frame.tagCount = 0;
        assertEquals("No tags detected", check(frame));
    }
    @Test void acceptsFreshTaggedFrameButDoesNotRefuseARealFieldOrigin() {
        assertEquals("Ready", check(frame()));
        PoseEstimate frame = frame(); frame.pose = new Pose2d();
        assertEquals("Ready", LimelightVision.rejectionReason(frame, new Pose2d(), 0, 10, 9.8));
    }
    @Test void rejectsStaleRepeatedAndFutureFrames() {
        PoseEstimate frame = frame(); frame.timestampSeconds = 9;
        assertEquals("Frame older than 0.5 seconds", check(frame));
        frame.timestampSeconds = 9.8;
        assertEquals("Waiting for new frame", check(frame));
        frame.timestampSeconds = 11;
        assertEquals("Invalid frame timestamp", check(frame));
    }
    @Test void rejectsNonfiniteDataBeforeCalculatingTrust() {
        PoseEstimate frame = frame(); frame.pose = new Pose2d(Double.NaN, 4, new Rotation2d());
        assertEquals("Invalid pose or sensor data", check(frame));
        frame = frame(); frame.avgTagDist = Double.NaN;
        assertEquals("Invalid pose or sensor data", check(frame));
        frame = frame(); frame.pose = null;
        assertEquals("Invalid pose or sensor data", check(frame));
    }
    @Test void identifiesLargeJumpsAndFastTurnsSeparately() {
        PoseEstimate frame = frame(); frame.pose = new Pose2d(12, 4, new Rotation2d());
        assertTrue(check(frame).startsWith("Pose differs by more than 2 m"));
        assertEquals("Robot turning too fast", LimelightVision.rejectionReason(
            frame(), frame().pose, Math.toRadians(361), 10, 9.8));
    }
}
