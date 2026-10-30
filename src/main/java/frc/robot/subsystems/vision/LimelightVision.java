package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LimelightVision {
    private Pose2d robotPose2d = new Pose2d();
    private double latency;
    private final String name;
    private final String logKey;
    private final Pose3d cameraPose;
    private final SwerveSubsystem drivebase;
    private double lastProcessedTimestamp = Double.NEGATIVE_INFINITY;
    private double lastUsedTime = Double.NaN;
    private double lastHeartbeat = -1;
    private double lastHeartbeatTime = Double.NEGATIVE_INFINITY;

    public LimelightVision(SwerveSubsystem drivebase, String name, Pose3d cameraPose) {
        this.name = name;
        this.drivebase = drivebase;
        this.cameraPose = cameraPose;
        logKey = "Vision/" + switch (name) {
            case LimelightConstants.FRONT_CAMERA -> "Front";
            case LimelightConstants.LEFT_CAMERA -> "Left";
            case LimelightConstants.RIGHT_CAMERA -> "Right";
            default -> name;
        };
        // Values are already in Limelight's forward/right/up convention.
        LimelightHelpers.setCameraPose_RobotSpace(name,
            cameraPose.getX(), cameraPose.getY(), cameraPose.getZ(),
            Units.radiansToDegrees(cameraPose.getRotation().getX()),
            Units.radiansToDegrees(cameraPose.getRotation().getY()),
            Units.radiansToDegrees(cameraPose.getRotation().getZ()));
    }

    public void update() {
        updateRotation();
        updateLimelightEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
    }

    public void updateLimelightEstimate(PoseEstimate estimate) {
        double now = Timer.getFPGATimestamp();
        Pose2d fusedPose = drivebase.getPose();
        double omega = drivebase.getRobotVelocity().omegaRadiansPerSecond;
        double heartbeat = LimelightHelpers.getLimelightNTTableEntry(name, "hb").getDouble(-1);
        if (heartbeat >= 0 && heartbeat != lastHeartbeat) {
            lastHeartbeat = heartbeat;
            lastHeartbeatTime = now;
        }
        boolean connected = now - lastHeartbeatTime < 1.0;
        String status = rejectionReason(estimate, fusedPose, omega, now, lastProcessedTimestamp);
        if (!connected) status = "Camera heartbeat missing/stale";
        boolean used = status.equals("Ready");
        double stdXY = Double.NaN;
        latency = estimate == null ? 0 : estimate.latency / 1000.0;
        if (estimate != null && finitePose(estimate.pose)) robotPose2d = estimate.pose;
        if (estimate != null && Double.isFinite(estimate.timestampSeconds)
                && estimate.timestampSeconds <= now && estimate.timestampSeconds > lastProcessedTimestamp) {
            lastProcessedTimestamp = estimate.timestampSeconds;
        }
        if (used) {
            stdXY = Math.max(LimelightConstants.kStdvXYFloor,
                estimate.avgTagDist * estimate.avgTagDist / estimate.tagCount) * LimelightConstants.kStdvXYBase;
            drivebase.getSwerveDrive().setVisionMeasurementStdDevs(
                VecBuilder.fill(stdXY, stdXY, LimelightConstants.kStdvThetaBase));
            drivebase.getSwerveDrive().addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
            lastUsedTime = now;
            status = "Used";
        }
        // Vision fusion above still runs every loop; publish only a compact 5 Hz summary.
        if (now < nextTelemetry) return;
        nextTelemetry = now + 0.2;
        SmartDashboard.putString(logKey + "/Status", status);
        SmartDashboard.putBoolean(logKey + "/Connected", connected);
        SmartDashboard.putNumber(logKey + "/TagCount", estimate == null ? 0 : estimate.tagCount);
        SmartDashboard.putBoolean(name + "/MeasurementAccepted", used);
        Logger.recordOutput(logKey + "/Status", status);
        Logger.recordOutput(logKey + "/Connected", connected);
        Logger.recordOutput(logKey + "/UsedThisLoop", used);
    }

    private double nextTelemetry;

    static boolean finitePose(Pose2d pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians());
    }

    /** Pure validation so missing data, repeated frames and bad numbers cannot look accepted. */
    static String rejectionReason(PoseEstimate estimate, Pose2d robotPose, double omega,
            double now, double previousTimestamp) {
        if (estimate == null) return "No pose received";
        if (estimate.tagCount <= 0) return "No tags detected";
        if (!finitePose(estimate.pose) || !finitePose(robotPose) || !Double.isFinite(omega)
                || !Double.isFinite(estimate.avgTagDist) || estimate.avgTagDist < 0
                || !Double.isFinite(estimate.latency) || estimate.latency < 0)
            return "Invalid pose or sensor data";
        if (!Double.isFinite(estimate.timestampSeconds) || estimate.timestampSeconds <= 0
                || estimate.timestampSeconds > now) return "Invalid frame timestamp";
        if (now - estimate.timestampSeconds > 0.5) return "Frame older than 0.5 seconds";
        if (estimate.timestampSeconds <= previousTimestamp) return "Waiting for new frame";
        if (Math.abs(omega) > Units.degreesToRadians(360)) return "Robot turning too fast";
        if (estimate.pose.getTranslation().getDistance(robotPose.getTranslation()) > LimelightConstants.JUMP_TOLERANCE)
            return "Pose differs by more than 2 m; verify map/start position";
        return "Ready";
    }

    /** Explicit operator reset uses MT1, which does not require the current robot heading. */
    public void hardUpdate() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        double now = Timer.getFPGATimestamp();
        String status = estimate == null ? "No pose received"
            : rejectionReason(estimate, estimate.pose, 0, now, Double.NEGATIVE_INFINITY);
        if (status.equals("Ready")) {
            robotPose2d = estimate.pose;
            drivebase.resetOdometry(robotPose2d);
            status = "Reset to MegaTag1";
        }
        Logger.recordOutput(logKey + "/ResetStatus", status);
        SmartDashboard.putString(logKey + "/ResetStatus", status);
    }

    public void updateRotation() {
        Rotation2d rotation = drivebase.getPose().getRotation();
        double yawRate = Units.radiansToDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond);
        Logger.recordOutput(logKey + "/HeadingSentDeg", rotation.getDegrees());
        Logger.recordOutput(logKey + "/YawRateSentDegPerSec", yawRate);
        LimelightHelpers.SetRobotOrientation(name, rotation.getDegrees(), yawRate, 0, 0, 0, 0);
    }

    public void disableVision(boolean disabled) {
        LimelightHelpers.setPipelineIndex(name, disabled ? 0 : 1);
    }

    public Pose2d getRobotPose2d() { return robotPose2d; }
    public double getLatency() { return latency; }
    public String getName() { return name; }
}
