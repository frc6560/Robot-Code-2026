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
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        boolean hasPose = estimate != null && estimate.tagCount > 0 && finitePose(estimate.pose);
        boolean hasMT1 = mt1 != null && mt1.tagCount > 0 && finitePose(mt1.pose);
        double[] ids = estimate == null || estimate.rawFiducials == null ? new double[0]
            : Arrays.stream(estimate.rawFiducials).filter(f -> f != null).mapToDouble(f -> f.id).toArray();
        // Compact dashboard summary; all diagnostic detail lives in the same saved-log folder.
        SmartDashboard.putString(logKey + "/Status", status);
        SmartDashboard.putBoolean(logKey + "/Connected", connected);
        SmartDashboard.putNumber(logKey + "/TagCount", estimate == null ? 0 : estimate.tagCount);
        // Retain the old boolean for existing dashboards, but never leave it stuck true.
        SmartDashboard.putBoolean(name + "/MeasurementAccepted", used);
        Logger.recordOutput(logKey + "/Status", status);
        Logger.recordOutput(logKey + "/Connected", connected);
        Logger.recordOutput(logKey + "/UsedThisLoop", used);
        Logger.recordOutput(logKey + "/SecondsSinceUsed", Double.isFinite(lastUsedTime) ? now - lastUsedTime : -1);
        Logger.recordOutput(logKey + "/NetworkName", name);
        Logger.recordOutput(logKey + "/TagCount", estimate == null ? 0 : estimate.tagCount);
        Logger.recordOutput(logKey + "/TagIDs", ids);
        Logger.recordOutput(logKey + "/PrimaryTagID", LimelightHelpers.getFiducialID(name));
        Logger.recordOutput(logKey + "/TargetVisible", LimelightHelpers.getTV(name));
        Logger.recordOutput(logKey + "/MegaTag1Pose", hasMT1 ? new Pose2d[]{mt1.pose} : new Pose2d[0]);
        Logger.recordOutput(logKey + "/MegaTag1FrameAgeSeconds", mt1 == null ? -1 : now - mt1.timestampSeconds);
        Logger.recordOutput(logKey + "/MegaTag2Pose", hasPose ? new Pose2d[]{estimate.pose} : new Pose2d[0]);
        Logger.recordOutput(logKey + "/HasMegaTag2Pose", hasPose);
        recordPoseNumbers("MegaTag1", hasMT1 ? mt1.pose : null);
        recordPoseNumbers("MegaTag2", hasPose ? estimate.pose : null);
        Logger.recordOutput(logKey + "/FrameAgeSeconds", estimate == null ? -1 : now - estimate.timestampSeconds);
        Logger.recordOutput(logKey + "/LatencySeconds", latency);
        Logger.recordOutput(logKey + "/PoseDifferenceMeters", hasPose
            ? estimate.pose.getTranslation().getDistance(fusedPose.getTranslation()) : -1);
        Logger.recordOutput(logKey + "/StdDevXYMeters", stdXY);
        Logger.recordOutput(logKey + "/ConfiguredMount", cameraPose);
        Logger.recordOutput(logKey + "/ReportedMount", LimelightHelpers.getLimelightNTDoubleArray(name, "camerapose_robotspace"));
        Logger.recordOutput(logKey + "/IMU", LimelightHelpers.getLimelightNTDoubleArray(name, "imu"));
        Logger.recordOutput(logKey + "/PipelineIndex", LimelightHelpers.getCurrentPipelineIndex(name));
        drivebase.getSwerveDrive().field.getObject(logKey + "/CameraEstimate")
            .setPoses(hasPose ? new Pose2d[]{estimate.pose} : new Pose2d[0]);
    }

    private void recordPoseNumbers(String algorithm, Pose2d pose) {
        String key = logKey + "/" + algorithm;
        double x = pose == null ? Double.NaN : pose.getX();
        double y = pose == null ? Double.NaN : pose.getY();
        double heading = pose == null ? Double.NaN : pose.getRotation().getDegrees();
        Logger.recordOutput(key + "/XMeters", x);
        Logger.recordOutput(key + "/YMeters", y);
        Logger.recordOutput(key + "/HeadingDeg", heading);
        SmartDashboard.putString(key + "/Pose", pose == null ? "No tagged pose"
            : String.format(java.util.Locale.ROOT, "X %.3f m, Y %.3f m, heading %.1f deg", x, y, heading));
    }

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
