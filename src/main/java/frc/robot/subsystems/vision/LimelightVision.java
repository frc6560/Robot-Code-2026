package frc.robot.subsystems.vision;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;

public class LimelightVision{
    private static final double FIELD_MARGIN_METERS = 1.0;

    private Pose2d robotPose2d = new Pose2d();
    private double latency = 0.0;
    private final String name;
    private double kStdvXY = Double.POSITIVE_INFINITY;
    private double kStdvTheta = Double.POSITIVE_INFINITY;

    private boolean measurementAccepted = true;
    private final StructPublisher<Pose2d> posePublisher;
    private double nextTelemetry;

    private SwerveSubsystem drivebase;

    public LimelightVision(SwerveSubsystem drivebase, String name, Pose3d cameraPose) {
        this.name = name;
        String logKey = "Vision/" + switch (name) {
            case LimelightConstants.FRONT_CAMERA -> "Front";
            case LimelightConstants.LEFT_CAMERA -> "Left";
            case LimelightConstants.RIGHT_CAMERA -> "Right";
            default -> name;
        };
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/" + logKey + "/RawPose", Pose2d.struct)
            .publish(PubSubOption.periodic(0.5));
        this.drivebase = drivebase;
        // Sets the camera's position on the robot. The actual Pose3d this originates from comes from the camera.
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            cameraPose.getTranslation().getX(),
            cameraPose.getTranslation().getY(),
            cameraPose.getTranslation().getZ(),
            Units.radiansToDegrees(cameraPose.getRotation().getX()),
            Units.radiansToDegrees(cameraPose.getRotation().getY()),
            Units.radiansToDegrees(cameraPose.getRotation().getZ())
        );
    }

    public void update(){
        updateRotation();
        updateLimelightEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name));
    }

    public void updateLimelightEstimate(PoseEstimate poseEstimate){
        measurementAccepted = true;

        if(poseEstimate == null){
            return;
        }

        robotPose2d = poseEstimate.pose;
        latency = poseEstimate.latency / 1000.0; // in seconds

        double now = Timer.getFPGATimestamp();
        if (now >= nextTelemetry && robotPose2d != null
                && Double.isFinite(robotPose2d.getX()) && Double.isFinite(robotPose2d.getY())
                && Double.isFinite(robotPose2d.getRotation().getRadians())) {
            nextTelemetry = now + 0.5;
            posePublisher.set(robotPose2d);
        }

        String status = rejectionReason(poseEstimate, drivebase.getPose(),
            drivebase.getRobotVelocity().omegaRadiansPerSecond);
        measurementAccepted = status.equals("Ready");


        if(!measurementAccepted){
            return;
        }

        // Calculates standard deviation dynamically. never use rotation.
        // Floor prevents the formula from producing unphysically tight covariances at
        // close range / high tag counts (a few mm std dev is tighter than the camera
        // solution is actually capable of).
        kStdvXY = xyStdDevScale(poseEstimate);
        kStdvTheta = LimelightConstants.kStdvThetaBase;

        drivebase.getSwerveDrive().setVisionMeasurementStdDevs(
            VecBuilder.fill(kStdvXY * LimelightConstants.kStdvXYBase,
                             kStdvXY * LimelightConstants.kStdvXYBase,
                            kStdvTheta)
        );

        // Adds our vision measurement
        drivebase.getSwerveDrive().addVisionMeasurement(
            robotPose2d,
            fusionTimestamp(Timer.getFPGATimestamp(), latency)
        );
    }

    static String rejectionReason(PoseEstimate estimate, Pose2d robotPose, double omega) {
        String invalidPose = invalidPoseReason(estimate);
        if (invalidPose != null) return invalidPose;
        if (Math.abs(omega) > Units.degreesToRadians(360)) return "Robot turning too fast";
        if (estimate.pose.getTranslation().getDistance(robotPose.getTranslation())
                > LimelightConstants.JUMP_TOLERANCE) return "Pose differs by more than 2 m";
        return "Ready";
    }

    static double xyStdDevScale(PoseEstimate estimate) {
        return Math.max(LimelightConstants.kStdvXYFloor,
            Math.pow(estimate.avgTagDist, 2) / estimate.tagCount);
    }

    static double fusionTimestamp(double now, double latencySeconds) {
        return now - latencySeconds;
    }

    static String invalidPoseReason(PoseEstimate estimate) {
        if (estimate == null) return "No pose received";
        if (estimate.pose == null || estimate.pose.equals(Pose2d.kZero)) return "Null or zero pose";

        double x = estimate.pose.getX();
        double y = estimate.pose.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) return "Non-finite position";

        double nearestX = MathUtil.clamp(x, 0.0, FieldConstants.FIELD_LENGTH);
        double nearestY = MathUtil.clamp(y, 0.0, FieldConstants.FIELD_WIDTH);
        if (Math.hypot(x - nearestX, y - nearestY) > FIELD_MARGIN_METERS) {
            return "Pose more than 1 m outside field bounds";
        }
        return null;
    }

    /** Glendale's heading-independent MT1 reset; return success for the current driver command. */
    public boolean hardUpdate(){
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
        if (poseEstimate != null) robotPose2d = poseEstimate.pose;
        String invalidPose = invalidPoseReason(poseEstimate);
        boolean reset = invalidPose == null;
        if (reset) {
            drivebase.resetOdometry(robotPose2d);
            posePublisher.set(robotPose2d);
        }


        return reset;
    }

    public void updateRotation(){
        Rotation2d robotRotation = drivebase.getPose().getRotation();
        // Pipe real yaw rate so MegaTag2 can compensate for rotation-during-exposure.
        // Passing 0 with a spinning robot causes MT2 to solve against a stale heading.
        double yawRateDegPerSec = Units.radiansToDegrees(
            drivebase.getRobotVelocity().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(
            this.name,
            robotRotation.getDegrees(), yawRateDegPerSec,
            0, 0, 0, 0);
    }

    public void disableVision(boolean isDisabled){
        if(isDisabled){
            // 0 is the disabled pipeline. 1 is the enabled pipeline. Apply for all cameras.
            LimelightHelpers.setPipelineIndex(this.name, 0);
        } else {
            LimelightHelpers.setPipelineIndex(this.name, 1);
        }
    }

    public Pose2d getRobotPose2d(){
        return robotPose2d;
    }

    public double getLatency(){
        return latency;
    }

    public String getName(){
        return name;
    }
}
