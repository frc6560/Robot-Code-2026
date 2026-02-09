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

public class LimelightVision{
    private Pose2d robotPose2d = new Pose2d();
    private double latency = 0.0;
    private final String name;
    private double kStdvXY = Double.POSITIVE_INFINITY;
    private double kStdvTheta = Double.POSITIVE_INFINITY;

    private SwerveSubsystem drivebase;

    public LimelightVision(SwerveSubsystem drivebase, String name, Pose3d cameraPose) {
        this.name = name;
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

    Pose2d nullPose = new Pose2d();

    public void updateLimelightEstimate(PoseEstimate poseEstimate){
        if(poseEstimate == null){
            return;
        }
        robotPose2d = poseEstimate.pose;
        latency = poseEstimate.latency / 1000.0; // in seconds
        
        SmartDashboard.putNumber(this.name + "/TagCount", poseEstimate.tagCount);
        SmartDashboard.putNumber(this.name + "/AvgTagDist", poseEstimate.avgTagDist);
        SmartDashboard.putNumber(this.name + "/Latency", latency);
        SmartDashboard.putNumber(this.name + "/STDVX", kStdvXY * LimelightConstants.kStdvXYBase);

        if(!Double.isNaN(poseEstimate.pose.getX()) && poseEstimate.tagCount > 0){
            drivebase.getSwerveDrive().field.getObject(this.name + "/LimelightPose").setPose(robotPose2d);
        }

        // Rejects null measurements
        if(robotPose2d == null || robotPose2d.equals(nullPose)){
            return;
        }

        // Rejects bad measurements, like sudden jumps in vision pose
        if(robotPose2d.getTranslation()
            .getDistance(drivebase.getPose().getTranslation()) > LimelightConstants.JUMP_TOLERANCE){
            return;
        }

        // Calculates standard deviation dynamically. Only use rotation in certain circumstances.
        boolean useRotation = poseEstimate.tagCount > 1 && 
                            poseEstimate.avgTagDist < Units.feetToMeters(5); // be AGGRESSIVE. this is only to tune out drift in edge cases.

        kStdvXY = Math.pow(poseEstimate.avgTagDist, 2) 
                            / poseEstimate.tagCount;
        kStdvTheta = 
            useRotation ? Math.pow(poseEstimate.avgTagDist, 2) 
                            / poseEstimate.tagCount 
                        : Double.POSITIVE_INFINITY;

        drivebase.getSwerveDrive().setVisionMeasurementStdDevs(
            VecBuilder.fill(kStdvXY * LimelightConstants.kStdvXYBase,
                             kStdvXY * LimelightConstants.kStdvXYBase,
                            kStdvTheta * LimelightConstants.kStdvThetaBase)
        );

        // Adds our vision measurement
        drivebase.getSwerveDrive().addVisionMeasurement(
            robotPose2d,
            Timer.getFPGATimestamp() - latency
        );
    }

    /** This is assuming that the gyro is going to be down. */
    public void hardUpdate(){
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
        if(poseEstimate != null){
            robotPose2d = poseEstimate.pose;
            if(robotPose2d!= null && !robotPose2d.equals(new Pose2d())) drivebase.resetOdometry(robotPose2d);
        }
    }

    public void updateRotation(){
        Rotation2d robotRotation = drivebase.getPose().getRotation();
        LimelightHelpers.SetRobotOrientation(this.name, robotRotation.getDegrees(), 0, 0, 0, 0, 0);
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
