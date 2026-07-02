package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;


public class Autoalign extends SequentialCommandGroup {
    
    // Autopilot configuration
    private static final APConstraints kConstraints = new APConstraints()
        .withAcceleration(5.0)
        .withJerk(2.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    // Camera + tag-frame goal. SAME frame getTagRelativeRobotPose() returns:
    //   x = meters out from the tag face, y = meters to the left, heading = pi means facing the tag.
    private static final String kCamera = "limelight-br";
    private static final double kStandoffMeters = 0.5; // TUNE: tag-face -> robot-center distance
    private static final Pose2d kTargetPose = new Pose2d(kStandoffMeters, 0.0, new Rotation2d(Math.PI));
    // Direction of travel at arrival = into the tag (-x) = pi  (straight head-on approach).
    private static final Rotation2d kEntryAngle = new Rotation2d(Math.PI);

    // PID Controllers (kept for backward compatibility and heading control)
    private PIDController rotationController;

    // Subsystems
    private SwerveSubsystem drivetrain;


    /** Constructor for our climb command */
    public Autoalign(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        // Heading controller for Autopilot's target-angle reference. (MUST be constructed before use.)
        this.rotationController = new PIDController(5.5, 0.15, 0.05);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Log initialization
        SmartDashboard.putString("Climb/Status", "Initialized");

        super.addCommands(
            getDriveToTarget()
        );
        super.addRequirements(drivetrain);
    }

    /** Update PID values from SmartDashboard (call this in execute if you want live tuning) */


    /** Drives to the tag-relative target using Autopilot (everything in the TAG frame). */
    public Command getDriveToTarget() {
        return Commands.run(() -> {
            Pose2d currentPose = getTagRelativeRobotPose(kCamera);          // robot in tag frame
            ChassisSpeeds robotRelativeSpeeds = drivetrain.getRobotVelocity();

            APTarget target = new APTarget(kTargetPose).withEntryAngle(kEntryAngle);

            Autopilot.APResult output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, target);

            // Autopilot's vx/vy are in the pose frame (= tag frame here).
            double xVel = output.vx().in(edu.wpi.first.units.Units.MetersPerSecond);
            double yVel = output.vy().in(edu.wpi.first.units.Units.MetersPerSecond);
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                output.targetAngle().getRadians()
            );

            // Convert tag-frame velocities to robot-relative using the robot's heading IN THE TAG
            // FRAME (= currentPose.getRotation()), then drive robot-relative.
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            // ---- Essential debug logging ----
            // Vision health (the usual failure: no botpose_targetspace from the Limelight).
            SmartDashboard.putBoolean("Climb/Prescore/HasTarget", LimelightHelpers.getTV(kCamera));
            SmartDashboard.putNumber("Climb/Prescore/Tid", LimelightHelpers.getFiducialID(kCamera));
            SmartDashboard.putNumber("Climb/Prescore/BotposeNorm",
                LimelightHelpers.getBotPose3d_TargetSpace(kCamera).getTranslation().getNorm());
            // Robot pose in the tag frame (what we feed Autopilot).
            SmartDashboard.putNumber("Climb/Prescore/Current_X", currentPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Current_Y", currentPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Current_HeadingDeg", currentPose.getRotation().getDegrees());
            // Target (tag frame).
            SmartDashboard.putNumber("Climb/Prescore/Target_X", kTargetPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Target_Y", kTargetPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Target_HeadingDeg", kTargetPose.getRotation().getDegrees());
            // Errors + distance.
            SmartDashboard.putNumber("Climb/Prescore/Error_X", kTargetPose.getX() - currentPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Error_Y", kTargetPose.getY() - currentPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Error_Rot_Deg",
                Math.toDegrees(kTargetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));
            SmartDashboard.putNumber("Climb/Prescore/Distance",
                currentPose.getTranslation().getDistance(kTargetPose.getTranslation()));
            // Autopilot output (commanded, tag frame) + its heading reference.
            SmartDashboard.putNumber("Climb/Prescore/Output_X_Vel", xVel);
            SmartDashboard.putNumber("Climb/Prescore/Output_Y_Vel", yVel);
            SmartDashboard.putNumber("Climb/Prescore/Output_Rot_Vel", rotVel);
            SmartDashboard.putNumber("Climb/Prescore/TargetAngleDeg", output.targetAngle().getDegrees());
            SmartDashboard.putBoolean("Climb/Prescore/At_Target", kAutopilot.atTarget(currentPose, target));
        }, drivetrain).until(() -> {
            APTarget target = new APTarget(kTargetPose).withEntryAngle(kEntryAngle);
            return kAutopilot.atTarget(getTagRelativeRobotPose(kCamera), target);
        }).finallyDo(() -> drivetrain.drive(new ChassisSpeeds()));
    }

    private Pose2d getTagRelativeRobotPose(String limelightName) {
        Pose3d r = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
        Translation3d fwd = new Translation3d(1, 0, 0).rotateBy(r.getRotation());
        return new Pose2d(r.getZ(), -r.getX(), new Rotation2d(Math.atan2(-fwd.getX(), fwd.getZ())));
    }
    }




