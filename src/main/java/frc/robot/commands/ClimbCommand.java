

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimbState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

/**
 * A command that automatically aligns the robot to a target pose for climbing.
 * Uses Autopilot for path planning with PID control for heading.
 */
public class ClimbCommand extends SequentialCommandGroup {

    // Autopilot configuration
    private static final APConstraints kConstraints = new APConstraints()
        .withAcceleration(5.0)
        .withJerk(2.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    // Poses
    private Pose2d targetPose;
    private Pose2d prescorePose;
    private double initialY;

    // PID Controllers (kept for backward compatibility and heading control)
    private PIDController xController;
    private PIDController yController;
    private PIDController rotationController;

    // Subsystems
    private SwerveSubsystem drivetrain;
    private Intake intake;
    private Climber climb;


    /** Constructor for our climb command */
    public ClimbCommand(SwerveSubsystem drivetrain, Intake intake, Climber climb) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.climb = climb;
        this.initialY = drivetrain.getPose().getY();

        // Initialize PID controllers (tune these values as needed)
        // Reduced kP and increased kD to prevent overshoot in X and Y
           this.xController = new PIDController(3, 0.03, 0.1);  // Reduced kP: 4.5→2.5, Increased kD: 0.4→0.8
        this.yController = new PIDController(2.75, 0.02, 0.1);  // Reduced kP: 4.5→2.5, Increased kD: 0.4→0.8
        this.rotationController = new PIDController(5.5, 0.15, 0.05);  // Reduced kP: 5.0→4.0, Increased kD: 0.5→0.6
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Log PID constants to SmartDashboard for tuning
        SmartDashboard.putNumber("Climb/PID/X_kP", 2.5);
        SmartDashboard.putNumber("Climb/PID/X_kD", 0.8);
        SmartDashboard.putNumber("Climb/PID/Y_kP", 2.5);
        SmartDashboard.putNumber("Climb/PID/Y_kD", 0.8);
        SmartDashboard.putNumber("Climb/PID/Rot_kP", 4.0);
        SmartDashboard.putNumber("Climb/PID/Rot_kD", 0.6);

        setTargets();

        // Log initialization
        SmartDashboard.putString("Climb/Status", "Initialized");
        SmartDashboard.putNumber("Climb/Initial_Y", initialY);
        System.out.println("ClimbCommand initialized at Y=" + initialY);

        super.addCommands(
            Commands.parallel(
                Commands.runOnce(() -> intake.setIdleMode(), intake),
                Commands.runOnce(() -> climb.setState(ClimbState.EXTENDED), climb),
                getDriveToPrescore()
            ),
            getDriveInCommand(),
            Commands.runOnce(() -> climb.setState(ClimbState.RETRACTED), climb)
        );
        super.addRequirements(drivetrain, intake, climb);
    }

    /** Update PID values from SmartDashboard (call this in execute if you want live tuning) */


    /** Drives to prescore position using Autopilot */
    public Command getDriveToPrescore() {
        return Commands.run(() -> {
            prescorePose = getPrescore(targetPose);
            Pose2d currentPose = drivetrain.getPose();
            ChassisSpeeds robotRelativeSpeeds = drivetrain.getRobotVelocity();

            // Create Autopilot target with entry angle for curved path
            // Approach from opposite direction (180 degrees from target rotation)
            Rotation2d entryAngle = prescorePose.getRotation().plus(Rotation2d.fromDegrees(180));

            APTarget prescoreTarget = new APTarget(prescorePose)
                .withEntryAngle(entryAngle);

            // Calculate velocities using Autopilot
            Autopilot.APResult output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, prescoreTarget);

            // Extract field-relative velocities and target rotation
            double xVel = output.vx().in(edu.wpi.first.units.Units.MetersPerSecond);
            double yVel = output.vy().in(edu.wpi.first.units.Units.MetersPerSecond);

            // Use Autopilot's target angle for rotation control (like AutoAlign)
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                output.targetAngle().getRadians()
            );

            // Calculate errors for logging
            double distance = currentPose.getTranslation().getDistance(prescorePose.getTranslation());
            double xError = prescorePose.getX() - currentPose.getX();
            double yError = prescorePose.getY() - currentPose.getY();
            double rotError = prescorePose.getRotation().getRadians() - currentPose.getRotation().getRadians();

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            // Get robot velocity for logging
            ChassisSpeeds robotVel = drivetrain.getFieldVelocity();

            // Position logging
            SmartDashboard.putNumber("Climb/Prescore/Current_X", currentPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Current_Y", currentPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Target_X", prescorePose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Target_Y", prescorePose.getY());

            // Error logging
            SmartDashboard.putNumber("Climb/Prescore/Error_X", xError);
            SmartDashboard.putNumber("Climb/Prescore/Error_Y", yError);
            SmartDashboard.putNumber("Climb/Prescore/Error_Rot", rotError);
            SmartDashboard.putNumber("Climb/Prescore/Distance", distance);

            // Velocity/Output logging
            SmartDashboard.putNumber("Climb/Prescore/Output_X_Vel", xVel);
            SmartDashboard.putNumber("Climb/Prescore/Output_Y_Vel", yVel);
            SmartDashboard.putNumber("Climb/Prescore/Output_Rot_Vel", rotVel);

            // Actual robot velocity
            SmartDashboard.putNumber("Climb/Prescore/Actual_X_Vel", robotVel.vxMetersPerSecond);
            SmartDashboard.putNumber("Climb/Prescore/Actual_Y_Vel", robotVel.vyMetersPerSecond);
            SmartDashboard.putNumber("Climb/Prescore/Actual_Rot_Vel", robotVel.omegaRadiansPerSecond);

            // Status
            SmartDashboard.putBoolean("Climb/Prescore/At_Target", kAutopilot.atTarget(currentPose, prescoreTarget));
        }, drivetrain).until(() -> {
            Rotation2d entryAngle = prescorePose.getRotation().plus(Rotation2d.fromDegrees(180));
            APTarget prescoreTarget = new APTarget(prescorePose)
                .withEntryAngle(entryAngle);
            return kAutopilot.atTarget(drivetrain.getPose(), prescoreTarget);
        });
    }

    public Command getDriveInCommand() {
        return Commands.run(() -> {
            Pose2d currentPose = drivetrain.getPose();

            // Calculate errors
            double xError = targetPose.getX() - currentPose.getX();
            double yError = targetPose.getY() - currentPose.getY();
            double rotError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

            // Calculate velocities
            double xVel = xController.calculate(currentPose.getX(), targetPose.getX());
            double yVel = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            );

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            // Get robot velocity for logging
            ChassisSpeeds robotVel = drivetrain.getFieldVelocity();

            // Comprehensive logging
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double absRotError = Math.abs(rotError);

            // Position logging
            SmartDashboard.putNumber("Climb/Final/Current_X", currentPose.getX());
            SmartDashboard.putNumber("Climb/Final/Current_Y", currentPose.getY());
            SmartDashboard.putNumber("Climb/Final/Current_Rot", currentPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Climb/Final/Target_X", targetPose.getX());
            SmartDashboard.putNumber("Climb/Final/Target_Y", targetPose.getY());
            SmartDashboard.putNumber("Climb/Final/Target_Rot", targetPose.getRotation().getDegrees());

            // Error logging
            SmartDashboard.putNumber("Climb/Final/Error_X", xError);
            SmartDashboard.putNumber("Climb/Final/Error_Y", yError);
            SmartDashboard.putNumber("Climb/Final/Error_Rot_Rad", rotError);
            SmartDashboard.putNumber("Climb/Final/Error_Rot_Deg", Math.toDegrees(rotError));
            SmartDashboard.putNumber("Climb/Final/Distance", distance);

            // Velocity/Output logging
            SmartDashboard.putNumber("Climb/Final/Output_X_Vel", xVel);
            SmartDashboard.putNumber("Climb/Final/Output_Y_Vel", yVel);
            SmartDashboard.putNumber("Climb/Final/Output_Rot_Vel", rotVel);

            // Actual robot velocity
            SmartDashboard.putNumber("Climb/Final/Actual_X_Vel", robotVel.vxMetersPerSecond);
            SmartDashboard.putNumber("Climb/Final/Actual_Y_Vel", robotVel.vyMetersPerSecond);
            SmartDashboard.putNumber("Climb/Final/Actual_Rot_Vel", robotVel.omegaRadiansPerSecond);

            // Status
            SmartDashboard.putBoolean("Climb/Final/At_Target", distance < 0.02 && absRotError < 0.017);
            SmartDashboard.putBoolean("Climb/Final/Translation_Done", distance < 0.02);
            SmartDashboard.putBoolean("Climb/Final/Rotation_Done", absRotError < 0.017);

            // Overshoot detection warnings
            if (Math.abs(xError) > 0.5) {
                SmartDashboard.putString("Climb/Final/Warning", "Large X error: " + String.format("%.3f", xError));
            } else if (Math.abs(yError) > 0.5) {
                SmartDashboard.putString("Climb/Final/Warning", "Large Y error: " + String.format("%.3f", yError));
            } else {
                SmartDashboard.putString("Climb/Final/Warning", "None");
            }
        }, drivetrain).until(() -> {
            Pose2d currentPose = drivetrain.getPose();
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
            return distance < 0.02 && rotError < 0.017;
        });
    }



    /** Gets the prescore position 1 meter back from target */
    public Pose2d getPrescore(Pose2d targetPose) {
        DriverStation.Alliance alliance;
        if (!DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.Alliance.Blue;
        } else {
            alliance = DriverStation.getAlliance().get();
        }

        // Calculate prescore position 1 meter back from target
        double prescoreX;
        if (alliance.equals(DriverStation.Alliance.Blue)) {
            prescoreX = targetPose.getX() + 0.3; // Move back toward center (positive X)
        } else {
            prescoreX = targetPose.getX() - 0.3; // Move back toward center (negative X)
        }

        return new Pose2d(prescoreX, targetPose.getY(), targetPose.getRotation());
}

    /** Sets the target pose based on alliance and starting Y position */
    public void setTargets() {
        DriverStation.Alliance alliance;
        if (!DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.Alliance.Blue;
        } else {
            alliance = DriverStation.getAlliance().get();
        }

        double yThreshold = 3.75;
        String selectedTarget = "";

        if (alliance.equals(DriverStation.Alliance.Blue)) {
            if (initialY > yThreshold) {
                targetPose = new Pose2d(1.614, 4.183515548706055, new Rotation2d(0));
                selectedTarget = "Blue Upper";
            } else {
                targetPose = new Pose2d(1.614, 3.292, new Rotation2d(0));
                selectedTarget = "Blue Lower";
            }
        } else {
            if (initialY > yThreshold) {
                targetPose = new Pose2d(14.976325035095215, 4.183515548706055, new Rotation2d(Math.PI));
                selectedTarget = "Red Upper";
            } else {
                targetPose = new Pose2d(14.977962493896484, 3.330711841583252, new Rotation2d(Math.PI));
                selectedTarget = "Red Lower";
            }
        }

        // Log target selection
        SmartDashboard.putString("Climb/Target_Location", selectedTarget);
        SmartDashboard.putNumber("Climb/Target_X", targetPose.getX());
        SmartDashboard.putNumber("Climb/Target_Y", targetPose.getY());
        SmartDashboard.putNumber("Climb/Target_Rot_Deg", targetPose.getRotation().getDegrees());
        System.out.println("ClimbCommand target set to: " + selectedTarget +
                           " at (" + targetPose.getX() + ", " + targetPose.getY() + ")");
    }
}


