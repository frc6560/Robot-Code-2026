package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A command that automatically aligns the robot to a target pose for climbing.
 * Uses simple PID control for X, Y, and rotation.
 */
public class ClimbCommand extends SequentialCommandGroup {

    // Poses
    private Pose2d targetPose;
    private Pose2d prescorePose;
    private double initialY;

    // PID Controllers
    private PIDController xController;
    private PIDController yController;
    private PIDController rotationController;

    // Subsystems
    private SwerveSubsystem drivetrain;


    /** Constructor for our climb command */
    public ClimbCommand(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.initialY = drivetrain.getPose().getY();

        // Initialize PID controllers (tune these values as needed)
        // Increased kP for faster response, added kD for damping to prevent overshoot
        this.xController = new PIDController(4.5, 0, 0.4);  // kP: 2.0→4.5, kD: 0→0.4
        this.yController = new PIDController(4.5, 0, 0.4);  // kP: 2.0→4.5, kD: 0→0.4
        this.rotationController = new PIDController(5.0, 0, 0.5);  // kP: 3.0→5.0, kD: 0→0.5
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        setTargets();

        super.addCommands(
            getDriveToPrescore(),
            getDriveInCommand()
        );
        super.addRequirements(drivetrain);
    }


    /** Drives to prescore position using braindead PID */
    public Command getDriveToPrescore() {
        return Commands.run(() -> {
            prescorePose = getPrescore(targetPose);
            Pose2d currentPose = drivetrain.getPose();

            double xVel = xController.calculate(currentPose.getX(), prescorePose.getX());
            double yVel = yController.calculate(currentPose.getY(), prescorePose.getY());
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                prescorePose.getRotation().getRadians()
            );

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            double distance = currentPose.getTranslation().getDistance(prescorePose.getTranslation());
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Prescore Translation Distance", distance);
        }, drivetrain).until(() -> {
            double distance = drivetrain.getPose().getTranslation().getDistance(prescorePose.getTranslation());
            return distance < 0.05;
        });
    }

    /** Drives to final climb position using braindead PID */
    public Command getDriveInCommand() {
        return Commands.run(() -> {
            Pose2d currentPose = drivetrain.getPose();

            double xVel = xController.calculate(currentPose.getX(), targetPose.getX());
            double yVel = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            );

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Climb Translation Distance", distance);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Climb Rotation Error", rotError);
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
            prescoreX = targetPose.getX() + 1.0; // Move back toward center (positive X)
        } else {
            prescoreX = targetPose.getX() - 1.0; // Move back toward center (negative X)
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

        if (alliance.equals(DriverStation.Alliance.Blue)) {
            if (initialY > yThreshold) {
                targetPose = new Pose2d(1.5753228664398193, 4.183515548706055, new Rotation2d(0));
            } else {
                targetPose = new Pose2d(1.5753228664398193, 3.330711841583252, new Rotation2d(0));
            }
        } else {
            if (initialY > yThreshold) {
                targetPose = new Pose2d(14.976325035095215, 4.183515548706055, new Rotation2d(Math.PI));
            } else {
                targetPose = new Pose2d(14.977962493896484, 3.330711841583252, new Rotation2d(Math.PI));
            }
        }
    }
}