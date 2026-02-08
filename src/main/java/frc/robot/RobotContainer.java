package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.RevolverSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import frc.robot.Constants.OperatorConstants;

import swervelib.SwerveInputStream;

import java.io.File;

public class RobotContainer {

    /* ================= CONTROLLERS ================= */
    private final CommandXboxController driverXbox = new CommandXboxController(0);

    /* ================= DRIVEBASE ================= */
    private final SwerveSubsystem drivebase =
        new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/falcon")
        );

    /* ================= REVOLVER ================= */
    private final RevolverSubsystem revolver = new RevolverSubsystem();

    /* ================= DRIVE INPUT ================= */
    private final SwerveInputStream driveAngularVelocity =
        SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX()
        )
        .withControllerRotationAxis(() -> -driverXbox.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    /* ================= CONSTRUCTOR ================= */
    public RobotContainer() {
        configureBindings();
    }

    /* ================= BUTTON BINDINGS ================= */
    private void configureBindings() {

        /* ---------- Drive ---------- */
        drivebase.setDefaultCommand(
            drivebase.driveFieldOriented(driveAngularVelocity)
        );

        /* ---------- Utility ---------- */
        driverXbox.y().onTrue(
            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
        );

        driverXbox.start().onTrue(
            Commands.runOnce(drivebase::zeroNoAprilTagsGyro)
        );

        driverXbox.leftBumper().whileTrue(
            Commands.runOnce(drivebase::lock, drivebase).repeatedly()
        );

        /* ================= REVOLVER ================= */
        // X button: feed while held, stop when released
        driverXbox.x()
            .onTrue(Commands.runOnce(revolver::requestFeed, revolver))
            .onFalse(Commands.runOnce(revolver::requestStop, revolver));
    }
}
