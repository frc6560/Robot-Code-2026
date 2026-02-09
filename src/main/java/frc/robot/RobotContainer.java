package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.RevolverSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;

import swervelib.SwerveInputStream;

import java.io.File;

public class RobotContainer {

    /* ================= CONTROLLERS ================= */

    // Driver controller (port 0)
    private final CommandXboxController driverXbox =
        new CommandXboxController(0);

    // Operator controller (port 1 ONLY)
    private final XboxController operatorXbox =
        new XboxController(1);

    private final ManualControls controls =
        new ManualControls(null, operatorXbox); // firstXbox unused

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

        /* ================= REVOLVER (OPERATOR CONTROLLER) ================= */

        // Y → FEED while held
        new Trigger(controls::revolverFeed)
            .onTrue(Commands.runOnce(revolver::requestFeed, revolver))
            .onFalse(Commands.runOnce(revolver::requestStop, revolver));

        // A → HARD STOP
        new Trigger(controls::revolverStop)
            .onTrue(Commands.runOnce(revolver::requestStop, revolver));

        // Left bumper → BEAM BREAK OVERRIDE while held
        new Trigger(controls::revolverOverride)
            .onTrue(Commands.runOnce(revolver::enableBeamBreakOverride, revolver))
            .onFalse(Commands.runOnce(revolver::disableBeamBreakOverride, revolver));
    }
}
