package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.superstructure.*;
import frc.robot.commands.*;

import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

    /* ================= CONTROLLERS ================= */
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

    /* ================= SUBSYSTEMS ================= */
    private final SwerveSubsystem drivebase =
        new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));

    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final BallGrabber ballGrabber = new BallGrabber();
    private final RevolverSubsystem revolver = new RevolverSubsystem();

    private final SubsystemManager subsystemManager =
        new SubsystemManager(drivebase, elevator, arm, ballGrabber, controls);

    private final VisionSubsystem vision;

    /* ================= AUTONOMOUS ================= */
    private final AutoFactory factory;
    private final SendableChooser<Auto> autoChooser;

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

        // Default commands
        arm.setDefaultCommand(new ArmCommand(arm, controls));
        elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));
        ballGrabber.setDefaultCommand(new BallGrabberCommand(ballGrabber, controls));

        subsystemManager.setDefaultCommand(
            new SubsystemManagerCommand(
                drivebase, elevator, arm, ballGrabber, controls, subsystemManager
            )
        );

        /* ---------- Autonomous chooser ---------- */
        factory = new AutoFactory(null, drivebase);
        autoChooser = new SendableChooser<>();

        for (AutoRoutines auto : AutoRoutines.values()) {
            Auto autonomousRoutine = new Auto(auto, factory);
            if (auto == AutoRoutines.TEST) {
                autoChooser.setDefaultOption(autonomousRoutine.getName(), autonomousRoutine);
            } else {
                autoChooser.addOption(autonomousRoutine.getName(), autonomousRoutine);
            }
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);

        /* ---------- Vision ---------- */
        List<LimelightVision> limelights = new ArrayList<>();
        for (String name : Constants.LimelightConstants.LIMELIGHT_NAMES) {
            Pose3d pose = Constants.LimelightConstants.getLimelightPose(name);
            limelights.add(new LimelightVision(drivebase, name, pose));
        }
        vision = new VisionSubsystem(limelights);

        configureBindings();

        /* ---------- Simulation safety ---------- */
        if (RobotBase.isSimulation()) {
            revolver.requestStop();
        }
    }

    /* ================= BUTTON BINDINGS ================= */
    private void configureBindings() {

        /* ---------- Drive ---------- */
        drivebase.setDefaultCommand(
            drivebase.driveFieldOriented(driveAngularVelocity)
        );

        /* ---------- Driver controls ---------- */
        driverXbox.a().onTrue(
            Commands.defer(
                () -> Commands.runOnce(() -> vision.hardReset("limelight"), vision),
                Set.of(vision)
            )
        );

        driverXbox.b().onTrue(
            Commands.runOnce(() -> drivebase.trackAprilTag().schedule(), drivebase)
        );

        driverXbox.y().onTrue(
            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
        );

        driverXbox.x().onTrue(
            Commands.runOnce(
                () -> CommandScheduler.getInstance()
                    .schedule(drivebase.alignToTrenchCommand()),
                drivebase
            )
        );

        driverXbox.start().onTrue(
            Commands.runOnce(drivebase::zeroNoAprilTagsGyro)
        );

        driverXbox.leftBumper().whileTrue(
            Commands.runOnce(drivebase::lock, drivebase).repeatedly()
        );

        /* ================================================= */
        /* ================== REVOLVER ===================== */
        /* ================================================= */

        // Hold Y → feed using state machine
        controls.revolverFeed()
            .onTrue(Commands.runOnce(revolver::requestFeed, revolver))
            .onFalse(Commands.runOnce(revolver::requestStop, revolver));

        // A → emergency stop
        controls.revolverStop()
            .onTrue(Commands.runOnce(revolver::requestStop, revolver));

        // Left bumper → beam break override
        controls.revolverOverride()
            .onTrue(Commands.runOnce(revolver::enableBeamBreakOverride, revolver))
            .onFalse(Commands.runOnce(revolver::disableBeamBreakOverride, revolver));
    }

    /* ================= AUTONOMOUS ================= */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getCommand();
    }
}
