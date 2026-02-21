package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.periodic.SuperstructureCommand;
import frc.robot.utility.Shooter.ShotCalculator;

import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.superstructure.LEDSubsystem;


public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox   = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/falcon"));
    private final VisionSubsystem vision;

    private final Hood    hood    = new Hood();
    private final Shooter shooter = new Shooter();
    private final Turret  turret  = new Turret();
    private final Feeder  feeder  = new Feeder();

    private final LEDSubsystem leds = new LEDSubsystem();

    private final ShotCalculator shotCalculator = new ShotCalculator();

    private final AutoCommands   factory;
    private final AutoModeChooser autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> -driverXbox.getRightX())
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);


    public RobotContainer() {
        factory     = new AutoCommands(drivebase);
        autoChooser = new AutoModeChooser(factory);
        SmartDashboard.putData("Auto Chooser", autoChooser.getAutoChooser());

        List<LimelightVision> limelights = new ArrayList<LimelightVision>();
        for (String name : LimelightConstants.LIMELIGHT_NAMES) {
            Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
            limelights.add(new LimelightVision(drivebase, name, cameraPose));
        }

        vision = new VisionSubsystem(limelights);

        SuperstructureCommand superstructureCommand = new SuperstructureCommand(
                hood,
                shooter,
                turret,
                drivebase::getPose,
                drivebase::getFieldVelocity,
                shotCalculator);

        // Hood is the default command owner since it's the slowest to react
        hood.setDefaultCommand(superstructureCommand);

        // LEDs run every loop automatically via their internal state machine —
        // no default command needed.

        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // ── Driver bindings ───────────────────────────────────────────────────

        driverXbox.a().onTrue(
                Commands.defer(() ->
                    Commands.runOnce(() -> vision.hardReset("limelight-br"), vision),
                Set.of(vision)));

        driverXbox.y()
                .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        driverXbox.x()
                .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));

        driverXbox.b()
                .onTrue(Commands.runOnce(() ->
                    CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));

        driverXbox.start()
                .onTrue(Commands.runOnce(drivebase::zeroNoAprilTagsGyro));

        // Turret manual aim + LED shift signals
        driverXbox.leftBumper()
                .onTrue(Commands.runOnce(() -> {
                    turret.setGoal(92);
                    leds.setShiftLeft(true);
                }))
                .onFalse(Commands.runOnce(() -> {
                    turret.setGoal(0);
                    leds.setShiftLeft(false);
                }));

        driverXbox.rightBumper()
                .onTrue(Commands.runOnce(() -> {
                    turret.setGoal(-92);
                    leds.setShiftRight(true);
                }))
                .onFalse(Commands.runOnce(() -> {
                    turret.setGoal(0);
                    leds.setShiftRight(false);
                }));

        // ── Operator bindings ─────────────────────────────────────────────────

        operatorXbox.b().onTrue(Commands.runOnce(() -> {
            hood.setGoal(20.0);
        }, hood));

        operatorXbox.x().onTrue(Commands.runOnce(() -> {
            hood.setGoal(0.0);
        }, hood));

        operatorXbox.y()
                .onTrue(Commands.runOnce(feeder::requestFeed, feeder))
                .onFalse(Commands.runOnce(feeder::requestStop, feeder));

        operatorXbox.a()
                .onTrue(Commands.runOnce(feeder::requestStop, feeder));

        operatorXbox.leftBumper()
                .onTrue(Commands.run(() -> shooter.setRPM(2000.0), shooter))
                .onFalse(Commands.run(() -> shooter.setRPM(0.0), shooter));

        operatorXbox.rightBumper()
                .onTrue(Commands.runOnce(() -> hood.setGoal(25.0), hood))
                .onFalse(Commands.runOnce(() -> hood.setGoal(0.0), hood));

        // ── LED state updates — wired to robot mechanisms ─────────────────────
        // TODO: replace the hood.atGoal() / feeder checks below with whatever
        //       methods your subsystems expose for "mechanisms stowed" and
        //       "auto stow active". These are best-guess placeholders.

        // Mechanisms stowed = hood is at 0 and feeder is stopped
        // Call this continuously via a trigger on the hood's at-goal state.
        // If Hood exposes a BooleanSupplier, wire it like this:
        //   new Trigger(hood::isAtHome)
        //       .onTrue(Commands.runOnce(() -> leds.setMechanismsStowed(true)))
        //       .onFalse(Commands.runOnce(() -> leds.setMechanismsStowed(false)));
        //
        // For now, update stowed state alongside hood goal changes:
        operatorXbox.x().onTrue(Commands.runOnce(() -> leds.setMechanismsStowed(true)));
        operatorXbox.b().onTrue(Commands.runOnce(() -> leds.setMechanismsStowed(false)));
        operatorXbox.rightBumper()
                .onTrue(Commands.runOnce(() -> leds.setMechanismsStowed(false)))
                .onFalse(Commands.runOnce(() -> leds.setMechanismsStowed(true)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getAutoChooser().selectedCommand();
    }
}