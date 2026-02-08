package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SnotmCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.SimpleFlywheelTest;
import frc.robot.commands.FlywheelRawTest;

import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.GroundIntake;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Snotm;
import frc.robot.subsystems.superstructure.Sotm;
import frc.robot.subsystems.superstructure.Turret;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.Auto;
import frc.robot.autonomous.AutoFactory;
import frc.robot.autonomous.AutoRoutines;


public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve/falcon"));
    private final VisionSubsystem vision;
    
    // Subsystems
    private final Feeder feeder = new Feeder();
    private final Flywheel flywheel = new Flywheel();
    private final GroundIntake intake = new GroundIntake();
    private final Hood hood = new Hood();
    private final ShooterLUT lut = new ShooterLUT();
    private final Turret turret = new Turret();
    
    private final Sotm sotm = new Sotm(drivebase, flywheel, turret, lut, hood);
  private final Snotm snotm = new Snotm(drivebase, flywheel, turret, controls, lut, hood, feeder, intake);


    private final AutoFactory factory;
    private final SendableChooser<Auto> autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
  .withControllerRotationAxis(() -> drivebase.isRotationOverridden() ? 0.0 : -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


    public RobotContainer() {
      snotm.setDefaultCommand(new SnotmCommand(snotm, controls, feeder,flywheel));
      feeder.setDefaultCommand(new FeederCommand(feeder, controls));

      factory = new AutoFactory(
      null,
      drivebase
      );

      autoChooser = new SendableChooser<Auto>();

      for(AutoRoutines auto : AutoRoutines.values()) {
        Auto autonomousRoutine = new Auto(auto, factory);
        if(auto == AutoRoutines.TEST){
          autoChooser.setDefaultOption(autonomousRoutine.getName(), autonomousRoutine);
        }
        else {
          autoChooser.addOption(autonomousRoutine.getName(), autonomousRoutine);
        }
      }

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
        Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
        limelights.add(new LimelightVision(drivebase, name, cameraPose));
      }

      vision = new VisionSubsystem(limelights);
      configureBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Wrap the existing SwerveInputStream supplier so that when a rotation
        // override is active we replace only the angular velocity while
        // preserving the translational components. This avoids exclusive
        // ownership of the drivetrain and prevents translation from locking out.
        java.util.function.Supplier<ChassisSpeeds> driveWithOverride = () -> {
          ChassisSpeeds speeds = driveAngularVelocity.get();
          if (drivebase.isRotationOverridden()) {
            return new ChassisSpeeds(speeds.vxMetersPerSecond,
                                     speeds.vyMetersPerSecond,
                                     drivebase.getRotationOverrideOmega());
          }
          return speeds;
        };

        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveWithOverride);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        driverXbox.a().onTrue(
          Commands.defer(() -> {
            return Commands.runOnce(() -> vision.hardReset("limelight"), vision);
          }, Set.of(vision))
        );

        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));

        // Manual flywheel control for calibration (operator controller)
        // D-Pad Up: Increase flywheel speed by 50 RPM
        new Trigger(controls::increaseFlywheelSpeed)
            .onTrue(Commands.runOnce(() -> flywheel.adjustRPM(50), flywheel));

        // D-Pad Down: Decrease flywheel speed by 50 RPM
        new Trigger(controls::decreaseFlywheelSpeed)
            .onTrue(Commands.runOnce(() -> flywheel.adjustRPM(-50), flywheel));

        // D-Pad Left: Reset flywheel to idle speed
        new Trigger(controls::resetFlywheelSpeed)
            .onTrue(Commands.runOnce(() -> flywheel.setIdle(), flywheel));

        // Simple flywheel testing (driver controller)
        // B button: Test at 800 RPM (low speed test)
        driverXbox.b().whileTrue(new SimpleFlywheelTest(flywheel, 800));

        // X button: Test at 1500 RPM (medium speed test)
        driverXbox.x().whileTrue(new SimpleFlywheelTest(flywheel, 1500));

        // Y button: Test at 2500 RPM (high speed test)
        driverXbox.y().whileTrue(new SimpleFlywheelTest(flywheel, 2500));

        // Left bumper: Raw motor test at 20% power (bypasses PID to verify motors work)
        driverXbox.leftBumper().whileTrue(new FlywheelRawTest(0.2));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected().getCommand();
    }
}
