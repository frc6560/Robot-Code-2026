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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.periodic.SuperstructureCommand;
import frc.robot.utility.Shooter.ShotCalculator;

import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;


public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve/falcon"));
    private final VisionSubsystem vision;

    private final Hood hood = new Hood();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final Feeder feeder = new Feeder();

    private final ShotCalculator shotCalculator = new ShotCalculator();

    // Shuffleboard entries for manual control
    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    private final GenericEntry flywheelRPMEntry = tuningTab.add("Flywheel RPM", 2000.0).getEntry();
    private final GenericEntry hoodAngleEntry = tuningTab.add("Hood Angle", 15.0).getEntry();

    private final AutoCommands factory;
    private final AutoModeChooser autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


    public RobotContainer() {
      factory = new AutoCommands(drivebase);

      autoChooser = new AutoModeChooser(factory);
      SmartDashboard.putData("Auto Chooser", autoChooser.getAutoChooser());

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
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
        shotCalculator
      );

      hood.setDefaultCommand(superstructureCommand); // choose one of these to be default!

      configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        driverXbox.a().onTrue(
          Commands.defer(() -> {
            return Commands.runOnce(() -> vision.hardReset("limelight-br"), vision);
          }, Set.of(vision))
        );

        // driverXbox.y()
        //   .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x()
          .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.b()
          .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));
        driverXbox.start().
          onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.y().onTrue(Commands.defer(() -> new ClimbCommand(drivebase), Set.of(drivebase)));

        operatorXbox.y()
          .onTrue(Commands.runOnce(feeder::requestFeed, feeder))
          .onFalse(Commands.runOnce(feeder::requestStop, feeder));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().selectedCommand();
    }
}
