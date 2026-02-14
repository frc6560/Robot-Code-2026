package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.periodic.HoodCommand;
import frc.robot.commands.periodic.TurretCommand;

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

    private final Hood hood = new Hood(drivebase::getPose);
    private final Shooter shooter = new Shooter(drivebase::getPose);
    private final Turret turret = new Turret();
    private final Feeder feeder = new Feeder();


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
      // hood.setDefaultCommand(new HoodCommand(hood));
      // turret.setDefaultCommand(new TurretCommand(turret, drivebase::getPose, drivebase::getFieldVelocity));
      
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

        driverXbox.y()
          .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x()
          .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.b()
          .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));
        driverXbox.start().
          onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.leftBumper()
          .onTrue(Commands.runOnce(() -> turret.setGoal(92)))
          .onFalse(Commands.runOnce(() -> turret.setGoal(0)));
        driverXbox.rightBumper()
          .onTrue(Commands.runOnce(() -> turret.setGoal(-92)))
          .onFalse(Commands.runOnce(() -> turret.setGoal(0)));

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
          .onTrue(Commands.runOnce(feeder::enableBeamBreakOverride, feeder))
          .onFalse(Commands.runOnce(feeder::disableBeamBreakOverride, feeder));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().selectedCommand();
    }
}
