package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.RobotBase;

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
import frc.robot.commands.periodic.SuperstructureCommand;
import frc.robot.utility.Shooter.ShotCalculator;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOKraken;
import frc.robot.subsystems.climb.ClimbIOSim;


public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve/falcon"));
    private final VisionSubsystem vision;

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final Feeder feeder;
    private final Intake intake;
    private final Climb climb; 

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
      // Initialize subsystems with appropriate IO implementations
      if (RobotBase.isReal()) {
        hood = new Hood(new HoodIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        climb = new Climb(new ClimbIOKraken());
      } else {
        hood = new Hood(new HoodIOSim());
        shooter = new Shooter(new ShooterIOSim());
        turret = new Turret(new TurretIOSim());
        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim());
        climb = new Climb(new ClimbIOSim());
      }

      factory = new AutoCommands(drivebase, feeder, intake);

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

        driverXbox.y()
          .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x()
          .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.b()
          .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));
        driverXbox.start().
          onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));


        driverXbox.rightBumper()
          .onTrue(Commands.runOnce(feeder::requestFeed, feeder))
          .onFalse(Commands.runOnce(feeder::requestStop, feeder));

        driverXbox.leftBumper()
          .onTrue(Commands.runOnce(intake::setExtensionMode, intake))
          .onFalse(Commands.runOnce(intake::setIdleMode, intake));

  
  
        climb.setDefaultCommand(new RunCommand(() -> climb.setState(ClimbState.BOTH_DOWN), climb));

        // Changed operatorController to operatorXbox
        operatorXbox.rightBumper().whileTrue(climb.autoClimbRoutine());

        operatorXbox.povLeft().whileTrue(new RunCommand(() -> climb.setState(ClimbState.LEFT_REACH), climb));
        operatorXbox.povRight().whileTrue(new RunCommand(() -> climb.setState(ClimbState.RIGHT_REACH), climb));
        operatorXbox.povDown().whileTrue(new RunCommand(() -> climb.setState(ClimbState.MEET_MIDDLE), climb));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().selectedCommand();
    }
}
