package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.RobotBase;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.scoring.ShotCommand;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.periodic.SuperstructureCommand;
import frc.robot.utility.Shooter.PassCalculator;
import frc.robot.utility.Shooter.ShotCalculator;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOKraken;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final ManualControls m_Controls = new ManualControls(1);

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
    private final PassCalculator passCalculator = new PassCalculator();
    private Command shotCommand;

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
        shotCalculator,
        passCalculator
      );

      hood.setDefaultCommand(superstructureCommand);

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

        // driverXbox.x()
        //   .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.y()
          .onTrue(Commands.defer(() -> new ClimbCommand(drivebase, intake), Set.of(drivebase, intake)));
        driverXbox.start().
          onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));


        driverXbox.leftBumper()
          .onTrue(Commands.runOnce(intake::setExtensionMode, intake))
          .onFalse(Commands.runOnce(intake::setIdleMode, intake));
        
        Trigger shootTrigger = new Trigger(m_Controls::getShootTrigger);
        Trigger shootReleaseTrigger = new Trigger(m_Controls::getShootReleaseTrigger);

        shootTrigger.onTrue(Commands.runOnce(() -> {
          shotCommand = new ShotCommand(feeder, turret, hood, shooter, shotCalculator);
          shotCommand.schedule();
        }));
        shootReleaseTrigger.onTrue(Commands.runOnce(() -> {
          if (shotCommand != null) {
            shotCommand.cancel();
          }
        }));


        Trigger intakeTrigger = new Trigger(m_Controls::getIntakeTrigger);
        Trigger intakeRollingTrigger = new Trigger(m_Controls::getIntakeRollingTrigger);
        Trigger intakeReleaseTrigger = new Trigger(m_Controls::getIntakeReleaseTrigger);

        intakeTrigger.onTrue(Commands.runOnce(intake::setExtendOnlyMode, intake));
        intakeRollingTrigger.onTrue(Commands.runOnce(intake::setExtensionMode, intake));
        intakeReleaseTrigger.onTrue(Commands.runOnce(intake::setIdleMode, intake));

        // Reset buttons
        Trigger visionResetTrigger = new Trigger(() -> m_Controls.getButton(4));
        visionResetTrigger.onTrue(
            Commands.defer(() -> {
                return Commands.runOnce(() -> vision.hardReset("limelight-br"), vision);
            }, Set.of(vision))
        );

        Trigger intakeResetTrigger = new Trigger(() -> m_Controls.getButton(3));
        intakeResetTrigger.onTrue(Commands.runOnce(intake::resetPositionCommand, intake));

        // Climb bindings
        Trigger climbTrigger = new Trigger(m_Controls::getClimbTrigger);
        Trigger climbReleaseTrigger = new Trigger(m_Controls::getClimbReleaseTrigger);
        Trigger climbResetTrigger = new Trigger(m_Controls::getClimbResetTrigger);

        climbTrigger.onTrue(Commands.runOnce(() -> climb.setState(ClimbState.PULL_UP), climb));
        climbReleaseTrigger.onTrue(Commands.runOnce(() -> climb.setState(ClimbState.RETRACTED), climb));
        climbResetTrigger.onTrue(Commands.runOnce(() -> climb.setState(ClimbState.EXTENDED), climb));

        // SysID bindings (using D-pad on driver controller) - COMMENTED OUT
        // // Hood SysID - D-pad Up/Down for quasistatic, hold B + D-pad for dynamic
        // driverXbox.povUp().and(driverXbox.b().negate())
        //     .whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driverXbox.povDown().and(driverXbox.b().negate())
        //     .whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driverXbox.povUp().and(driverXbox.b())
        //     .whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driverXbox.povDown().and(driverXbox.b())
        //     .whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // // Flywheel SysID - D-pad Left/Right for quasistatic, hold B + D-pad for dynamic
        // driverXbox.povRight().and(driverXbox.b().negate())
        //     .whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driverXbox.povLeft().and(driverXbox.b().negate())
        //     .whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driverXbox.povRight().and(driverXbox.b())
        //     .whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driverXbox.povLeft().and(driverXbox.b())
        //     .whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // // Turret SysID - hold RightBumper + D-pad for quasistatic, hold RightBumper + B + D-pad for dynamic
        // driverXbox.rightBumper().and(driverXbox.povUp()).and(driverXbox.b().negate())
        //     .whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driverXbox.rightBumper().and(driverXbox.povDown()).and(driverXbox.b().negate())
        //     .whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driverXbox.rightBumper().and(driverXbox.povUp()).and(driverXbox.b())
        //     .whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driverXbox.rightBumper().and(driverXbox.povDown()).and(driverXbox.b())
        //     .whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }


    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().selectedCommand();
    }

    public SwerveSubsystem getDrivebase() {
      return drivebase;
    }

    public AutoModeChooser getAutoChooser() {
      return autoChooser;
    }
}
