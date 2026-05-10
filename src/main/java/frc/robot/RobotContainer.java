package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.scoring.ShotCommand;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.periodic.SuperstructureCommand;
import frc.robot.utility.Shooter.PassCalculator;
import frc.robot.utility.Shooter.ShotCalculator;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOAddressable;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final ManualControls m_Controls = new ManualControls(1);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase;
    private final VisionSubsystem vision;

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final Feeder feeder;
    private final Intake intake;
    private final LED led;

    private final ShotCalculator shotCalculator = new ShotCalculator();
    private final PassCalculator passCalculator = new PassCalculator();
    private Command shotCommand;

    private final AutoCommands factory;
    private final AutoModeChooser autoChooser;


    public RobotContainer() {
      // Initialize drive subsystem based on mode
      switch (Constants.currentMode) {
        case REAL:
          drivebase = new SwerveSubsystem(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
          break;
        case SIM:
          var simModules = new ModuleIOSim[] {
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight)
          };
          final SwerveSubsystem[] simDriveHolder = new SwerveSubsystem[1];
          simDriveHolder[0] = new SwerveSubsystem(
              new GyroIOSim((java.util.function.DoubleSupplier) () -> simDriveHolder[0] != null ? simDriveHolder[0].getCommandedOmegaRadPerSec() : 0.0),
              simModules[0], simModules[1], simModules[2], simModules[3]);
          drivebase = simDriveHolder[0];
          break;
        default:
          drivebase = new SwerveSubsystem(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
          break;
      }

      // Initialize other subsystems
      if (Robot.isReal()) {
        hood = new Hood(new HoodIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        led = new LED(new LEDIOAddressable(3, 65), hood, shooter, turret, shotCalculator);
      } else {
          hood = new Hood(new HoodIO() {});
          shooter = new Shooter(new ShooterIO() {});
          turret = new Turret(new TurretIO() {});
          feeder = new Feeder(new FeederIO() {});
          intake = new Intake(new IntakeIO() {});
          led = new LED(new LEDIO() {}, hood, shooter, turret, shotCalculator);
      }

      factory = new AutoCommands(drivebase, feeder, intake, shooter);
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
      led.setDefaultCommand(Commands.run(() -> {}, led));               

      configureBindings();
    }


    private static final double MAX_SHOOTING_VELOCITY_MPS = 1.2;
    private static final double MAX_PASSING_VELOCITY_MPS = 3.0;

    private boolean isShotCommandActive() {
        return driverXbox.rightBumper().getAsBoolean();
    }

    private boolean isInPassingZone() {
        double poseX = drivebase.getPose().getX();
        return poseX > Constants.FieldConstants.BLUE_ZONE_X && poseX < Constants.FieldConstants.RED_ZONE_X;
    }

    private ChassisSpeeds clampSpeedsForShooting(ChassisSpeeds speeds) {
        if (!isShotCommandActive()) {
            return speeds;
        }

        double maxVelocity = isInPassingZone() ? MAX_PASSING_VELOCITY_MPS : MAX_SHOOTING_VELOCITY_MPS;

        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double translationSpeed = Math.hypot(vx, vy);

        if (translationSpeed > maxVelocity) {
            double scale = maxVelocity / translationSpeed;
            vx *= scale;
            vy *= scale;
        }

        return new ChassisSpeeds(vx, vy, speeds.omegaRadiansPerSecond);
    }

    private void configureBindings() {
        drivebase.setDefaultCommand(
            DriveCommands.driveFieldOriented(
                drivebase,
                () -> -driverXbox.getLeftY(),
                () -> -driverXbox.getLeftX(),
                () -> -driverXbox.getRightX()));

        driverXbox.x().onTrue(Commands.runOnce(drivebase::lock, drivebase));
        driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));

        // --- SHOTS ---
        
        Trigger shootTrigger = new Trigger(m_Controls::getShootTrigger);
        Trigger shootReleaseTrigger = new Trigger(m_Controls::getShootReleaseTrigger);
        Trigger ungatedShootTrigger = new Trigger(m_Controls::getUngatedShootTrigger);

        shootTrigger.onTrue(Commands.runOnce(() -> {
          shotCommand = new ShotCommand(feeder, turret, hood, shooter, shotCalculator, drivebase::getPose, led);
          shotCommand.schedule();
        }));
        shootReleaseTrigger.onTrue(Commands.runOnce(() -> {
          if (shotCommand != null) {
            shotCommand.cancel();
          }
        }));
        ungatedShootTrigger.onTrue(Commands.sequence(Commands.runOnce(() -> {
          if (shotCommand != null) {
            shotCommand.cancel();
          }
        }), Commands.runOnce(() -> feeder.setShooting(true))));
        ungatedShootTrigger.onFalse(Commands.runOnce(() -> feeder.setShooting(false)));

        // --- INTAKE ---

        Trigger intakeTrigger = new Trigger(m_Controls::getRollerTrigger);
        Trigger intakeReleaseTrigger = new Trigger(m_Controls::getRollerReleaseTrigger);

        intakeTrigger.onTrue(Commands.runOnce(() -> {
            intake.activate();
            feeder.setIntaking(true);
        }));
        intakeReleaseTrigger.onTrue(Commands.runOnce(() -> {
            intake.deactivate();
            feeder.setIntaking(false);
        }));

        // --- OUTTAKE/DEJAM ---
        driverXbox.b().onTrue(Commands.runOnce(() -> {
            intake.activateOuttake();
            feeder.setOuttaking(true);
        }));
        driverXbox.b().onFalse(Commands.runOnce(() -> {
            intake.deactivate();
            feeder.setOuttaking(false);
        }));

        // --- RESETS ---
        Trigger resetPoseTrigger = new Trigger(m_Controls::getVisionResetTrigger);
        resetPoseTrigger.onTrue(Commands.runOnce(() -> vision.hardReset("limelight-br"), vision));
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
