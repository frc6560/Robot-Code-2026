package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoModeChooser;
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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.GamePieceVisionSystem;
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
    private final GamePieceVisionSystem gamePieceVision;

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

    private double assistSuppressedUntilSec = 0.0;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


    public RobotContainer() {
      // Initialize subsystems with appropriate IO implementations
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
      gamePieceVision = new GamePieceVisionSystem(LimelightConstants.GAMEPIECE_LIMELIGHT_NAME);

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
      ChassisSpeeds processed = speeds;

      if (isShotCommandActive()) {
        double maxVelocity = isInPassingZone() ? MAX_PASSING_VELOCITY_MPS : MAX_SHOOTING_VELOCITY_MPS;

        double vx = processed.vxMetersPerSecond;
        double vy = processed.vyMetersPerSecond;
        double translationSpeed = Math.hypot(vx, vy);

        if (translationSpeed > maxVelocity) {
          double scale = maxVelocity / translationSpeed;
          vx *= scale;
          vy *= scale;
        }

        processed = new ChassisSpeeds(vx, vy, processed.omegaRadiansPerSecond);
      }

      return applyHeadingAssist(processed);
    }

    private ChassisSpeeds applyHeadingAssist(ChassisSpeeds speeds) {
      double now = Timer.getFPGATimestamp();

      if (!Constants.VisionConstants.ENABLE_HEADING_ASSIST) {
        publishAssistTelemetry(0.0, 0.0, 0.0, false, "disabled", 0.0, false);
        return speeds;
      }

      boolean assistHeld = !Constants.VisionConstants.REQUIRE_ASSIST_HOLD || driverXbox.leftBumper().getAsBoolean();
      boolean freshTarget = gamePieceVision.hasFreshTarget(Constants.VisionConstants.ASSIST_TARGET_MAX_AGE_SEC);

      if (!assistHeld) {
        publishAssistTelemetry(0.0, 0.0, 0.0, false, "hold", 0.0, freshTarget);
        return speeds;
      }

      if (!freshTarget) {
        assistSuppressedUntilSec = now + Constants.VisionConstants.ASSIST_DISABLE_COOLDOWN_SEC;
        publishAssistTelemetry(0.0, 0.0, 0.0, false, "target", Math.max(0.0, assistSuppressedUntilSec - now), false);
        return speeds;
      }

      if (now < assistSuppressedUntilSec) {
        publishAssistTelemetry(0.0, 0.0, 0.0, false, "cooldown", Math.max(0.0, assistSuppressedUntilSec - now), true);
        return speeds;
      }

      double headingErrorDeg = gamePieceVision.getHeadingErrorDeg();
      double headingErrorRad = gamePieceVision.getHeadingErrorRad();

      double assistAlpha = computeAssistAlpha(Math.abs(headingErrorDeg));
      double driverScale = computeDriverOverrideScale(Math.abs(speeds.omegaRadiansPerSecond));
      assistAlpha *= driverScale;

      if (driverScale <= Constants.VisionConstants.DRIVER_OVERRIDE_DISABLE_SCALE) {
        assistSuppressedUntilSec = now + Constants.VisionConstants.ASSIST_DISABLE_COOLDOWN_SEC;
        publishAssistTelemetry(0.0, 0.0, headingErrorDeg, false, "override", Math.max(0.0, assistSuppressedUntilSec - now), true);
        return speeds;
      }

      if (assistAlpha < Constants.VisionConstants.ASSIST_MIN_ACTIVE_ALPHA) {
        publishAssistTelemetry(assistAlpha, 0.0, headingErrorDeg, false, "alpha", 0.0, true);
        return speeds;
      }

      double omegaAssist = Constants.VisionConstants.HEADING_ASSIST_SIGN
        * Constants.VisionConstants.HEADING_ASSIST_KP
        * headingErrorRad
        * assistAlpha;
      omegaAssist = MathUtil.clamp(
        omegaAssist,
        -Constants.VisionConstants.HEADING_ASSIST_MAX_OMEGA_RAD_PER_SEC,
        Constants.VisionConstants.HEADING_ASSIST_MAX_OMEGA_RAD_PER_SEC);

      double omegaFinal = MathUtil.clamp(
        speeds.omegaRadiansPerSecond + omegaAssist,
        -Constants.VisionConstants.HEADING_ASSIST_MAX_FINAL_OMEGA_RAD_PER_SEC,
        Constants.VisionConstants.HEADING_ASSIST_MAX_FINAL_OMEGA_RAD_PER_SEC);

      publishAssistTelemetry(assistAlpha, omegaAssist, headingErrorDeg, true, "active", 0.0, true);
      return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaFinal);
    }

    private static double computeAssistAlpha(double absHeadingErrorDeg) {
      double start = Constants.VisionConstants.ASSIST_ALPHA_START_ERROR_DEG;
      double full = Constants.VisionConstants.ASSIST_ALPHA_FULL_ERROR_DEG;
      if (full <= start) {
        return 0.0;
      }
      double ramp = (absHeadingErrorDeg - start) / (full - start);
      return MathUtil.clamp(ramp, 0.0, 1.0) * Constants.VisionConstants.ASSIST_ALPHA_MAX;
    }

    private static double computeDriverOverrideScale(double absDriverOmega) {
      double start = Constants.VisionConstants.DRIVER_OVERRIDE_START_OMEGA_RAD_PER_SEC;
      double full = Constants.VisionConstants.DRIVER_OVERRIDE_FULL_OMEGA_RAD_PER_SEC;
      if (full <= start) {
        return 1.0;
      }
      if (absDriverOmega <= start) {
        return 1.0;
      }
      if (absDriverOmega >= full) {
        return 0.0;
      }
      double t = (absDriverOmega - start) / (full - start);
      return 1.0 - MathUtil.clamp(t, 0.0, 1.0);
    }

    private void publishAssistTelemetry(
      double alpha,
      double omegaAssist,
      double headingErrorDeg,
      boolean active,
      String state,
      double cooldownRemainingSec,
      boolean freshTarget) {
      SmartDashboard.putBoolean("Vision/GamePiece/Assist/Active", active);
      SmartDashboard.putString("Vision/GamePiece/Assist/State", state);
      SmartDashboard.putNumber("Vision/GamePiece/Assist/Alpha", alpha);
      SmartDashboard.putNumber("Vision/GamePiece/Assist/OmegaAssistRadPerSec", omegaAssist);
      SmartDashboard.putNumber("Vision/GamePiece/Assist/HeadingErrorDeg", headingErrorDeg);
      SmartDashboard.putNumber("Vision/GamePiece/Assist/CooldownRemainingSec", cooldownRemainingSec);
      SmartDashboard.putBoolean("Vision/GamePiece/Assist/FreshTarget", freshTarget);
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
            () -> clampSpeedsForShooting(driveAngularVelocity.get()));
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        driverXbox.x()
          .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.start().
          onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));

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

    public GamePieceVisionSystem getGamePieceVision() {
      return gamePieceVision;
    }
}
