package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.commands.scoring.FuelShootCommand;
import frc.robot.utility.shooting.FuelShotModel;
import frc.robot.utility.shooting.LaunchCalibration;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.swervedrive.drivebase.AimAtHub;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIOAddressable;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driverXbox =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox =
        new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
    private final ManualControls m_Controls = new ManualControls(OperatorConstants.BUTTON_BOARD_PORT);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve/falcon"));
    private final VisionSubsystem vision;

    private final Shooter shooter;
    private final Intake intake;
    private final Indexer indexer;
    private final LED led;

    private FuelShootCommand shotCommand;
    private boolean pitCoastMode = false;
    private boolean manualControlsSuppressedAfterAim;
    private FuelShotModel fuelShotModel;
    private String fuelCalibrationStatus = "No launch measurements: use calibration mode";

    private final AutoCommands factory;
    private final AutoModeChooser autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1.0)
      .allianceRelativeControl(true);


    public RobotContainer() {
      // Initialize subsystems with appropriate IO implementations
      Constants.Mode indexerMode = Robot.isReal()
          ? Constants.Mode.REAL
          : Constants.currentMode == Constants.Mode.REPLAY
              ? Constants.Mode.REPLAY
              : Constants.Mode.SIM;

      indexer = switch (indexerMode) {
        case REAL -> new Indexer(new IndexerIOTalonFX(
            Constants.IndexerConstants.FLOOR_MOTOR_ID,
            Constants.IndexerConstants.FLOOR_SECONDARY_MOTOR_ID,
            Constants.IndexerConstants.TUNNEL_MOTOR_ID));
        case SIM -> new Indexer(new IndexerIOSim());
        case REPLAY -> new Indexer(new IndexerIO() {});
      };

      if (Robot.isReal()) {
        shooter = new Shooter(new ShooterIOTalonFX());


        intake = new Intake(new IntakeIOTalonFX());
        led = new LED(new LEDIOAddressable(3, 65), () -> shotCommand != null && shotCommand.isScheduled(),
            () -> shotCommand != null && shotCommand.isReadyToFeed());
      } else {
          shooter = new Shooter(new ShooterIOSim());


          intake = new Intake(new IntakeIOSim());
          led = new LED(new LEDIOSim(65), () -> shotCommand != null && shotCommand.isScheduled(),
            () -> shotCommand != null && shotCommand.isReadyToFeed());
      }

      factory = new AutoCommands(drivebase, intake, shooter);
      autoChooser = new AutoModeChooser(factory, indexer);
      SmartDashboard.putData("Auto Chooser", autoChooser.getAutoChooser());
      SmartDashboard.putBoolean("Pit Coast Mode", false);

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
        Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
        limelights.add(new LimelightVision(drivebase, name, cameraPose));
      }

      vision = new VisionSubsystem(limelights);
      Logger.recordOutput("Vision/ConfigurationID", LimelightConstants.CONFIGURATION_ID);
      Logger.recordOutput("Vision/ConfiguredCameraNames", LimelightConstants.LIMELIGHT_NAMES);
      Logger.recordOutput("Vision/ConfiguredCameraCount", limelights.size());
      SmartDashboard.putString("Vision/ConfigurationID", LimelightConstants.CONFIGURATION_ID);
      SmartDashboard.putNumber("Vision/ConfiguredCameraCount", limelights.size());

      // Temporary no-vision behavior: keep the flywheel warm at idle until a command takes it.
      shooter.setDefaultCommand(Commands.run(this::trackFlywheelTarget, shooter));
      led.setDefaultCommand(Commands.run(() -> {}, led));

      // Takes the shooter from its idle default until cancelled from the dashboard.
      SmartDashboard.putData("Shooter/FF Characterization", shooter.feedforwardCharacterizationCommand());

      fuelShotModel = new FuelShotModel(LaunchCalibration.ideal());
      fuelCalibrationStatus = "Ideal wheel/hood formula with tunable speed, spin and aero coefficients";
      SmartDashboard.putString("Fuel/Calibration Status", fuelCalibrationStatus);
      FuelShootCommand.publishLaunchSpeedCoefficient();
      SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
      SmartDashboard.putNumber("Fuel/Test Flywheel RPM", Constants.ShooterConstants.FIXED_SHOT_RPM);
      SmartDashboard.putNumber("Fuel/Test Tunnel RPM", Constants.IndexerConstants.TUNNEL_RUNNING_RPM);
      SmartDashboard.putBoolean("Fuel/Feeding", false);
      configureBindings();
    }


    private double nextFlywheelSolve;
    private double continuousFlywheelRPM = Constants.ShooterConstants.FIXED_SHOT_RPM;

    /** Teleop default: keep the wheel ready without requesting aim or feed. */
    private void trackFlywheelTarget() {
        if (!DriverStation.isTeleopEnabled() || pitCoastMode) {
            shooter.stop();
            return;
        }
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now >= nextFlywheelSolve) {
            nextFlywheelSolve = now + 0.1;
            fuelShotModel.setLaunchSpeedCoefficient(frc.robot.utility.shooting.LaunchSpeedTuning.update());
            var alliance = DriverStation.getAlliance();
            var hub = alliance.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? Constants.FieldConstants.BLUE_HUB_CENTER : Constants.FieldConstants.RED_HUB_CENTER;
            var solution = fuelShotModel.solve(FuelShootCommand.releasePosition(drivebase.getPose()).getDistance(hub));
            continuousFlywheelRPM = SmartDashboard.getBoolean("Fuel/Calibration Mode", false) || !solution.valid()
                ? SmartDashboard.getNumber("Fuel/Test Flywheel RPM", Constants.ShooterConstants.FIXED_SHOT_RPM)
                : solution.rpm();
        }
        if (Double.isFinite(continuousFlywheelRPM)
                && continuousFlywheelRPM >= Constants.ShooterConstants.FLYWHEEL_IDLE_RPM
                && continuousFlywheelRPM <= Constants.ShooterConstants.MAX_RPM) shooter.setGoal(continuousFlywheelRPM);
        else shooter.stop();
    }

    private static final double MAX_SHOOTING_VELOCITY_MPS = 1.2;

    private boolean isShotCommandActive() {
        return driverXbox.rightBumper().getAsBoolean()
            || (shotCommand != null
                && CommandScheduler.getInstance().isScheduled(shotCommand));
    }

    /** Gated shot plus BLine's AimAtHub, which turns the chassis to the hub while the driver translates. */
    private Command createAimedShotCommand() {
        AimAtHub aim = new AimAtHub(drivebase,
            () -> clampSpeedsForShooting(driveAngularVelocity.get()),
            () -> FuelShootCommand.releasePosition(drivebase.getPose()));
        shotCommand = new FuelShootCommand(shooter, indexer, drivebase::getPose,
            drivebase::getRobotVelocity, aim, aim::isAtTarget,
            () -> pitCoastMode, fuelShotModel, fuelCalibrationStatus);
        return shotCommand;
    }

    private void cancelShotCommand() {
        if (shotCommand != null) {
            CommandScheduler.getInstance().cancel(shotCommand);
        }
    }

    /** One owner samples the controls so simultaneous holds cannot cancel each other. */
    private Command manualShotCommand(Trigger spinUp, Trigger floor, Trigger feed) {
        return Commands.run(() -> {
            if (spinUp.getAsBoolean()) {
                shooter.setGoal(Constants.ShooterConstants.FIXED_SHOT_RPM);
            } else {
                shooter.setIdle();
            }
            indexer.setFloorRPM(floor.getAsBoolean()
                ? Constants.IndexerConstants.FLOOR_RUNNING_RPM : 0.0);
            indexer.setTunnelRPM(feed.getAsBoolean()
                ? Constants.IndexerConstants.TUNNEL_RUNNING_RPM : 0.0);
        }, shooter, indexer).beforeStarting(this::cancelShotCommand).finallyDo(interrupted -> {
            indexer.stop();
            shooter.setIdle();
        });
    }

    private ChassisSpeeds clampSpeedsForShooting(ChassisSpeeds speeds) {
        if (!isShotCommandActive()) {
            return speeds;
        }

        double maxVelocity = MAX_SHOOTING_VELOCITY_MPS;

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
        // Both indexers stay stopped unless an explicit command owns them.
        indexer.setDefaultCommand(Commands.run(indexer::stop, indexer));

        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
            () -> DriverStation.isTeleopEnabled()
                ? clampSpeedsForShooting(driveAngularVelocity.get())
                : new ChassisSpeeds());
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        driveFieldOrientedAnglularVelocity.setName("Driver field-oriented drive");

        // Driver right bumper owns aim/spin/feed. Left bumper retains standalone manual indexer feed.
        // Keep separate operator/button-board controls for explicit manual testing.
        Trigger spinUp = operatorXbox.a().or(new Trigger(() -> m_Controls.getButtonActivated(11)));
        Trigger driver2Indexer = new Trigger(() -> m_Controls.getButtonActivated(3));
        Trigger feed = driverXbox.leftBumper().or(driver2Indexer);
        Trigger floor = feed;

        // Triggers select rack endpoints without running the wall indexer.
        // Ignore simultaneous presses so neither direction wins unexpectedly.
        Trigger rackRetract = driverXbox.leftTrigger();
        Trigger rackExtend = driverXbox.rightTrigger();
        rackRetract.and(rackExtend.negate())
            .and(DriverStation::isTeleopEnabled).and(() -> !pitCoastMode)
            .onTrue(Commands.runOnce(intake::requestRackRetracted, intake));
        rackExtend.and(rackRetract.negate())
            .and(DriverStation::isTeleopEnabled).and(() -> !pitCoastMode)
            .onTrue(Commands.runOnce(() -> {
                intake.requestRackExtended();
                intake.setRollersRequested(true);
            }, intake))
            .onFalse(Commands.runOnce(() -> intake.setRollersRequested(false), intake));
        // Releasing right bumper must stop feeding even if an operator control is held.
        // Require all manual holds to release before an ungated manual shot can start again.
        new Trigger(() -> !driverXbox.rightBumper().getAsBoolean() && !spinUp.getAsBoolean()
            && !floor.getAsBoolean() && !feed.getAsBoolean())
            .onTrue(Commands.runOnce(() -> manualControlsSuppressedAfterAim = false).ignoringDisable(true));
        spinUp.or(floor).or(feed)
            .and(DriverStation::isTeleopEnabled)
            .and(() -> !pitCoastMode && !driverXbox.rightBumper().getAsBoolean() && !manualControlsSuppressedAfterAim)
            .whileTrue(manualShotCommand(spinUp, floor, feed));

        driverXbox.start().
          onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.a()
          .onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));

        // Driver Back/View toggles a zero-output coast mode so every mechanism can be moved by hand.
        driverXbox.back()
          .and(DriverStation::isTeleopEnabled)
          .onTrue(Commands.runOnce(
              this::togglePitCoastMode,
              drivebase, shooter, intake, indexer));

        // --- SHOTS ---
        
        Trigger driverAimedShootTrigger = driverXbox.rightBumper().and(DriverStation::isTeleopEnabled).and(() -> !pitCoastMode);

        // One held command owns drivetrain, shooter and indexer. Releasing RB cancels all three.
        Command aimedShot = createAimedShotCommand();
        driverAimedShootTrigger.onTrue(Commands.runOnce(() -> manualControlsSuppressedAfterAim = true));
        driverAimedShootTrigger.whileTrue(aimedShot);

        // Driver left bumper and button-board 3 request manual floor/tunnel feed outside the aimed shot.

        // --- RESETS ---
        Trigger resetPoseTrigger = driverXbox.x().and(DriverStation::isTeleopEnabled);
        resetPoseTrigger.onTrue(Commands.runOnce(() -> vision.hardReset(LimelightConstants.FRONT_CAMERA), vision));
    }

    private void togglePitCoastMode() {
        setPitCoastMode(!pitCoastMode);
    }

    private void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        drivebase.setPitCoastMode(enabled);
        shooter.setPitCoastMode(enabled);
        intake.setPitCoastMode(enabled);
        indexer.stop();

        SmartDashboard.putBoolean("Pit Coast Mode", enabled);
        Logger.recordOutput("Robot/PitCoastMode", enabled);
    }

    /** Restore normal motor control before an autonomous or teleop period begins. */
    public void disablePitCoastMode() {
        if (pitCoastMode) {
            setPitCoastMode(false);
        }
    }


    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().getSelected();
    }

    public SwerveSubsystem getDrivebase() {
      return drivebase;
    }

    public AutoModeChooser getAutoChooser() {
      return autoChooser;
    }
}
