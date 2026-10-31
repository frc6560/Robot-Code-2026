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
import frc.robot.commands.scoring.BackUpAndShootCommand;
import frc.robot.utility.shooting.FuelShotModel;
import frc.robot.utility.shooting.LaunchCalibration;
import frc.robot.autonomous.AutoCommands;

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
import frc.robot.utility.telemetry.RateLimitedTelemetry;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driverXbox =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
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
      RateLimitedTelemetry.putBoolean("Pit Coast Mode", false);

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
        Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
        limelights.add(new LimelightVision(drivebase, name, cameraPose));
      }

      vision = new VisionSubsystem(limelights);
      RateLimitedTelemetry.recordOutput("Vision/ConfigurationID", LimelightConstants.CONFIGURATION_ID);
      RateLimitedTelemetry.recordOutput("Vision/ConfiguredCameraNames", LimelightConstants.LIMELIGHT_NAMES);
      RateLimitedTelemetry.recordOutput("Vision/ConfiguredCameraCount", limelights.size());
      RateLimitedTelemetry.putString("Vision/ConfigurationID", LimelightConstants.CONFIGURATION_ID);
      RateLimitedTelemetry.putNumber("Vision/ConfiguredCameraCount", limelights.size());

      // Hold 1000 RPM in teleop and auto until an explicit rev/shoot command takes ownership.
      shooter.setDefaultCommand(Commands.run(shooter::setIdle, shooter));
      led.setDefaultCommand(Commands.run(() -> {}, led));

      // Takes the shooter from its idle default until cancelled from the dashboard.
      SmartDashboard.putData("Shooter/FF Characterization", shooter.feedforwardCharacterizationCommand());

      fuelShotModel = new FuelShotModel(LaunchCalibration.ideal());
      fuelCalibrationStatus = "Ideal wheel/hood formula with tunable speed, spin and aero coefficients";
      RateLimitedTelemetry.putString("Fuel/Calibration Status", fuelCalibrationStatus);
      FuelShootCommand.publishLaunchSpeedCoefficient();
      RateLimitedTelemetry.putBoolean("Fuel/Calibration Mode", false);
      RateLimitedTelemetry.putNumber("Fuel/Test Flywheel RPM", Constants.ShooterConstants.FIXED_SHOT_RPM);
      RateLimitedTelemetry.putNumber("Fuel/Test Tunnel RPM", Constants.IndexerConstants.TUNNEL_RUNNING_RPM);
      RateLimitedTelemetry.putBoolean("Fuel/Feeding", false);
      autoChooser = new AutoModeChooser(factory, (name, red) -> factory.getCompetitionAuto(name, red, indexer,
          () -> pitCoastMode, fuelShotModel, fuelCalibrationStatus),
          new BackUpAndShootCommand(drivebase, shooter, indexer,
              fuelShotModel, () -> pitCoastMode, fuelCalibrationStatus, factory::followFieldPath));
      SmartDashboard.putData("Auto Chooser", autoChooser.getAutoChooser());
      configureBindings();
    }


    private static final double MAX_SHOOTING_VELOCITY_MPS = 1.2;

    private boolean isShotCommandActive() {
        return driverXbox.rightTrigger().getAsBoolean()
            || (shotCommand != null
                && CommandScheduler.getInstance().isScheduled(shotCommand));
    }

    /** Gated shot plus BLine's AimAtHub, which turns the chassis to the hub while the driver translates. */
    private Command createAimedShotCommand() {
        shotCommand = FuelShootCommand.aimedShot(drivebase, shooter, indexer,
            () -> clampSpeedsForShooting(driveAngularVelocity.get()),
            () -> pitCoastMode, fuelShotModel, fuelCalibrationStatus, false);
        return shotCommand;
    }

    private void cancelShotCommand() {
        if (shotCommand != null) {
            CommandScheduler.getInstance().cancel(shotCommand);
        }
    }

    /** One owner samples the controls so simultaneous holds cannot cancel each other. */
    private Command manualShotCommand(Trigger floor, Trigger feed) {
        return Commands.run(() -> {
            shooter.setIdle();
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

        // Driver right trigger owns aim/spin/feed. Left trigger retains standalone manual indexer feed.
        // Manual floor/tunnel controls retain idle; right trigger is the teleop spin-up request.
        Trigger driver2Indexer = new Trigger(() -> m_Controls.getButtonActivated(3));
        Trigger feed = driverXbox.leftTrigger().or(driver2Indexer);
        Trigger floor = feed;

        // Bumper presses select rack endpoints. Retraction takes priority on overlap.
        // Check the opposite bumper inside the action: composing it into the trigger
        // would turn a bumper RELEASE into a new press of the still-held bumper.
        Trigger rackRetract = driverXbox.leftBumper();
        Trigger rackExtend = driverXbox.rightBumper();
        rackRetract
            .and(DriverStation::isTeleopEnabled).and(() -> !pitCoastMode)
            .onTrue(Commands.runOnce(intake::requestRackRetracted, intake));
        rackExtend
            .and(DriverStation::isTeleopEnabled).and(() -> !pitCoastMode)
            .onTrue(Commands.runOnce(() -> {
                if (rackRetract.getAsBoolean()) return;
                intake.requestRackExtended();
                intake.setRollersRequested(true);
            }, intake))
            .onFalse(Commands.runOnce(() -> {
                // A teleop trigger falling on the mode transition must not cancel an auto
                // that now owns the intake, or clear its persistent intake event request.
                if (DriverStation.isTeleopEnabled()) intake.setRollersRequested(false);
            }));
        // Releasing right trigger must stop feeding even if an operator control is held.
        // Require all manual holds to release before an ungated manual shot can start again.
        new Trigger(() -> !driverXbox.rightTrigger().getAsBoolean()
            && !floor.getAsBoolean() && !feed.getAsBoolean())
            .onTrue(Commands.runOnce(() -> manualControlsSuppressedAfterAim = false).ignoringDisable(true));
        floor.or(feed)
            .and(DriverStation::isTeleopEnabled)
            .and(() -> !pitCoastMode && !driverXbox.rightTrigger().getAsBoolean() && !manualControlsSuppressedAfterAim)
            .whileTrue(manualShotCommand(floor, feed));

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
        
        Trigger driverAimedShootTrigger = driverXbox.rightTrigger().and(DriverStation::isTeleopEnabled).and(() -> !pitCoastMode);

        // One held command owns drivetrain, shooter and indexer. Releasing RT cancels all three.
        Command aimedShot = createAimedShotCommand();
        driverAimedShootTrigger.onTrue(Commands.runOnce(() -> manualControlsSuppressedAfterAim = true));
        driverAimedShootTrigger.whileTrue(aimedShot);

        // Driver left trigger and button-board 3 request manual floor/tunnel feed outside the aimed shot.

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

        RateLimitedTelemetry.putBoolean("Pit Coast Mode", enabled);
        RateLimitedTelemetry.recordOutput("Robot/PitCoastMode", enabled);
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

    private double nextShotDashboard;

    /** Passive Elastic readiness summary, available before any shoot-button press. */
    public void updateShotDashboard() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now < nextShotDashboard) return;
        nextShotDashboard = now + 0.5;
        var pose = drivebase.getPose();
        var alliance = DriverStation.getAlliance();
        var hub = alliance.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
            ? Constants.FieldConstants.BLUE_HUB_CENTER : Constants.FieldConstants.RED_HUB_CENTER;
        var delta = hub.minus(FuelShootCommand.releasePosition(pose));
        double range = delta.getNorm();
        boolean passing = alliance.isPresent() && frc.robot.utility.shooting.AlliancePass.outsideZone(alliance.get(), pose.getX());
        double heading = passing ? frc.robot.utility.shooting.AlliancePass.heading(alliance.get())
            : Math.atan2(delta.getY(), delta.getX());
        double error = edu.wpi.first.math.MathUtil.angleModulus(heading - pose.getRotation().getRadians());
        boolean known = alliance.isPresent() && Double.isFinite(range) && Double.isFinite(error);
        boolean reachable = known && (passing || fuelShotModel.solve(range).valid());
        boolean aligned = known && delta.getNorm() > 1e-6
            && Math.abs(error) <= (passing ? frc.robot.utility.shooting.AlliancePass.ANGLE_TOLERANCE
                : Constants.DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS)
            && Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond)
                <= Constants.DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC;
        SmartDashboard.putBoolean("Fuel/Passing Mode", passing);
        SmartDashboard.putBoolean("Fuel/Target Reachable", reachable);
        SmartDashboard.putBoolean("Fuel/Aligned", aligned);
        SmartDashboard.putNumber("Fuel/Alignment Error Deg", known ? Math.toDegrees(error) : Double.NaN);
        SmartDashboard.putNumber("Fuel/Range M", range);
    }

    public SwerveSubsystem getDrivebase() {
      return drivebase;
    }

    public AutoModeChooser getAutoChooser() {
      return autoChooser;
    }
}
