package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.drivebase.AimAtHub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.shooting.FuelShotConfig;
import frc.robot.utility.shooting.LaunchSpeedTuning;
import frc.robot.utility.shooting.AlliancePass;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.shooting.FuelShotModel;
import frc.robot.utility.shooting.FuelShotModel.Shot;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Owns aim, flywheel and indexers for a gated teleop or autonomous shot. */
public final class FuelShootCommand extends Command {
    private final Shooter shooter;
    private final Indexer indexer;
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> velocity;
    private final Command aim;
    private final BooleanSupplier aligned;
    private final BooleanSupplier pitCoast;
    private final FuelShotModel model;
    private final String calibrationStatus;
    private final boolean autonomous;
    private final BooleanSupplier feedAllowed;
    private boolean feeding;

    public boolean isFeeding() { return feeding; }
    public static final String LAUNCH_COEFFICIENT_KEY = LaunchSpeedTuning.REQUEST_KEY;
    public static void publishLaunchSpeedCoefficient() {
        LaunchSpeedTuning.initialize();
    }
    private double launchSpeedCoefficient = LaunchSpeedTuning.DEFAULT_COEFFICIENT;
    private boolean calibrationMode;
    private double nextTelemetry;
    private boolean readyToFeed;

    public boolean isReadyToFeed() { return readyToFeed; }
    private double testRPM, tunnelRPM, readySince;
    private double solvedRange;
    private double nextSolve;
    private final Shot shot = Shot.invalid();

    public FuelShootCommand(Shooter shooter, Indexer indexer, Supplier<Pose2d> pose,
            Supplier<ChassisSpeeds> velocity, Command aim, BooleanSupplier aligned,
            BooleanSupplier pitCoast,
            FuelShotModel model, String calibrationStatus) {
        this(shooter, indexer, pose, velocity, aim, aligned, pitCoast, model,
            calibrationStatus, false, () -> true);
    }

    /** Autonomous always requires a modeled trajectory, even if dashboard calibration is enabled. */
    public FuelShootCommand(Shooter shooter, Indexer indexer, Supplier<Pose2d> pose,
            Supplier<ChassisSpeeds> velocity, Command aim, BooleanSupplier aligned,
            BooleanSupplier pitCoast, FuelShotModel model, String calibrationStatus,
            boolean autonomous, BooleanSupplier feedAllowed) {
        this.autonomous = autonomous;
        this.feedAllowed = feedAllowed;
        this.shooter = shooter; this.indexer = indexer; this.pose = pose; this.velocity = velocity;
        this.aim = aim; this.aligned = aligned;
        this.pitCoast = pitCoast; this.model = model; this.calibrationStatus = calibrationStatus;
        addRequirements(shooter, indexer);
        addRequirements(aim.getRequirements());
    }

    /** Shared right-bumper/auto behavior: identical aim, trajectory, flywheel and indexer gates. */
    public static FuelShootCommand aimedShot(
            SwerveSubsystem drive, Shooter shooter, Indexer indexer,
            Supplier<ChassisSpeeds> translation, BooleanSupplier pitCoast, FuelShotModel model,
            String calibrationStatus, boolean autonomous) {
        var aim = new AimAtHub(drive, translation,
            () -> releasePosition(drive.getPose()));
        return new FuelShootCommand(shooter, indexer, drive::getPose, drive::getRobotVelocity,
            aim, aim::isAtTarget, pitCoast, model, calibrationStatus, autonomous, () -> true);
    }

    public static Translation2d releasePosition(Pose2d pose) {
        if (FuelShotConfig.RELEASE_FORWARD_METERS == 0 && FuelShotConfig.RELEASE_LEFT_METERS == 0)
            return pose.getTranslation();
        return pose.getTranslation().plus(new Translation2d(FuelShotConfig.RELEASE_FORWARD_METERS,
            FuelShotConfig.RELEASE_LEFT_METERS).rotateBy(pose.getRotation()));
    }

    @Override public void initialize() {
        calibrationMode = !autonomous && SmartDashboard.getBoolean("Fuel/Calibration Mode", false);
        testRPM = SmartDashboard.getNumber("Fuel/Test Flywheel RPM", Constants.ShooterConstants.FIXED_SHOT_RPM);
        tunnelRPM = calibrationMode || model == null
            ? SmartDashboard.getNumber("Fuel/Test Tunnel RPM", Constants.IndexerConstants.TUNNEL_RUNNING_RPM)
            : model.calibration().tunnelRPM();
        feeding = false;
        readyToFeed = false;
        readySince = Double.NaN; solvedRange = Double.NaN; shot.clear();
        indexer.stop();
        aim.initialize();
    }

    @Override public void execute() {
        launchSpeedCoefficient = LaunchSpeedTuning.update();
        if (model != null && model.setLaunchSpeedCoefficient(launchSpeedCoefficient)) {
            solvedRange = Double.NaN;
            readySince = Double.NaN;
        }
        Pose2d current = pose.get();
        ChassisSpeeds speeds = velocity.get();
        boolean finite = Double.isFinite(current.getX()) && Double.isFinite(current.getY())
            && Double.isFinite(current.getRotation().getRadians())
            && Double.isFinite(speeds.vxMetersPerSecond) && Double.isFinite(speeds.vyMetersPerSecond)
            && Double.isFinite(speeds.omegaRadiansPerSecond);
        // Do not send NaN into the heading controller or drivetrain.
        if (finite) aim.execute(); else aim.end(true);
        var alliance = DriverStation.getAlliance();
        Translation2d hub = alliance.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
            ? Constants.FieldConstants.BLUE_HUB_CENTER : Constants.FieldConstants.RED_HUB_CENTER;
        double range = releasePosition(current).getDistance(hub);
        boolean passing = !autonomous && alliance.isPresent() && finite
            && AlliancePass.outsideZone(alliance.get(), current.getX());
        boolean stationary = finite && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            <= FuelShotConfig.MAX_TRANSLATION_MPS;
        if (!passing && !calibrationMode && model != null && finite
                && (!Double.isFinite(solvedRange) || (Timer.getFPGATimestamp() >= nextSolve && Math.abs(range - solvedRange) > 0.02))) {
            model.solveInto(range, shot);
            solvedRange = range;
            nextSolve = Timer.getFPGATimestamp() + 0.1;
        }
        boolean fixedRPM = !autonomous && (calibrationMode || model == null);
        // Accelerate while approaching; only the release gate waits for aim/position readiness.
        double rpm = passing ? AlliancePass.RPM : fixedRPM || !shot.valid() ? testRPM : shot.rpm();
        boolean controlsValid = Double.isFinite(rpm) && rpm >= Constants.ShooterConstants.FLYWHEEL_IDLE_RPM
            && rpm <= Constants.ShooterConstants.MAX_RPM
            && Double.isFinite(tunnelRPM) && tunnelRPM > 0 && tunnelRPM <= 3000;
        boolean ownZone = alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Blue
            ? current.getX() < Constants.FieldConstants.BLUE_ZONE_X
            : current.getX() > Constants.FieldConstants.RED_ZONE_X);
        if (controlsValid && finite && !pitCoast.getAsBoolean() && modeEnabled()) {
            shooter.setGoal(rpm); // Closed-loop velocity; no open-loop bang-bang during calibration.
        } else shooter.stop();
        String reason = !modeEnabled() ? "Shooting mode disabled"
            : pitCoast.getAsBoolean() ? "Pit coast mode"
            : !finite ? "Invalid pose or velocity"
            : alliance.isEmpty() ? "No alliance"
            : !ownZone && !passing ? "Outside own alliance zone"
            : !controlsValid ? "Invalid RPM settings / no reachable shot"
            : !passing && !fixedRPM && !shot.valid() ? "No reachable shot: " + (model == null ? "No model" : model.diagnostics().reason())
            : !feedAllowed.getAsBoolean() ? "Waiting for autonomous stop"
            : !stationary ? "Stop translating to shoot"
            : !aligned.getAsBoolean() ? "Aiming"
            : !indexer.isTunnelReady() ? "Tunnel disconnected or configuration failed"
            : !shooter.isHardwareReady() ? "Flywheel disconnected or pit coast"
            : !shooter.withinTolerance(FuelShotConfig.RPM_TOLERANCE)
                || (!passing && !shooter.atTarget())
                ? "Waiting for measured flywheel RPM"
            : !passing && !fixedRPM && !model.scores(range, shooter.getCurrentRPM()) ? "Measured RPM outside scoring band"
            : "Ready";
        boolean rawReady = reason.equals("Ready");
        double now = Timer.getFPGATimestamp();
        if (!rawReady) readySince = Double.NaN;
        else if (!Double.isFinite(readySince)) readySince = now;
        readyToFeed = rawReady;
        feeding = rawReady && now - readySince >= FuelShotConfig.READY_DWELL_SECONDS;
        indexer.setFloorRPM(feeding ? Constants.IndexerConstants.FLOOR_RUNNING_RPM : 0);
        indexer.setTunnelRPM(feeding ? tunnelRPM : 0);
        String status = rawReady && !feeding ? "Settling" : reason;
        // Release gates run every loop; telemetry is limited to two updates per second.
        if (edu.wpi.first.wpilibj.RobotBase.isReal() && now < nextTelemetry) return;
        nextTelemetry = now + 0.5;
        SmartDashboard.putNumber("Shooter/Applied Launch Speed Coefficient", launchSpeedCoefficient);
        SmartDashboard.putBoolean("Fuel/Passing Mode", passing);
        SmartDashboard.putString("Fuel/Mode", passing ? "Alliance wall pass" : fixedRPM
            ? (calibrationMode ? "Fixed RPM measurement" : "Fixed RPM (uncalibrated)") : "Formula trajectory");
        SmartDashboard.putString("Fuel/Calibration Status", calibrationStatus);
        SmartDashboard.putString("Fuel/Status", status);
        String trajectoryStatus = passing ? "Fixed RPM; alliance wall pass"
            : fixedRPM ? "Fixed RPM; trajectory model bypassed"
            : model == null ? "No trajectory model" : model.diagnostics().reason();
        SmartDashboard.putString("Fuel/Trajectory Status", trajectoryStatus);
        SmartDashboard.putString("Fuel/Trajectory/Status", trajectoryStatus);
        if (!passing && !fixedRPM && model != null) {
            var diagnostic = model.diagnostics();
            SmartDashboard.putNumber("Fuel/Trajectory/Sampled", diagnostic.sampled());
            SmartDashboard.putNumber("Fuel/Trajectory/ReachedHeight", diagnostic.reachedHeight());
            SmartDashboard.putNumber("Fuel/Trajectory/ClearedNearRim", diagnostic.clearedNearRim());
            SmartDashboard.putNumber("Fuel/Trajectory/LandedInOpening", diagnostic.landedInOpening());
            SmartDashboard.putNumber("Fuel/Trajectory/PassedEntryAngle", diagnostic.passedEntryAngle());
        }
        SmartDashboard.putBoolean("Fuel/Feeding", feeding);
        SmartDashboard.putNumber("Fuel/Range M", range);
        SmartDashboard.putNumber("Fuel/Target RPM", rpm);
        SmartDashboard.putNumber("Fuel/Measured RPM", shooter.getCurrentRPM());
        SmartDashboard.putNumber("Fuel/Tunnel Target RPM", feeding ? tunnelRPM : 0);
        SmartDashboard.putNumber("Fuel/Minimum Scoring RPM", shot.minimumRPM());
        SmartDashboard.putNumber("Fuel/Maximum Scoring RPM", shot.maximumRPM());
        // Record each gate independently: Status shows the first blocker, not all blockers.
        SmartDashboard.putBoolean("Fuel/Readiness/AllianceKnown", alliance.isPresent());
        SmartDashboard.putBoolean("Fuel/Readiness/PoseAndVelocityFinite", finite);
        SmartDashboard.putBoolean("Fuel/Readiness/InOwnZone", ownZone);
        SmartDashboard.putBoolean("Fuel/Readiness/ShotReachable", fixedRPM || shot.valid());
        SmartDashboard.putBoolean("Fuel/Readiness/SettingsValid", controlsValid);
        SmartDashboard.putBoolean("Fuel/Readiness/StoppedTranslating", stationary);
        SmartDashboard.putBoolean("Fuel/Readiness/Aimed", aligned.getAsBoolean());
        SmartDashboard.putBoolean("Fuel/Readiness/TunnelReady", indexer.isTunnelReady());
        SmartDashboard.putBoolean("Fuel/Readiness/FlywheelReady", shooter.isHardwareReady());
        SmartDashboard.putBoolean("Fuel/Readiness/FlywheelAtSpeed", shooter.atTarget()
            && shooter.withinTolerance(FuelShotConfig.RPM_TOLERANCE));
        SmartDashboard.putBoolean("Fuel/Feeding", feeding);
        SmartDashboard.putBoolean("Fuel/CalibrationMode", calibrationMode);
        SmartDashboard.putNumber("Fuel/RangeMeters", range);
        SmartDashboard.putNumber("Fuel/TargetRPM", rpm);
        SmartDashboard.putNumber("Fuel/MeasuredRPM", shooter.getCurrentRPM());
        SmartDashboard.putNumber("Fuel/FlightSeconds", shot.flightSeconds());
        SmartDashboard.putNumber("Fuel/TunnelMeasuredRPM", indexer.getTunnelRPM());
        SmartDashboard.putNumber("Fuel/TunnelRequestedRPM", feeding ? tunnelRPM : 0);
    }

    private boolean modeEnabled() {
        return autonomous ? DriverStation.isAutonomousEnabled() : DriverStation.isTeleopEnabled();
    }

    @Override public void end(boolean interrupted) {
        feeding = false;
        readyToFeed = false;
        aim.end(interrupted);
        indexer.stop(); shooter.stop();
        readySince = Double.NaN;
        SmartDashboard.putBoolean("Fuel/Feeding", false);
        SmartDashboard.putString("Fuel/Status", "Released");
    }
}
