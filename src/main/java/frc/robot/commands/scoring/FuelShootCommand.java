package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utility.shooting.FuelShotConfig;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.shooting.FuelShotModel;
import frc.robot.utility.shooting.FuelShotModel.Shot;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Owns aim, flywheel and the real tunnel for the entire held auto-turn shot. */
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
    public static final String LAUNCH_COEFFICIENT_KEY = "Shooter/Launch Speed Coefficient";
    private double launchSpeedCoefficient = 1.0;
    private boolean calibrationMode;
    private double testRPM, tunnelRPM, readySince;
    private double solvedRange;
    private Shot shot = Shot.invalid();

    public FuelShootCommand(Shooter shooter, Indexer indexer, Supplier<Pose2d> pose,
            Supplier<ChassisSpeeds> velocity, Command aim, BooleanSupplier aligned,
            BooleanSupplier pitCoast,
            FuelShotModel model, String calibrationStatus) {
        this.shooter = shooter; this.indexer = indexer; this.pose = pose; this.velocity = velocity;
        this.aim = aim; this.aligned = aligned;
        this.pitCoast = pitCoast; this.model = model; this.calibrationStatus = calibrationStatus;
        addRequirements(shooter, indexer);
        addRequirements(aim.getRequirements());
    }

    public static Translation2d releasePosition(Pose2d pose) {
        return pose.getTranslation().plus(new Translation2d(FuelShotConfig.RELEASE_FORWARD_METERS,
            FuelShotConfig.RELEASE_LEFT_METERS).rotateBy(pose.getRotation()));
    }

    @Override public void initialize() {
        calibrationMode = SmartDashboard.getBoolean("Fuel/Calibration Mode", false);
        testRPM = SmartDashboard.getNumber("Fuel/Test Flywheel RPM", Constants.ShooterConstants.FIXED_SHOT_RPM);
        tunnelRPM = calibrationMode || model == null
            ? SmartDashboard.getNumber("Fuel/Test Tunnel RPM", Constants.IndexerConstants.TUNNEL_RUNNING_RPM)
            : model.calibration().tunnelRPM();
        readySince = Double.NaN; solvedRange = Double.NaN; shot = Shot.invalid();
        indexer.stop();
        aim.initialize();
    }

    @Override public void execute() {
        double requested = SmartDashboard.getNumber(LAUNCH_COEFFICIENT_KEY, launchSpeedCoefficient);
        if (Double.isFinite(requested) && requested >= 0.5 && requested <= 1.5) {
            launchSpeedCoefficient = requested;
            if (model != null && model.setLaunchSpeedCoefficient(requested)) {
                solvedRange = Double.NaN;
                readySince = Double.NaN;
            }
        } else {
            SmartDashboard.putNumber(LAUNCH_COEFFICIENT_KEY, launchSpeedCoefficient);
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
        boolean stationary = finite && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            <= FuelShotConfig.MAX_TRANSLATION_MPS;
        if (!calibrationMode && model != null && finite
                && (!Double.isFinite(solvedRange) || Math.abs(range - solvedRange) > 0.02)) {
            shot = model.solve(range);
            solvedRange = range;
        }
        boolean fixedRPM = calibrationMode || model == null;
        // Accelerate while approaching; only the release gate waits for aim/position readiness.
        double rpm = fixedRPM || !shot.valid() ? testRPM : shot.rpm();
        boolean controlsValid = Double.isFinite(rpm) && rpm >= Constants.ShooterConstants.FLYWHEEL_IDLE_RPM
            && rpm <= Constants.ShooterConstants.MAX_RPM
            && Double.isFinite(tunnelRPM) && tunnelRPM > 0 && tunnelRPM <= 3000;
        boolean ownZone = alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Blue
            ? current.getX() < Constants.FieldConstants.BLUE_ZONE_X
            : current.getX() > Constants.FieldConstants.RED_ZONE_X);
        if (controlsValid && finite && !pitCoast.getAsBoolean() && DriverStation.isTeleopEnabled()) {
            shooter.setGoal(rpm); // Closed-loop velocity; no open-loop bang-bang during calibration.
        } else shooter.stop();
        String reason = !DriverStation.isTeleopEnabled() ? "Teleop disabled"
            : pitCoast.getAsBoolean() ? "Pit coast mode"
            : !finite ? "Invalid pose or velocity"
            : alliance.isEmpty() ? "No alliance"
            : !ownZone ? "Outside own alliance zone"
            : !controlsValid ? "Invalid RPM settings / no reachable shot"
            : !fixedRPM && !shot.valid() ? "No reachable shot"
            : !stationary ? "Stop translating to shoot"
            : !aligned.getAsBoolean() ? "Aiming"
            : !indexer.isTunnelReady() ? "Tunnel disconnected or configuration failed"
            : !shooter.isHardwareReady() ? "Flywheel disconnected or pit coast"
            : !shooter.atTarget() || !shooter.withinTolerance(FuelShotConfig.RPM_TOLERANCE)
                ? "Waiting for measured flywheel RPM"
            : !fixedRPM && !model.scores(range, shooter.getCurrentRPM()) ? "Measured RPM outside scoring band"
            : "Ready";
        boolean rawReady = reason.equals("Ready");
        double now = Timer.getFPGATimestamp();
        if (!rawReady) readySince = Double.NaN;
        else if (!Double.isFinite(readySince)) readySince = now;
        boolean feeding = rawReady && now - readySince >= FuelShotConfig.READY_DWELL_SECONDS;
        indexer.setFloorRPM(feeding ? Constants.IndexerConstants.FLOOR_RUNNING_RPM : 0);
        indexer.setTunnelRPM(feeding ? tunnelRPM : 0);
        String status = rawReady && !feeding ? "Settling" : reason;
        SmartDashboard.putString("Fuel/Mode", fixedRPM
            ? (calibrationMode ? "Fixed RPM measurement" : "Fixed RPM (uncalibrated)") : "Formula trajectory");
        SmartDashboard.putString("Fuel/Calibration Status", calibrationStatus);
        SmartDashboard.putString("Fuel/Status", status);
        SmartDashboard.putBoolean("Fuel/Feeding", feeding);
        SmartDashboard.putNumber("Fuel/Range M", range);
        SmartDashboard.putNumber("Fuel/Target RPM", rpm);
        SmartDashboard.putNumber("Fuel/Measured RPM", shooter.getCurrentRPM());
        SmartDashboard.putNumber("Fuel/Tunnel Target RPM", feeding ? tunnelRPM : 0);
        SmartDashboard.putNumber("Fuel/Minimum Scoring RPM", shot.minimumRPM());
        SmartDashboard.putNumber("Fuel/Maximum Scoring RPM", shot.maximumRPM());
        Logger.recordOutput("Fuel/Status", status);
        // Record each gate independently: Status shows the first blocker, not all blockers.
        Logger.recordOutput("Fuel/Readiness/AllianceKnown", alliance.isPresent());
        Logger.recordOutput("Fuel/Readiness/PoseAndVelocityFinite", finite);
        Logger.recordOutput("Fuel/Readiness/InOwnZone", ownZone);
        Logger.recordOutput("Fuel/Readiness/ShotReachable", fixedRPM || shot.valid());
        Logger.recordOutput("Fuel/Readiness/SettingsValid", controlsValid);
        Logger.recordOutput("Fuel/Readiness/StoppedTranslating", stationary);
        Logger.recordOutput("Fuel/Readiness/Aimed", aligned.getAsBoolean());
        Logger.recordOutput("Fuel/Readiness/TunnelReady", indexer.isTunnelReady());
        Logger.recordOutput("Fuel/Readiness/FlywheelReady", shooter.isHardwareReady());
        Logger.recordOutput("Fuel/Readiness/FlywheelAtSpeed", shooter.atTarget()
            && shooter.withinTolerance(FuelShotConfig.RPM_TOLERANCE));
        Logger.recordOutput("Fuel/Feeding", feeding);
        Logger.recordOutput("Fuel/CalibrationMode", calibrationMode);
        Logger.recordOutput("Fuel/RangeMeters", range);
        Logger.recordOutput("Fuel/TargetRPM", rpm);
        Logger.recordOutput("Fuel/MeasuredRPM", shooter.getCurrentRPM());
        Logger.recordOutput("Fuel/FlightSeconds", shot.flightSeconds());
        Logger.recordOutput("Fuel/TunnelMeasuredRPM", indexer.getTunnelRPM());
        Logger.recordOutput("Fuel/TunnelRequestedRPM", feeding ? tunnelRPM : 0);
    }

    @Override public void end(boolean interrupted) {
        aim.end(interrupted);
        indexer.stop(); shooter.stop();
        readySince = Double.NaN;
        SmartDashboard.putBoolean("Fuel/Feeding", false);
        SmartDashboard.putString("Fuel/Status", "Released");
        Logger.recordOutput("Fuel/Feeding", false);
        Logger.recordOutput("Fuel/Status", "Released");
    }
}
