package frc.robot.commands.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.swervedrive.drivebase.AimAtHub;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.shooting.FuelShotModel;
import frc.robot.utility.shooting.LaunchSpeedTuning;
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

/** Generates a B-Line retreat from the measured pose, then unloads through the normal shot gates. */
public final class BackUpAndShootCommand extends Command {
    public static final double REVERSE_SPEED_MPS = 1.0;
    public static final double ACCELERATION_MPS2 = 0.5;
    public static final double RETREAT_DISTANCE_METERS = 2.4;
    private static final double END_TOLERANCE_METERS = 0.03;
    public static final double FEED_SECONDS = 6.0;
    public static final double AUTO_TIMEOUT_SECONDS = 15.0;
    private static final double FIELD_MARGIN_METERS = 0.5;
    private static final double STOPPED_MPS = 0.05;
    private enum Phase { AIM, RETREAT, SETTLE, SHOOT, DONE, ABORTED }

    private final SwerveSubsystem drive;
    private final Shooter shooter;
    private final Indexer indexer;
    private final FuelShotModel model;
    private final BooleanSupplier pitCoast;
    private final Function<Path, FollowPath> pathFactory;
    private final AimAtHub aim;
    private final FuelShootCommand shot;
    private Phase phase;
    private Pose2d start;
    private Translation2d retreatDirection;
    private FollowPath retreat;
    private boolean retreatActive;
    private DriverStation.Alliance alliance;
    private double started, previousTime, feedSeconds;

    public BackUpAndShootCommand(SwerveSubsystem drive, Shooter shooter, Indexer indexer,
            FuelShotModel model, BooleanSupplier pitCoast, String calibrationStatus,
            Function<Path, FollowPath> pathFactory) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.model = model;
        this.pitCoast = pitCoast;
        this.pathFactory = pathFactory;
        aim = new AimAtHub(drive, ChassisSpeeds::new,
            () -> FuelShootCommand.releasePosition(drive.getPose()));
        // This command selects either B-Line or hub aim as the drivetrain owner each cycle.
        // FuelShootCommand retains every release gate, but does not issue a second drive output.
        shot = new FuelShootCommand(shooter, indexer, drive::getPose, drive::getRobotVelocity,
            new Command() {}, aim::isAtTarget, pitCoast, model, calibrationStatus, true,
            () -> phase == Phase.SHOOT && measuredSpeed() <= STOPPED_MPS);
        addRequirements(drive);
        addRequirements(shot.getRequirements());
    }

    @Override public void initialize() {
        phase = Phase.AIM;
        start = drive.getPose();
        alliance = DriverStation.getAlliance().orElse(null);
        started = previousTime = Timer.getFPGATimestamp();
        feedSeconds = 0;
        retreat = null;
        retreatActive = false;
        retreatDirection = new Translation2d();
        shot.initialize();
        aim.initialize();
        publish();
    }

    @Override public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = MathUtil.clamp(now - previousTime, 0, 0.05);
        previousTime = now;
        Pose2d pose = drive.getPose();
        ChassisSpeeds measured = drive.getRobotVelocity();
        boolean finite = Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians())
            && Double.isFinite(measured.vxMetersPerSecond)
            && Double.isFinite(measured.vyMetersPerSecond)
            && Double.isFinite(measured.omegaRadiansPerSecond);
        if (!DriverStation.isAutonomousEnabled() || pitCoast.getAsBoolean() || !finite
                || alliance == null || DriverStation.getAlliance().orElse(null) != alliance
                || !allowedPosition(pose.getTranslation(), alliance)
                || now - started >= AUTO_TIMEOUT_SECONDS) {
            abort();
            return;
        }
        model.setLaunchSpeedCoefficient(LaunchSpeedTuning.update());
        if (phase == Phase.AIM) {
            if (retreat == null) {
                Pose2d end = findRetreatEnd(start, alliance, model);
                if (end == null) { abort(); return; }
                Translation2d displacement = end.getTranslation().minus(start.getTranslation());
                retreatDirection = displacement.div(displacement.getNorm());
                Path path = new Path(new Path.PathConstraints()
                    .setMaxVelocityMetersPerSec(REVERSE_SPEED_MPS)
                    .setMaxAccelerationMetersPerSec2(ACCELERATION_MPS2)
                    .setEndTranslationToleranceMeters(END_TOLERANCE_METERS),
                    new Path.Waypoint(start.getTranslation(), end.getRotation(), false),
                    new Path.Waypoint(end, false));
                retreat = pathFactory.apply(path);
            }
            aim.execute();
            if (aim.isAtTarget()) {
                aim.end(false);
                retreat.initialize();
                retreatActive = true;
                phase = Phase.RETREAT;
            }
        } else if (phase == Phase.RETREAT) {
            retreat.execute();
            if (retreat.isFinished()) {
                retreat.end(false);
                retreatActive = false;
                phase = Phase.SETTLE;
                aim.initialize();
            }
        } else {
            aim.execute();
            if (phase == Phase.SETTLE && measuredSpeed() <= STOPPED_MPS) {
                double range = FuelShootCommand.releasePosition(pose).getDistance(hub(alliance));
                // Confirm the measured retreat agrees with the fixed endpoint before feeding.
                if (Math.abs(retreatProgress() - RETREAT_DISTANCE_METERS) > END_TOLERANCE_METERS + 1e-6
                        || !model.solve(range).valid()) {
                    abort();
                    return;
                }
                phase = Phase.SHOOT;
            }
        }
        if (phase == Phase.SHOOT) shot.execute();
        else { shooter.setIdle(); indexer.stop(); }
        if (shot.isFeeding()) feedSeconds += dt;
        if (feedSeconds >= FEED_SECONDS) {
            phase = Phase.DONE;
            stop(false);
        }
        publish();
    }

    /** Fixed total retreat along the hub-to-robot ray; never extend travel to find a shot. */
    static Pose2d findRetreatEnd(Pose2d start, DriverStation.Alliance alliance, FuelShotModel model) {
        Translation2d away = start.getTranslation().minus(hub(alliance));
        if (!Double.isFinite(away.getNorm()) || away.getNorm() < 0.01) return null;
        away = away.div(away.getNorm());
        Translation2d position = start.getTranslation().plus(away.times(RETREAT_DISTANCE_METERS));
        if (!allowedPosition(position, alliance)) return null;
        Pose2d end = new Pose2d(position, away.unaryMinus().getAngle());
        double range = FuelShootCommand.releasePosition(end).getDistance(hub(alliance));
        return model.solve(range).valid() && model.solve(range - 0.10).valid() ? end : null;
    }

    private static Translation2d hub(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
            ? FieldConstants.BLUE_HUB_CENTER : FieldConstants.RED_HUB_CENTER;
    }

    private static boolean allowedPosition(Translation2d position, DriverStation.Alliance alliance) {
        return position.getX() >= FIELD_MARGIN_METERS
            && position.getX() <= FieldConstants.FIELD_LENGTH - FIELD_MARGIN_METERS
            && position.getY() >= FIELD_MARGIN_METERS
            && position.getY() <= FieldConstants.FIELD_WIDTH - FIELD_MARGIN_METERS
            && (alliance == DriverStation.Alliance.Blue
                ? position.getX() < FieldConstants.BLUE_ZONE_X
                : position.getX() > FieldConstants.RED_ZONE_X);
    }

    private double measuredSpeed() {
        ChassisSpeeds speed = drive.getRobotVelocity();
        return Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    }

    private double retreatProgress() {
        Translation2d offset = drive.getPose().getTranslation().minus(start.getTranslation());
        return offset.getX() * retreatDirection.getX() + offset.getY() * retreatDirection.getY();
    }

    private void abort() {
        phase = Phase.ABORTED;
        stop(true);
        publish();
    }

    private void stop(boolean interrupted) {
        if (retreatActive) {
            retreat.end(interrupted);
            retreatActive = false;
        }
        aim.end(interrupted);
        shot.end(interrupted);
        drive.setChassisSpeeds(new ChassisSpeeds());
    }

    private void publish() {
        RateLimitedTelemetry.putString("Auto/Back Up and Shoot/Phase", phase.name());
        RateLimitedTelemetry.putNumber("Auto/Back Up and Shoot/Feed Seconds", feedSeconds);
        RateLimitedTelemetry.putNumber("Auto/Back Up and Shoot/Retreat Meters", retreatProgress());
    }

    @Override public boolean isFinished() { return phase == Phase.DONE || phase == Phase.ABORTED; }
    @Override public void end(boolean interrupted) { stop(interrupted); }
}
