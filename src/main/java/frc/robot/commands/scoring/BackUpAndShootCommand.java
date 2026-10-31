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
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

/** Generates a B-Line retreat from the measured pose, then unloads through the normal shot gates. */
public final class BackUpAndShootCommand extends Command {
    public static final double REVERSE_SPEED_MPS = 1.0;
    public static final double TRANSLATION_KP = 4.0;
    public static final double ACCELERATION_MPS2 = 0.75;
    public static final double RETREAT_DISTANCE_METERS = 2.4;
    private static final double END_TOLERANCE_METERS = 0.03;
    public static final double FEED_SECONDS = 6.0;
    private static final double STOPPED_MPS = 0.05;
    private enum Phase { PREPARE, RETREAT, SETTLE, SHOOT, DONE, ABORTED }

    private final SwerveSubsystem drive;
    private final Shooter shooter;
    private final Indexer indexer;
    private final BooleanSupplier pitCoast;
    private final Function<Path, FollowPath> pathFactory;
    private final AimAtHub aim;
    private final FuelShootCommand shot;
    private Phase phase;
    private String abortReason = "";
    private Pose2d start;
    private Translation2d retreatDirection;
    private FollowPath retreat;
    private boolean retreatActive;
    private DriverStation.Alliance alliance;
    private double previousTime, feedSeconds;

    public BackUpAndShootCommand(SwerveSubsystem drive, Shooter shooter, Indexer indexer,
            FuelShotModel model, BooleanSupplier pitCoast, String calibrationStatus,
            Function<Path, FollowPath> pathFactory) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
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
        phase = Phase.PREPARE;
        abortReason = "";
        start = drive.getPose();
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        previousTime = Timer.getFPGATimestamp();
        feedSeconds = 0;
        retreat = null;
        retreatActive = false;
        retreatDirection = new Translation2d();
        shot.initialize();
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
        String blocked = !DriverStation.isAutonomousEnabled() ? "Autonomous is not enabled"
            : pitCoast.getAsBoolean() ? "Pit coast is enabled"
            : !finite ? "Invalid pose or velocity" : "";
        if (!blocked.isEmpty()) {
            abort(blocked);
            return;
        }
        if (phase == Phase.PREPARE) {
            if (retreat == null) {
                Pose2d end = findRetreatEnd(start, alliance);
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
            retreat.initialize();
            retreatActive = true;
            phase = Phase.RETREAT;
            retreat.execute();
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

    /** Always produce the fixed retreat, without field, zone, or predicted-shot rejection. */
    static Pose2d findRetreatEnd(Pose2d start, DriverStation.Alliance alliance) {
        Translation2d position = retreatEndPosition(start, alliance);
        return new Pose2d(position, start.getTranslation().minus(position).getAngle());
    }

    private static Translation2d retreatEndPosition(Pose2d start, DriverStation.Alliance alliance) {
        Translation2d away = start.getTranslation().minus(hub(alliance));
        if (away.getNorm() < 0.01) {
            away = new Translation2d(-start.getRotation().getCos(), -start.getRotation().getSin());
        }
        return start.getTranslation().plus(away.div(away.getNorm()).times(RETREAT_DISTANCE_METERS));
    }

    private static Translation2d hub(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
            ? FieldConstants.BLUE_HUB_CENTER : FieldConstants.RED_HUB_CENTER;
    }

    private double measuredSpeed() {
        ChassisSpeeds speed = drive.getRobotVelocity();
        return Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    }

    private double retreatProgress() {
        Translation2d offset = drive.getPose().getTranslation().minus(start.getTranslation());
        return offset.getX() * retreatDirection.getX() + offset.getY() * retreatDirection.getY();
    }

    private void abort(String reason) {
        abortReason = reason;
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
        RateLimitedTelemetry.putString("Auto/Back Up and Shoot/Abort Reason", abortReason);
        RateLimitedTelemetry.recordOutput("Auto/Back Up and Shoot/Abort Reason", abortReason);
        RateLimitedTelemetry.recordOutput("Auto/Back Up and Shoot/Start Pose", start);
        if (alliance != null) {
            Translation2d planned = retreatEndPosition(start, alliance);
            if (planned != null) {
                RateLimitedTelemetry.putNumber("Auto/Back Up and Shoot/Planned End X", planned.getX());
                RateLimitedTelemetry.putNumber("Auto/Back Up and Shoot/Planned End Y", planned.getY());
            }
        }
        RateLimitedTelemetry.putNumber("Auto/Back Up and Shoot/Feed Seconds", feedSeconds);
        RateLimitedTelemetry.putNumber("Auto/Back Up and Shoot/Retreat Meters", retreatProgress());
    }

    @Override public boolean isFinished() { return phase == Phase.DONE || phase == Phase.ABORTED; }
    @Override public void end(boolean interrupted) { stop(interrupted); }
}
