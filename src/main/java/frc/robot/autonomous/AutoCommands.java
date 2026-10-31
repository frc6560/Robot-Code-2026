package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.Constants.BLineConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import frc.robot.commands.scoring.FuelShootCommand;
import frc.robot.utility.shooting.FuelShotModel;


/** Creates autonomous commands: BLine path following and the fixed-speed center autonomous. */
public class AutoCommands {
    private static final double BLINE_WAIT_EVENT_SECONDS = 5.0;

    private final SwerveSubsystem drivetrain;
    private final Intake intake;
    private final Shooter shooter;
    private final FollowPath.Builder firstPathBuilder;
    private double bLinePauseUntilSeconds;

    public AutoCommands(SwerveSubsystem drivetrain,
        Intake intake,
        Shooter shooter
    ) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.shooter = shooter;

        configureBLineLogging();

        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
            BLineConstants.GLOBAL_MAX_VELOCITY_MPS,
            BLineConstants.GLOBAL_MAX_ACCELERATION_MPS2,
            BLineConstants.GLOBAL_MAX_ANGULAR_VELOCITY_DEG_PER_SEC,
            BLineConstants.GLOBAL_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2,
            BLineConstants.END_TRANSLATION_TOLERANCE_METERS,
            BLineConstants.END_ROTATION_TOLERANCE_DEGREES,
            BLineConstants.INTERMEDIATE_HANDOFF_RADIUS_METERS));

        firstPathBuilder = createPathBuilder().withPoseReset(drivetrain::resetOdometry);

        registerLegacyEvents();
    }

    private void registerLegacyEvents() {
        FollowPath.registerEventTrigger("intake", intake());
        FollowPath.registerEventTrigger("retract", retract());
        FollowPath.registerEventTrigger("rev", (Runnable) () -> {});
        FollowPath.registerEventTrigger("shoot", (Runnable) () -> {});
        FollowPath.registerEventTrigger("wait", this::startBLineWait);
        FollowPath.registerEventTrigger("Wait", this::startBLineWait);
    }

    public Command getCompetitionAuto(String name, boolean red, Indexer indexer,
            BooleanSupplier pitCoast,
            FuelShotModel model, String calibrationStatus) {
        var shoot = FuelShootCommand.aimedShot(drivetrain, shooter,
            indexer, ChassisSpeeds::new, pitCoast, model, calibrationStatus, true);
        var rev = new FuelShootCommand(shooter, indexer,
            drivetrain::getPose, drivetrain::getRobotVelocity, Commands.none(), () -> false,
            pitCoast, model, calibrationStatus, true, () -> false);
        // These six chooser entries are already authored in their selected field frame.
        // Disable both automatic transforms to avoid mirroring or rotating red a second time.
        var follower = createPathBuilder().withShouldFlip(() -> false)
            .withShouldMirror(() -> false).withPoseReset(drivetrain::resetOdometry)
            .build(competitionPath(name, red));
        var auto = new EventPathAuto(follower, rev, shoot,
            shooter, indexer, intake, pitCoast, this::registerLegacyEvents);
        auto.setName((red ? "Red " : "Blue ") + switch (name) {
                case "non-human" -> "Left";
                case "hub" -> "Center";
                case "human" -> "Right";
                default -> throw new IllegalArgumentException("Unknown competition path: " + name);
            });
        return auto;
    }

    /** Rotate every geometric target 180 degrees about field center; keep order/events/constraints. */
    public static Path competitionPath(String name, boolean red) {
        Path path = new Path(name);
        if (red) {
            path.setPathElements(path.getPathElements().stream()
                .map(AutoCommands::rotateHalfTurn).toList());
        }
        return path;
    }

    private static Path.PathElement rotateHalfTurn(Path.PathElement element) {
        if (element instanceof Path.Waypoint waypoint) {
            return new Path.Waypoint(
                (Path.TranslationTarget) rotateHalfTurn(waypoint.translationTarget()),
                (Path.RotationTarget) rotateHalfTurn(waypoint.rotationTarget()));
        }
        if (element instanceof Path.TranslationTarget target) {
            Translation2d point = target.translation();
            return new Path.TranslationTarget(new Translation2d(
                FieldConstants.FIELD_LENGTH - point.getX(),
                FieldConstants.FIELD_WIDTH - point.getY()), target.intermediateHandoffRadiusMeters());
        }
        if (element instanceof Path.RotationTarget target) {
            return new Path.RotationTarget(target.rotation().plus(Rotation2d.fromDegrees(180)),
                target.t_ratio(), target.profiledRotation());
        }
        return element.copy();
    }

    /**
     * Retains BLine scalar diagnostics at 2 Hz. Bulk pose/path graphics are discarded to avoid
     * repeatedly serializing trajectory arrays; these consumers do not control path following.
     */
    private static void configureBLineLogging() {
        FollowPath.setDoubleLoggingConsumer(value -> {
            String key = value.getFirst();
            double measurement = value.getSecond();
            RateLimitedTelemetry.recordOutput("BLine/" + key, measurement);

        });
        FollowPath.setBooleanLoggingConsumer(
            value -> RateLimitedTelemetry.recordOutput("BLine/" + value.getFirst(), value.getSecond()));
        FollowPath.setPoseLoggingConsumer(
            value -> {});
        FollowPath.setTranslationListLoggingConsumer(value -> {});
    }

    private FollowPath.Builder createPathBuilder() {
        return new FollowPath.Builder(
            drivetrain,
            drivetrain::getPose,
            drivetrain::getRobotVelocity,
            this::setBLineChassisSpeeds,
            new PIDController(
                BLineConstants.TRANSLATION_KP,
                BLineConstants.TRANSLATION_KI,
                BLineConstants.TRANSLATION_KD),
            new PIDController(
                BLineConstants.ROTATION_KP,
                BLineConstants.ROTATION_KI,
                BLineConstants.ROTATION_KD),
            new PIDController(
                BLineConstants.CROSS_TRACK_KP,
                BLineConstants.CROSS_TRACK_KI,
                BLineConstants.CROSS_TRACK_KD))
            .withDefaultShouldFlip()
            .withTRatioBasedTranslationHandoffs(
                BLineConstants.USE_T_RATIO_BASED_TRANSLATION_HANDOFFS);
    }

    /** Follows a generated path already expressed in the measured field frame, without a pose reset. */
    public FollowPath followFieldPath(Path path) {
        bLinePauseUntilSeconds = 0.0;
        return createPathBuilder().withShouldFlip(() -> false).build(path);
    }

    /**
     * Starts a non-blocking five-second pause when a BLine event with lib_key "wait" is reached.
     * Using a timestamp instead of Thread.sleep() keeps the robot loop, watchdog, logging, and
     * command scheduler running normally during the pause.
     */
    private void startBLineWait() {
        bLinePauseUntilSeconds = Timer.getFPGATimestamp() + BLINE_WAIT_EVENT_SECONDS;
    }

    /**
     * Gates only BLine's drivetrain output. While a wait event is active, the path follower keeps
     * updating internally but the drivetrain receives zero velocity; afterward, following resumes
     * from the robot's measured pose.
     */
    private void setBLineChassisSpeeds(ChassisSpeeds requestedSpeeds) {
        boolean waiting = Timer.getFPGATimestamp() < bLinePauseUntilSeconds;
        ChassisSpeeds appliedSpeeds = waiting ? new ChassisSpeeds() : requestedSpeeds;
        ChassisSpeeds measuredSpeeds = drivetrain.getRobotVelocity();
        RateLimitedTelemetry.recordOutput("BLine/Wait/Active", waiting);
        RateLimitedTelemetry.recordOutput(
            "BLine/Wait/RemainingSeconds",
            waiting ? bLinePauseUntilSeconds - Timer.getFPGATimestamp() : 0.0);
        RateLimitedTelemetry.recordOutput("BLine/Drive/RequestedRobotVxMetersPerSec", requestedSpeeds.vxMetersPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/RequestedRobotVyMetersPerSec", requestedSpeeds.vyMetersPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/RequestedOmegaRadPerSec", requestedSpeeds.omegaRadiansPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/AppliedRobotVxMetersPerSec", appliedSpeeds.vxMetersPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/AppliedRobotVyMetersPerSec", appliedSpeeds.vyMetersPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/AppliedOmegaRadPerSec", appliedSpeeds.omegaRadiansPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/MeasuredRobotVxMetersPerSec", measuredSpeeds.vxMetersPerSecond);
        RateLimitedTelemetry.recordOutput("BLine/Drive/MeasuredRobotVyMetersPerSec", measuredSpeeds.vyMetersPerSecond);
        drivetrain.setChassisSpeeds(appliedSpeeds);
    }

    public Command getNoAuto() {
        return Commands.none();
    }

    /** Shooter runs from 0–10 seconds; tunnel runs from 4–10 seconds. */
    public Command getCenterAuto(Indexer indexer) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                drivetrain.setChassisSpeeds(new ChassisSpeeds());
                indexer.setTunnelRPM(0.0);
                shooter.setGoal(4000.0);
            }, drivetrain, shooter, indexer),
            Commands.waitSeconds(4.0),
            Commands.runOnce(() -> indexer.setTunnelRPM(2000.0), indexer),
            Commands.waitSeconds(6.0),
            Commands.runOnce(() -> {
                indexer.setTunnelRPM(0.0);
                shooter.stop();
            }, shooter, indexer),
            Commands.idle(shooter, indexer)
        ).finallyDo(interrupted -> {
            indexer.setTunnelRPM(0.0);
            shooter.stop();
            drivetrain.setChassisSpeeds(new ChassisSpeeds());
        });
    }

    /** Runs the path exported from the BLine editor and resets odometry to its first waypoint. */
    public Command getBLineEditorPath() {
        return firstPathBuilder.build(new Path("phase-1-canvas-draft"));
    }

    /** Runs the zigzag path exported from the BLine editor and resets odometry at its start. */
    public Command getZigzagPath() {
        Path zigzagPath = new Path("zigzag");
        List<Translation2d> translations = zigzagPath.getTranslations();
        if (translations.size() < 2) {
            throw new IllegalArgumentException("Zigzag path must contain at least two translation targets");
        }

        Translation2d finalSegmentStart = translations.get(translations.size() - 2);
        Translation2d finalWaypoint = translations.get(translations.size() - 1);
        return firstPathBuilder.build(zigzagPath)
            // If momentum carries the robot through the final waypoint, finish immediately
            // instead of allowing the endpoint PID to reverse back toward it.
            .until(() -> hasPassedFinishPlane(finalSegmentStart, finalWaypoint))
            .finallyDo(interrupted -> drivetrain.setChassisSpeeds(new ChassisSpeeds()));
    }

    private boolean hasPassedFinishPlane(
            Translation2d blueSegmentStart, Translation2d blueSegmentEnd) {
        Translation2d segmentStart = blueSegmentStart;
        Translation2d segmentEnd = blueSegmentEnd;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red) {
            segmentStart = FlippingUtil.flipFieldPosition(segmentStart);
            segmentEnd = FlippingUtil.flipFieldPosition(segmentEnd);
        }

        Translation2d finalSegment = segmentEnd.minus(segmentStart);
        Translation2d startToRobot = drivetrain.getPose().getTranslation().minus(segmentStart);
        double segmentLengthSquared =
            finalSegment.getX() * finalSegment.getX() + finalSegment.getY() * finalSegment.getY();
        double progress =
            (startToRobot.getX() * finalSegment.getX()
                + startToRobot.getY() * finalSegment.getY()) / segmentLengthSquared;
        boolean passedFinishPlane = progress >= 1.0;
        RateLimitedTelemetry.recordOutput("BLine/Zigzag/FinalSegmentProgress", progress);
        RateLimitedTelemetry.recordOutput("BLine/Zigzag/PassedFinishPlane", passedFinishPlane);
        return passedFinishPlane;
    }

    public Command intake() {
        return Commands.runOnce(() -> {
            intake.requestRackExtended();
            intake.setRollersRequested(true);
        }, intake);
    }

    public Command retract() {
        return Commands.runOnce(() -> {
            intake.setRollersRequested(false);
            intake.requestRackRetracted();
        }, intake);
    }
}
