package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.Constants.BLineConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.Shooter.ShotCalculator;

/** Creates autonomous commands backed entirely by BLine. */
public class AutoCommands {
    private static final double BLINE_WAIT_EVENT_SECONDS = 5.0;
    private static final Translation2d ZIGZAG_FINAL_SEGMENT_START =
        new Translation2d(6.12575, 2.31825);
    private static final Translation2d ZIGZAG_FINAL_WAYPOINT =
        new Translation2d(6.13805, 3.71397);

    private final SwerveSubsystem drivetrain;
    private final Feeder feeder;
    private final Intake intake;
    private final Shooter shooter;
    private final ShotCalculator calculator = new ShotCalculator();
    private final FollowPath.Builder pathBuilder;
    private final FollowPath.Builder firstPathBuilder;
    private double bLinePauseUntilSeconds;

    public AutoCommands(SwerveSubsystem drivetrain, Feeder feeder, Intake intake, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
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

        pathBuilder = createPathBuilder();
        firstPathBuilder = createPathBuilder().withPoseReset(drivetrain::resetOdometry);

        FollowPath.registerEventTrigger("intake", intake());
        FollowPath.registerEventTrigger("retract", retract());
        FollowPath.registerEventTrigger("wait", this::startBLineWait);
        FollowPath.registerEventTrigger("Wait", this::startBLineWait);
    }

    /**
     * Connects BLine's built-in telemetry hooks to AdvantageKit. FollowPath calculates and publishes
     * these values inside its execute() method while a trajectory is running. They appear beneath
     * BLine/FollowPath in AdvantageScope and in the WPILOG written by Robot.
     */
    private static void configureBLineLogging() {
        FollowPath.setDoubleLoggingConsumer(value -> {
            String key = value.getFirst();
            double measurement = value.getSecond();
            Logger.recordOutput("BLine/" + key, measurement);

        });
        FollowPath.setBooleanLoggingConsumer(
            value -> Logger.recordOutput("BLine/" + value.getFirst(), value.getSecond()));
        FollowPath.setPoseLoggingConsumer(
            value -> Logger.recordOutput("BLine/" + value.getFirst(), value.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(
            value -> Logger.recordOutput("BLine/" + value.getFirst(), value.getSecond()));
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
        Logger.recordOutput("BLine/Wait/Active", waiting);
        Logger.recordOutput(
            "BLine/Wait/RemainingSeconds",
            waiting ? bLinePauseUntilSeconds - Timer.getFPGATimestamp() : 0.0);
        Logger.recordOutput("BLine/Drive/RequestedRobotVxMetersPerSec", requestedSpeeds.vxMetersPerSecond);
        Logger.recordOutput("BLine/Drive/RequestedRobotVyMetersPerSec", requestedSpeeds.vyMetersPerSecond);
        Logger.recordOutput("BLine/Drive/RequestedOmegaRadPerSec", requestedSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("BLine/Drive/AppliedRobotVxMetersPerSec", appliedSpeeds.vxMetersPerSecond);
        Logger.recordOutput("BLine/Drive/AppliedRobotVyMetersPerSec", appliedSpeeds.vyMetersPerSecond);
        Logger.recordOutput("BLine/Drive/AppliedOmegaRadPerSec", appliedSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("BLine/Drive/MeasuredRobotVxMetersPerSec", measuredSpeeds.vxMetersPerSecond);
        Logger.recordOutput("BLine/Drive/MeasuredRobotVyMetersPerSec", measuredSpeeds.vyMetersPerSecond);
        drivetrain.setChassisSpeeds(appliedSpeeds);
    }

    public Command getNoAuto() {
        return Commands.none();
    }

    /** Runs the path exported from the BLine editor and resets odometry to its first waypoint. */
    public Command getBLineEditorPath() {
        return firstPathBuilder.build(new Path("phase-1-canvas-draft"));
    }

    /** Runs the zigzag path exported from the BLine editor and resets odometry at its start. */
    public Command getZigzagPath() {
        return firstPathBuilder.build(new Path("zigzag"))
            // If momentum carries the robot through the final waypoint, finish immediately
            // instead of allowing the endpoint PID to reverse back toward it.
            .until(this::hasPassedZigzagFinishPlane)
            .finallyDo(interrupted -> drivetrain.setChassisSpeeds(new ChassisSpeeds()));
    }

    private boolean hasPassedZigzagFinishPlane() {
        Translation2d segmentStart = ZIGZAG_FINAL_SEGMENT_START;
        Translation2d segmentEnd = ZIGZAG_FINAL_WAYPOINT;
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
        Logger.recordOutput("BLine/Zigzag/FinalSegmentProgress", progress);
        Logger.recordOutput("BLine/Zigzag/PassedFinishPlane", passedFinishPlane);
        return passedFinishPlane;
    }

    public Command shoot() {
        return Commands.run(() -> {
            calculator.calculate(drivetrain.getPose(), drivetrain.getFieldVelocity());
            shooter.setGoal(calculator.getFlywheelRPM());
            if (shooter.atTarget()) {
                feeder.setShooting(true);
                feeder.setIntaking(true);
                intake.activate();
            }
        }, shooter, feeder, intake).withTimeout(3.0).finallyDo(interrupted -> {
            feeder.setShooting(false);
            feeder.setIntaking(false);
            intake.deactivate();
            shooter.setGoal(0);
        });
    }

    public Command intake() {
        return Commands.runOnce(() -> {
            intake.activate();
            feeder.setIntaking(true);
        }, intake, feeder);
    }

    public Command retract() {
        return Commands.runOnce(() -> {
            intake.deactivate();
            feeder.setIntaking(false);
        }, intake, feeder);
    }

    public Command getRightAuto() {
        return BLineCommands.sequence(
            firstPathBuilder.build(BLinePaths.hpTrenchToCenter()),
            pathBuilder.build(BLinePaths.hpTrenchToShoot()),
            shoot(),
            pathBuilder.build(BLinePaths.hpTrenchToHp()),
            shoot(),
            pathBuilder.build(BLinePaths.hpHpToCenter()));
    }

    public Command getLeftAuto() {
        return BLineCommands.sequence(
            firstPathBuilder.build(BLinePaths.depotTrenchToCenter()),
            pathBuilder.build(BLinePaths.depotTrenchToShoot()),
            shoot(),
            pathBuilder.build(BLinePaths.depotTrenchToDepot()),
            shoot(),
            pathBuilder.build(BLinePaths.depotPullout()),
            pathBuilder.build(BLinePaths.depotDepotToCenter()));
    }

    public Command getRightTwoSwipe() {
        return BLineCommands.sequence(
            firstPathBuilder.build(BLinePaths.hpTrenchToCenter()),
            pathBuilder.build(BLinePaths.hpFirstSwipeBack()),
            shoot(),
            pathBuilder.build(BLinePaths.hpSecondSwipe()),
            shoot());
    }
}
