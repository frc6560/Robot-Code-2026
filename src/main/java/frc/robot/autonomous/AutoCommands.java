package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.BLineCommands;
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
    private final SwerveSubsystem drivetrain;
    private final Feeder feeder;
    private final Intake intake;
    private final Shooter shooter;
    private final ShotCalculator calculator = new ShotCalculator();
    private final FollowPath.Builder pathBuilder;
    private final FollowPath.Builder firstPathBuilder;

    public AutoCommands(SwerveSubsystem drivetrain, Feeder feeder, Intake intake, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        this.shooter = shooter;

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
    }

    private FollowPath.Builder createPathBuilder() {
        return new FollowPath.Builder(
            drivetrain,
            drivetrain::getPose,
            drivetrain::getRobotVelocity,
            drivetrain::setChassisSpeeds,
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

    public Command getNoAuto() {
        return Commands.none();
    }

    /** Runs the path exported from the BLine editor and resets odometry to its first waypoint. */
    public Command getBLineEditorPath() {
        return firstPathBuilder.build(new Path("phase-1-canvas-draft"));
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
