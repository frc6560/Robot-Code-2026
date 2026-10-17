package frc.robot.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/** Red-side, odometry-only autonomous. No path planner or vision estimate is required. */
public final class RedHubSimpleAuto {
    private static final Pose2d START_POSE = new Pose2d(12.8781423569, 7.3673672676, new Rotation2d());
    private static final double REVERSE_DISTANCE_METERS = Units.inchesToMeters(25.0);
    private static final double SHOOTER_RPM = 5000.0;
    private static final double TUNNEL_RPM = 2000.0;
    private static final double SPIN_UP_SECONDS = 4.0;

    private RedHubSimpleAuto() {}

    public static Command create(SwerveSubsystem drivebase, Shooter shooter, Indexer indexer) {
        Pose2d[] reverseStart = new Pose2d[1];
        Rotation2d[] heldHeading = new Rotation2d[1];
        ProfiledPIDController aimController = new ProfiledPIDController(
            DrivebaseConstants.HUB_AIM_KP,
            DrivebaseConstants.HUB_AIM_KI,
            DrivebaseConstants.HUB_AIM_KD,
            new TrapezoidProfile.Constraints(
                DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
                DrivebaseConstants.HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));
        aimController.enableContinuousInput(-Math.PI, Math.PI);
        aimController.setTolerance(
            DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS,
            DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC);

        PIDController holdController = new PIDController(
            DrivebaseConstants.HUB_AIM_KP,
            DrivebaseConstants.HUB_AIM_KI,
            DrivebaseConstants.HUB_AIM_KD);
        holdController.enableContinuousInput(-Math.PI, Math.PI);

        Command reverse = Commands.runOnce(() -> reverseStart[0] = drivebase.getPose(), drivebase)
            .andThen(Commands.run(() -> {
                Pose2d current = drivebase.getPose();
                Translation2d displacement = current.getTranslation().minus(reverseStart[0].getTranslation());
                double backwardMeters = -displacement.getX() * reverseStart[0].getRotation().getCos()
                    - displacement.getY() * reverseStart[0].getRotation().getSin();
                double remaining = REVERSE_DISTANCE_METERS - backwardMeters;
                double speed = MathUtil.clamp(1.5 * remaining, 0.12, 0.45);
                drivebase.setChassisSpeeds(new ChassisSpeeds(-speed, 0.0, 0.0));
                Logger.recordOutput("RedHubAuto/ReverseMeters", backwardMeters);
            }, drivebase)
                .until(() -> reverseDistance(drivebase.getPose(), reverseStart[0])
                    >= REVERSE_DISTANCE_METERS - 0.02)
                .withTimeout(4.0))
            .finallyDo(interrupted -> drivebase.setChassisSpeeds(new ChassisSpeeds()));

        Command aim = Commands.runOnce(() -> aimController.reset(
                drivebase.getPose().getRotation().getRadians(),
                drivebase.getRobotVelocity().omegaRadiansPerSecond), drivebase)
            .andThen(Commands.run(() -> {
                double current = drivebase.getPose().getRotation().getRadians();
                double target = headingToRedHub(drivebase.getPose());
                double feedback = aimController.calculate(current, target);
                double omega = aimController.atGoal() ? 0.0 : feedback + aimController.getSetpoint().velocity;
                omega = MathUtil.clamp(omega,
                    -DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
                    DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC);
                drivebase.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, omega));
                Logger.recordOutput("RedHubAuto/AimErrorDegrees",
                    Math.toDegrees(MathUtil.angleModulus(target - current)));
            }, drivebase)
                .until(aimController::atGoal)
                .withTimeout(4.0))
            .finallyDo(interrupted -> drivebase.setChassisSpeeds(new ChassisSpeeds()));

        Command shootAfterAim = Commands.sequence(
            Commands.runOnce(() -> {
                heldHeading[0] = drivebase.getPose().getRotation();
                holdController.reset();
                shooter.setGoal(SHOOTER_RPM);
            }, drivebase, shooter),
            holdAngle(drivebase, holdController, heldHeading).withTimeout(SPIN_UP_SECONDS),
            Commands.runOnce(() -> indexer.setTunnelRPM(TUNNEL_RPM), indexer),
            holdAngle(drivebase, holdController, heldHeading));

        return Commands.sequence(
            Commands.runOnce(() -> {
                drivebase.resetOdometry(START_POSE);
                indexer.setTunnelRPM(0.0);
                shooter.stop();
            }, drivebase, shooter, indexer),
            reverse,
            Commands.either(
                Commands.sequence(
                    aim,
                    Commands.either(shootAfterAim, Commands.none(),
                        () -> isAimedAtHub(drivebase))),
                Commands.none(),
                () -> reverseDistance(drivebase.getPose(), reverseStart[0])
                    >= REVERSE_DISTANCE_METERS - 0.05))
        .finallyDo(interrupted -> {
            drivebase.setChassisSpeeds(new ChassisSpeeds());
            indexer.setTunnelRPM(0.0);
            shooter.stop();
        });
    }

    private static double reverseDistance(Pose2d current, Pose2d start) {
        Translation2d displacement = current.getTranslation().minus(start.getTranslation());
        return -displacement.getX() * start.getRotation().getCos()
            - displacement.getY() * start.getRotation().getSin();
    }

    private static Command holdAngle(
            SwerveSubsystem drivebase, PIDController controller, Rotation2d[] heldHeading) {
        return Commands.run(() -> {
            double omega = controller.calculate(
                drivebase.getPose().getRotation().getRadians(), heldHeading[0].getRadians());
            omega = MathUtil.clamp(omega,
                -DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
                DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC);
            drivebase.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, omega));
        }, drivebase);
    }

    private static double headingToRedHub(Pose2d robotPose) {
        Translation2d toHub = FieldConstants.RED_HUB_CENTER.minus(robotPose.getTranslation());
        return Math.atan2(toHub.getY(), toHub.getX());
    }

    private static boolean isAimedAtHub(SwerveSubsystem drivebase) {
        double error = MathUtil.angleModulus(
            headingToRedHub(drivebase.getPose()) - drivebase.getPose().getRotation().getRadians());
        return Math.abs(error) <= DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS
            && Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond)
                <= DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC;
    }
}
