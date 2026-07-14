package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.LimelightHelpers;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Drives to a field-coordinate target pose using a private copy of the drivetrain's pose
 * estimation pipeline whose vision input is a single anchor tag only.
 */
public class Autoalign extends SequentialCommandGroup {

    private static final APConstraints kConstraints = new APConstraints()
        .withVelocity(0.3)
        .withAcceleration(0.3)
        .withJerk(1.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    private static final String kCamera = "limelight-back";
    private static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double kMaxSpinRadPerSec = Math.toRadians(360);

    private final int tagId;
    private final Pose2d tagFieldPose;
    private final Pose2d targetPose;

    private final PIDController rotationController;
    private final Field2d m_field = new Field2d();
    private final SwerveSubsystem drivetrain;

    private SwerveDrivePoseEstimator estimator;
    private boolean seeded;

    /** @param tagId the single tag the pose estimate is anchored to
     *  @param targetPose field-coordinate pose to drive to */
    public Autoalign(SwerveSubsystem drivetrain, int tagId, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.tagId = tagId;
        this.tagFieldPose = kFieldLayout.getTagPose(tagId)
            .orElseThrow(() -> new IllegalArgumentException(
                "Autoalign: tag " + tagId + " not in field layout"))
            .toPose2d();
        this.targetPose = targetPose;

        this.rotationController = new PIDController(5.5, 0.15, 0.05);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("Autoalign/TagId", tagId);
        SmartDashboard.putData("Autoalign/Field", m_field);
        m_field.getObject("target").setPose(targetPose);
        m_field.getObject("tag").setPose(tagFieldPose);

        super.addCommands(
            Commands.runOnce(this::init),
            getDriveToTarget()
        );
        super.addRequirements(drivetrain);
    }

    private void init() {
        SwerveDrive sd = drivetrain.getSwerveDrive();
        estimator = new SwerveDrivePoseEstimator(
            sd.kinematics, sd.getYaw(), sd.getModulePositions(), drivetrain.getPose());
        seeded = false;
        rotationController.reset();
        LimelightHelpers.setPriorityTagID(kCamera, tagId);
    }

    /**
     * Copy of LimelightVision.updateLimelightEstimate() with the MegaTag all-tags input replaced
     * by a solve from the single anchor tag.
     */
    private void updateEstimator() {
        SwerveDrive sd = drivetrain.getSwerveDrive();
        estimator.update(sd.getYaw(), sd.getModulePositions());

        Pose3d raw = LimelightHelpers.getBotPose3d_TargetSpace(kCamera);
        double tagDist = raw.getTranslation().getNorm();
        boolean valid = LimelightHelpers.getTV(kCamera)
            && (int) LimelightHelpers.getFiducialID(kCamera) == tagId
            && tagDist > 1e-3;
        SmartDashboard.putBoolean("Autoalign/TagVisible", valid);
        if (!valid) {
            return;
        }

        Pose2d robotInTag = toPlanarTagFrame(raw);
        Pose2d visionPose = tagFieldPose.transformBy(
            new Transform2d(robotInTag.getTranslation(), robotInTag.getRotation()));
        double latency = (LimelightHelpers.getLatency_Capture(kCamera)
            + LimelightHelpers.getLatency_Pipeline(kCamera)) / 1000.0;
        m_field.getObject("visionPose").setPose(visionPose);

        // First solve seeds position and heading; after that the gyro owns heading (theta stddev).
        if (!seeded) {
            estimator.resetPosition(sd.getYaw(), sd.getModulePositions(), visionPose);
            seeded = true;
            return;
        }

        boolean accepted = true;
        if (Math.abs(drivetrain.getRobotVelocity().omegaRadiansPerSecond) > kMaxSpinRadPerSec) {
            accepted = false;
        }
        if (visionPose.getTranslation().getDistance(
                estimator.getEstimatedPosition().getTranslation()) > LimelightConstants.JUMP_TOLERANCE) {
            accepted = false;
        }
        SmartDashboard.putBoolean("Autoalign/MeasurementAccepted", accepted);
        if (!accepted) {
            return;
        }

        double kStdvXY = Math.max(LimelightConstants.kStdvXYFloor, tagDist * tagDist);
        estimator.addVisionMeasurement(
            visionPose,
            Timer.getFPGATimestamp() - latency,
            VecBuilder.fill(
                kStdvXY * LimelightConstants.kStdvXYBase,
                kStdvXY * LimelightConstants.kStdvXYBase,
                LimelightConstants.kStdvThetaBase));
    }

    /** Axis remap measured on robot 2026-07-06. */
    private static Pose2d toPlanarTagFrame(Pose3d r) {
        Translation3d fwd = new Translation3d(1, 0, 0).rotateBy(r.getRotation());
        return new Pose2d(-r.getZ(), r.getX(), new Rotation2d(Math.atan2(fwd.getX(), -fwd.getZ())));
    }

    /** Back-in arrival: approach direction is opposite the robot's facing (intake away from target). */
    private APTarget getTarget() {
        return new APTarget(targetPose)
            .withEntryAngle(targetPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public Command getDriveToTarget() {
        return Commands.run(() -> {
            updateEstimator();

            if (!seeded) {
                drivetrain.drive(new ChassisSpeeds());
                SmartDashboard.putString("Autoalign/Status", "Waiting for tag " + tagId);
                SmartDashboard.putBoolean("Autoalign/At_Target", false);
                return;
            }
            SmartDashboard.putString("Autoalign/Status", "Tracking tag " + tagId);

            Pose2d currentPose = estimator.getEstimatedPosition();
            APTarget target = getTarget();
            Autopilot.APResult output =
                kAutopilot.calculate(currentPose, drivetrain.getRobotVelocity(), target);

            double xVel = output.vx().in(MetersPerSecond);
            double yVel = output.vy().in(MetersPerSecond);
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                output.targetAngle().getRadians());

            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            m_field.setRobotPose(currentPose);
            m_field.getObject("fusedPose").setPose(drivetrain.getPose());

            SmartDashboard.putNumber("Autoalign/Current_X", currentPose.getX());
            SmartDashboard.putNumber("Autoalign/Current_Y", currentPose.getY());
            SmartDashboard.putNumber("Autoalign/Current_HeadingDeg", currentPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Autoalign/Distance",
                currentPose.getTranslation().getDistance(targetPose.getTranslation()));
            SmartDashboard.putNumber("Autoalign/Output_X_Vel", xVel);
            SmartDashboard.putNumber("Autoalign/Output_Y_Vel", yVel);
            SmartDashboard.putNumber("Autoalign/Output_Rot_Vel", rotVel);
            SmartDashboard.putBoolean("Autoalign/At_Target", kAutopilot.atTarget(currentPose, target));
        }, drivetrain).until(() ->
            seeded && kAutopilot.atTarget(estimator.getEstimatedPosition(), getTarget())
        ).finallyDo(() -> drivetrain.drive(new ChassisSpeeds()));
    }
}
