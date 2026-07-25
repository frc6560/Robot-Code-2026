package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
 * estimation pipeline whose vision input is MegaTag2 restricted to a single anchor tag.
 */
public class Autoalign extends SequentialCommandGroup {

    // Jerk sets the landing brake: peak decel = (2/3)*sqrt(4.5*jerk*velocity). 4.0 here keeps that
    // at ~5.7 m/s^2, the same braking already proven at velocity 2.0 / jerk 8.0. Braking starts
    // ~1.9m out.
    private static final APConstraints kConstraints = new APConstraints()
        .withVelocity(4.0)
        .withAcceleration(3.0)
        .withJerk(4.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(1.0))
        .withBeelineRadius(Centimeters.of(30));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    private static final String[] kCameras = LimelightConstants.LIMELIGHT_NAMES;
    private static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final int[] kAllTagIds =
        kFieldLayout.getTags().stream().mapToInt(t -> t.ID).toArray();
    private static final double kMaxSpinRadPerSec = Math.toRadians(360);
    // Deliberately looser than LimelightConstants.kStdvXYFloor: close to the tag that floor gives a
    // ~1.5cm stddev, so vision overrides odometry every frame and the pose follows camera noise.
    private static final double kVisionStdvFloor = 1.0;
    // Below these the swerve modules hunt instead of holding still. Autopilot still commands
    // ~0.24 m/s at 2cm out, so this only engages once the robot is effectively parked.
    private static final double kMinTranslationMps = 0.05;
    private static final double kMinRotationRadPerSec = 0.03;

    private final int tagId;
    private final Pose2d tagFieldPose;
    private final Pose2d targetPose;

    private final ProfiledPIDController rotationController;
    private final Field2d m_field = new Field2d();
    private final SwerveSubsystem drivetrain;

    // Struct topic so the target shows up as a draggable 2D pose in AdvantageScope
    private final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Autoalign/TargetPose", Pose2d.struct).publish();

    private SwerveDrivePoseEstimator estimator;
    private boolean seeded;
    // Last consumed vision timestamp per camera, so a frame is never ingested twice.
    private final double[] lastVisionTimestamps = new double[kCameras.length];

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

        // No kD: it differentiates pose noise straight into the rotation command. 270 deg/s stays
        // under the 360 deg/s vision reject so tracking survives rotation.
        this.rotationController = new ProfiledPIDController(4.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(Math.toRadians(270), Math.toRadians(360)));
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("Autoalign/TagId", tagId);
        SmartDashboard.putNumber("Autoalign/Target_X", targetPose.getX());
        SmartDashboard.putNumber("Autoalign/Target_Y", targetPose.getY());
        SmartDashboard.putNumber("Autoalign/Target_Rot_Deg", targetPose.getRotation().getDegrees());
        SmartDashboard.putData("Autoalign/Field", m_field);
        m_field.getObject("target").setPose(targetPose);
        m_field.getObject("tag").setPose(tagFieldPose);
        targetPosePublisher.set(targetPose);

        // finallyDo wraps BOTH steps: an interrupt landing while init() is the active step must
        // still restore the tag filters, which a finallyDo on the drive step alone would miss.
        super.addCommands(
            Commands.runOnce(this::init)
                .andThen(getDriveToTarget())
                .finallyDo(this::cleanup)
        );
        super.addRequirements(drivetrain);
    }

    /** Stop the drivetrain and hand every camera back to normal multi-tag vision. Idempotent. */
    private void cleanup() {
        drivetrain.drive(new ChassisSpeeds());
        for (String camera : kCameras) {
            LimelightHelpers.SetFiducialIDFiltersOverride(camera, kAllTagIds);
        }
    }

    private void init() {
        SwerveDrive sd = drivetrain.getSwerveDrive();
        estimator = new SwerveDrivePoseEstimator(
            sd.kinematics, sd.getYaw(), sd.getModulePositions(), drivetrain.getPose());
        seeded = false;
        java.util.Arrays.fill(lastVisionTimestamps, 0.0);
        rotationController.reset(drivetrain.getPose().getRotation().getRadians());
        for (String camera : kCameras) {
            // Restrict MegaTag2 to the anchor tag; restored in finallyDo.
            LimelightHelpers.SetFiducialIDFiltersOverride(camera, new int[] {tagId});
            // Re-push the mount pose in case the boot-time config never reached the camera.
            Pose3d camPose = LimelightConstants.getLimelightPose(camera);
            LimelightHelpers.setCameraPose_RobotSpace(camera,
                camPose.getX(), camPose.getY(), camPose.getZ(),
                Math.toDegrees(camPose.getRotation().getX()),
                Math.toDegrees(camPose.getRotation().getY()),
                Math.toDegrees(camPose.getRotation().getZ()));
        }
    }

    /**
     * Copy of LimelightVision.updateLimelightEstimate() run for every camera, but a MegaTag2
     * estimate is only accepted when it was solved from the single anchor tag. MT2 pins
     * orientation to the gyro (via the existing SetRobotOrientation feed) and solves translation
     * from the tag, so there is no single-tag pose ambiguity.
     */
    private void updateEstimator() {
        SwerveDrive sd = drivetrain.getSwerveDrive();
        estimator.update(sd.getYaw(), sd.getModulePositions());

        boolean anyValid = false;
        for (int i = 0; i < kCameras.length; i++) {
            String camera = kCameras[i];
            PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
            // rawFiducials is sized to tagCount but left FULL OF NULLS when the botpose array
            // length doesn't match what LimelightHelpers expects, so the element needs its own
            // null check before .id.
            boolean valid = est != null
                && est.pose != null
                && !est.pose.equals(Pose2d.kZero)
                && est.tagCount == 1
                && est.rawFiducials != null
                && est.rawFiducials.length == 1
                && est.rawFiducials[0] != null
                && est.rawFiducials[0].id == tagId;
            if (!valid) {
                continue;
            }
            anyValid = true;

            // The cameras publish slower than the 20ms loop, so without this the same frame is fed
            // in on consecutive ticks and the estimator over-trusts it into a twitch.
            if (est.timestampSeconds <= lastVisionTimestamps[i]) {
                continue;
            }
            lastVisionTimestamps[i] = est.timestampSeconds;

            Pose2d visionPose = est.pose;
            m_field.getObject("visionPose").setPose(visionPose);
            SmartDashboard.putNumber("Autoalign/AvgTagDist", est.avgTagDist);
            SmartDashboard.putNumber("Autoalign/Vision_X", visionPose.getX());
            SmartDashboard.putNumber("Autoalign/Vision_Y", visionPose.getY());
            SmartDashboard.putNumber("Autoalign/Vision_HeadingDeg", visionPose.getRotation().getDegrees());
            SmartDashboard.putString("Autoalign/VisionCamera", camera);

            if (!seeded) {
                estimator.resetPosition(sd.getYaw(), sd.getModulePositions(), visionPose);
                seeded = true;
                rotationController.reset(visionPose.getRotation().getRadians());
                SmartDashboard.putNumber("Autoalign/SeedVsFusedHeadingDeg",
                    visionPose.getRotation().minus(drivetrain.getPose().getRotation()).getDegrees());
                continue;
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
                continue;
            }

            double kStdvXY = Math.max(kVisionStdvFloor,
                est.avgTagDist * est.avgTagDist / est.tagCount);
            // timestampSeconds is the NT receive time minus camera latency -- same clock as the
            // estimator, and steadier than sampling Timer at an arbitrary point in the loop.
            estimator.addVisionMeasurement(
                visionPose,
                est.timestampSeconds,
                VecBuilder.fill(
                    kStdvXY * LimelightConstants.kStdvXYBase,
                    kStdvXY * LimelightConstants.kStdvXYBase,
                    LimelightConstants.kStdvThetaBase));
        }
        SmartDashboard.putBoolean("Autoalign/TagVisible", anyValid);
    }

    /** Intake-first arrival: approach direction is the robot's facing. */
    private APTarget getTarget() {
        return new APTarget(targetPose).withEntryAngle(targetPose.getRotation());
    }

    public Command getDriveToTarget() {
        return Commands.run(() -> {
            updateEstimator();

            Pose2d currentPose = estimator.getEstimatedPosition();
            Pose2d fusedPose = drivetrain.getPose();
            APTarget target = getTarget();

            double xVel = 0.0;
            double yVel = 0.0;
            double rotVel = 0.0;
            double targetAngleDeg = targetPose.getRotation().getDegrees();

            if (seeded) {
                Autopilot.APResult output =
                    kAutopilot.calculate(currentPose, drivetrain.getRobotVelocity(), target);
                xVel = output.vx().in(MetersPerSecond);
                yVel = output.vy().in(MetersPerSecond);
                rotVel = rotationController.calculate(
                    currentPose.getRotation().getRadians(),
                    output.targetAngle().getRadians());
                targetAngleDeg = output.targetAngle().getDegrees();

                // Deadband so the modules stop hunting once the robot is essentially parked.
                if (Math.hypot(xVel, yVel) < kMinTranslationMps) {
                    xVel = 0.0;
                    yVel = 0.0;
                }
                if (Math.abs(rotVel) < kMinRotationRadPerSec) {
                    rotVel = 0.0;
                }

                drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));
                SmartDashboard.putString("Autoalign/Status", "Tracking tag " + tagId);
            } else {
                drivetrain.drive(new ChassisSpeeds());
                SmartDashboard.putString("Autoalign/Status", "Waiting for tag " + tagId);
            }

            m_field.setRobotPose(currentPose);
            m_field.getObject("fusedPose").setPose(fusedPose);

            // Target (constant lines for reference on plots)
            SmartDashboard.putNumber("Autoalign/Target_X", targetPose.getX());
            SmartDashboard.putNumber("Autoalign/Target_Y", targetPose.getY());
            SmartDashboard.putNumber("Autoalign/Target_Rot_Deg", targetPose.getRotation().getDegrees());

            // Single-tag pose estimate
            SmartDashboard.putNumber("Autoalign/Current_X", currentPose.getX());
            SmartDashboard.putNumber("Autoalign/Current_Y", currentPose.getY());
            SmartDashboard.putNumber("Autoalign/Current_HeadingDeg", currentPose.getRotation().getDegrees());

            // Fused (all-tags) pose and disagreement with the single-tag estimate
            SmartDashboard.putNumber("Autoalign/Fused_X", fusedPose.getX());
            SmartDashboard.putNumber("Autoalign/Fused_Y", fusedPose.getY());
            SmartDashboard.putNumber("Autoalign/Fused_HeadingDeg", fusedPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Autoalign/FusedDelta_X", currentPose.getX() - fusedPose.getX());
            SmartDashboard.putNumber("Autoalign/FusedDelta_Y", currentPose.getY() - fusedPose.getY());
            SmartDashboard.putNumber("Autoalign/FusedDelta_HeadingDeg",
                currentPose.getRotation().minus(fusedPose.getRotation()).getDegrees());

            // Errors to target
            SmartDashboard.putNumber("Autoalign/Error_X", targetPose.getX() - currentPose.getX());
            SmartDashboard.putNumber("Autoalign/Error_Y", targetPose.getY() - currentPose.getY());
            SmartDashboard.putNumber("Autoalign/Error_Rot_Deg",
                targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
            SmartDashboard.putNumber("Autoalign/Distance",
                currentPose.getTranslation().getDistance(targetPose.getTranslation()));

            // Commanded outputs and heading references
            SmartDashboard.putNumber("Autoalign/Output_X_Vel", xVel);
            SmartDashboard.putNumber("Autoalign/Output_Y_Vel", yVel);
            SmartDashboard.putNumber("Autoalign/Output_Rot_Vel", rotVel);
            SmartDashboard.putNumber("Autoalign/TargetAngleDeg", targetAngleDeg);
            SmartDashboard.putNumber("Autoalign/RotSetpointDeg",
                Math.toDegrees(rotationController.getSetpoint().position));

            // Actual robot velocity
            ChassisSpeeds actualVel = drivetrain.getFieldVelocity();
            SmartDashboard.putNumber("Autoalign/Actual_X_Vel", actualVel.vxMetersPerSecond);
            SmartDashboard.putNumber("Autoalign/Actual_Y_Vel", actualVel.vyMetersPerSecond);
            SmartDashboard.putNumber("Autoalign/Actual_Rot_Vel", actualVel.omegaRadiansPerSecond);

            SmartDashboard.putBoolean("Autoalign/Seeded", seeded);
            SmartDashboard.putBoolean("Autoalign/At_Target",
                seeded && kAutopilot.atTarget(currentPose, target));
        }, drivetrain).until(() ->
            seeded && kAutopilot.atTarget(estimator.getEstimatedPosition(), getTarget())
        ).finallyDo(() -> drivetrain.drive(new ChassisSpeeds()));
    }
}
