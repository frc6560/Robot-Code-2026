package frc.robot.commands.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.HashMap;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.AutoAlignPath;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import frc.robot.utility.LimelightHelpers.RawFiducial;
import frc.robot.utility.Setpoint;

import frc.robot.utility.Enums.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

// Autopilot (team 3414) -- three-phase takeoff/glide/constant-jerk-landing profile.
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Drivetrain-only auto-align command factory for the reef.
 *
 * <p>Design goal: satisfy six requirements -- (1) perpendicular to the target, (2) a smooth stable
 * stop, (3) no stopping mid-path, (4) precise, (5) able to start at any velocity, (6) only stops
 * once, at the end.
 *
 * <p>How it works:
 * <ul>
 *   <li><b>Vision-anchored goal.</b> The scoring goal is measured directly from the AprilTag
 *       ({@link #measureGoalFromVision}): the robot->tag transform (from
 *       {@code getBotPose3d_TargetSpace}) is composed with a constant scoring pose in the tag frame
 *       and applied to the current odometry pose. The tag's absolute field coordinates are never
 *       used, so global field-calibration error does not enter.</li>
 *   <li><b>Continuous re-anchoring.</b> Every tick the goal is re-measured and low-pass blended into
 *       the active goal ({@link #reanchorGoal}), with outlier rejection and hold-last-good on tag
 *       loss. This keeps the alignment closed on the observed tag the whole way (requirement 4),
 *       and it freezes near the end so the final stop is stable (requirement 6).</li>
 *   <li><b>Single coordinated trapezoidal profile.</b> One translation profile (arc length along the
 *       line) plus one rotation profile share a timeline, seeded with the robot's CURRENT velocity
 *       ({@link #seedProfiles}). That gives requirements 2, 3, 5 and 6. Odometry is only the
 *       high-rate execution frame over the short approach.</li>
 * </ul>
 *
 * <p>Multi-camera / 360-degree vision: {@link #bestReefCamera} picks whichever Limelight currently
 * sees a reef tag best (largest area, acceptable ambiguity). Because the robot->tag transform is
 * camera independent, "switching cameras" is just choosing the measurement source -- no controller
 * switch.
 *
 * <p>HARDWARE-VERIFICATION POINTS (the only convention-sensitive spots, all isolated and flagged):
 * <ul>
 *   <li>{@link #robotPoseInTagFrame} -- mapping Limelight target-space axes to a planar tag frame,
 *       and the heading sign.</li>
 *   <li>{@link #kStandoffMeters} / {@link #kBranchLateralMeters} and the LEFT/RIGHT lateral sign in
 *       {@link #measureGoalFromVision} -- the scoring geometry relative to the tag.</li>
 * </ul>
 */
public class AutoAlignCommandFactory {
    // ---- Profile state (instance: one align runs at a time; fully re-seeded each run) ----
    private final TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private final TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    // Translation target position is always 0 (remaining distance driven to zero).
    private final TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0);
    private final TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile translationProfile;
    private TrapezoidProfile rotationProfile;

    // Active straight-line endpoints for the current align. pathStart is fixed at init; activeGoal
    // is continuously re-anchored from vision.
    private Pose2d pathStart = new Pose2d();
    private Pose2d activeGoal = new Pose2d();

    // ---- Path-following constants ----
    private static final double kMaxVelocity = 2.0;              // m/s cruise toward the goal
    private static final double kMaxAcceleration = 1.8;          // m/s^2
    private static final double kMaxOmega = Math.toRadians(270); // rad/s
    private static final double kMaxAlpha = Math.toRadians(360); // rad/s^2
    private static final double kAlignTimeoutSeconds = 3.0;

    // ---- End-condition tolerances (a single, debounced, low-velocity stop) ----
    private static final double kPosToleranceMeters = 0.02;
    private static final double kHeadingToleranceRadians = 0.017; // ~1 deg
    private static final double kTransVelStopThreshold = 0.05;    // m/s, profile velocity
    private static final double kRotVelStopThreshold = 0.10;      // rad/s, profile velocity
    private static final double kStopDebounceSeconds = 0.06;

    // ---- Continuous re-anchoring ----
    private static final double kReanchorAlpha = 0.15;        // low-pass weight toward fresh vision
    private static final double kReanchorMaxJumpMeters = 0.5; // reject frames that move the goal more
    private static final double kReanchorLockMeters = 0.05;   // freeze the goal once this close (stable stop)

    // ---- Vision / scoring geometry ----
    // Add every Limelight that can see the reef here for 360-degree coverage.
    private static final String[] kCameras = { "limelight-left", "limelight-right" };
    // Reef AprilTag IDs (Reefscape field). Edit for the active field/game.
    private static final Set<Integer> kReefTagIds =
        Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    private static final boolean kFilterReefTags = true;
    // Minimum target area to trust a frame. getTA() is in PERCENT of image (0-100), so 0.10 == 0.1%.
    // Raise toward single-digit percent to reject small/far tags once verified on hardware.
    private static final double kMinTagArea = 0.10;
    private static final double kMaxAmbiguity = 0.3; // reject high-ambiguity pose solutions

    // Robot-center standoff out from the tag face at the scoring pose. TUNE to scoring geometry.
    // (package-private so the simulation in this package can reuse the exact geometry.)
    static final double kStandoffMeters = 0.50;
    // Lateral branch offset from the tag center (half the reef branch spacing).
    static final double kBranchLateralMeters = 0.164;
    // Straight perpendicular final-approach distance (the "prescore" offset out from scoring).
    // Used for drawing the planned approach; the live command realizes it via the Autopilot
    // beeline radius. Kept consistent here so the sim and the command agree.
    static final double kPrescoreMeters = 0.70;

    // ---- PID gains for the path-follower's pose correction (feed-forward + correction) ----
    private static final double kP_x = 3.0,   kI_x = 0.03, kD_x = 0.1;
    private static final double kP_y = 2.75,  kI_y = 0.02, kD_y = 0.1;
    private static final double kP_theta = 5.5, kI_theta = 0.15, kD_theta = 0.05;

    private final PIDController xPoseController = new PIDController(kP_x, kI_x, kD_x);
    private final PIDController yPoseController = new PIDController(kP_y, kI_y, kD_y);
    private final PIDController thetaPoseController = new PIDController(kP_theta, kI_theta, kD_theta);

    // ---- Autopilot (tag-based-pose variant) ----
    // Autopilot constraints: max velocity / acceleration / jerk for its takeoff-glide-landing profile.
    private static final double kApMaxVelocity = 2.0;       // m/s
    private static final double kApMaxAcceleration = 1.8;   // m/s^2
    private static final double kApMaxJerk = 8.0;           // m/s^3 (constant-jerk landing)
    private static final double kApErrorThetaDegrees = 1.0; // heading tolerance for atTarget
    private static final double kApBeelineRadiusMeters = 0.30; // within this, drive straight in
    // Heading controller: Autopilot returns a heading SETPOINT, so we close the loop on it ourselves.
    private final PIDController autopilotThetaController = buildAutopilotThetaController();
    private final Autopilot autopilot = buildAutopilot();

    // Field layout (2026 rebuilt-welded) -- used ONLY to look up a tag's facing direction and height
    // for single-tag head-on aligns. The alignment itself stays tag-relative (tx-ty + gyro); the
    // tag's absolute field POSITION is never used to drive.
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Builds the Autopilot with this project's tuned profile. Shared by the command and the sim. */
    static Autopilot buildAutopilot() {
        return new Autopilot(
            new APProfile(new APConstraints(kApMaxVelocity, kApMaxAcceleration, kApMaxJerk))
                .withErrorXY(Meters.of(kPosToleranceMeters))
                .withErrorTheta(Degrees.of(kApErrorThetaDegrees))
                .withBeelineRadius(Meters.of(kApBeelineRadiusMeters)));
    }

    /** Builds the heading controller used to close the loop on Autopilot's heading setpoint. */
    static PIDController buildAutopilotThetaController() {
        PIDController c = new PIDController(kP_theta, kI_theta, kD_theta);
        c.enableContinuousInput(-Math.PI, Math.PI);
        return c;
    }

    /** One Autopilot evaluation: the tag-anchored fake pose, its target, and Autopilot's result. */
    static record ApStep(APResult result, Pose2d current, APTarget target) {}

    /**
     * Pure tag-based-pose Autopilot evaluation, shared by the live command and the simulation.
     * Builds the fake pose in the tag-anchored frame T (tag at the origin -> the anchor cancels),
     * with translation from {@code tagInRobot} (tx-ty in the real robot) and heading from
     * {@code gyro}, then asks Autopilot for the field-relative result toward the perpendicular goal.
     */
    static ApStep computeApStep(Autopilot ap, Translation2d tagInRobot, Rotation2d gyro,
                                ChassisSpeeds robotRelSpeeds, double scoringHeading, double lateralSign) {
        // Reef variant: standoff/lateral come from the shared reef constants.
        return computeApStep(ap, tagInRobot, gyro, robotRelSpeeds, scoringHeading,
            kStandoffMeters, lateralSign * kBranchLateralMeters);
    }

    /**
     * As above, but with an EXPLICIT standoff (meters out from the tag face) and signed lateral
     * (meters along the face). Used for single-tag aligns (e.g. head-on to one tag) that must not
     * disturb the shared reef geometry constants.
     */
    static ApStep computeApStep(Autopilot ap, Translation2d tagInRobot, Rotation2d gyro,
                                ChassisSpeeds robotRelSpeeds, double scoringHeading,
                                double standoffMeters, double lateralMeters) {
        // Frame T: tag pinned at the origin. robotInT = -R(gyro) * tagInRobot.
        Translation2d robotPos = tagInRobot.rotateBy(gyro).unaryMinus();
        Pose2d current = new Pose2d(robotPos, gyro);
        // Goal in T: robot faces the tag at heading psi; goal = origin - standoff*dir(psi) + lateral.
        Rotation2d psi = new Rotation2d(scoringHeading);
        Translation2d standoff = new Translation2d(standoffMeters, psi);
        Translation2d lateral = new Translation2d(lateralMeters, psi.rotateBy(Rotation2d.fromDegrees(90)));
        Pose2d goalPose = new Pose2d(standoff.unaryMinus().plus(lateral), psi);
        APTarget target = new APTarget(goalPose).withEntryAngle(psi).withVelocity(0.0);
        return new ApStep(ap.calculate(current, robotRelSpeeds, target), current, target);
    }
    // Per-tick Autopilot bookkeeping for the (debounced) end condition.
    private boolean autopilotReached = false;
    private Debouncer autopilotStopDebouncer = new Debouncer(kStopDebounceSeconds);
    // Hold-last-good on a brief vision dropout so a momentary occlusion coasts through instead of
    // braking to a stop mid-path (requirement 3). Only fall back to a stop if the dropout persists.
    private static final double kApHoldSeconds = 0.3;
    private static final double kLoopDtSeconds = 0.02;
    private Pose2d apLastCurrent = null;
    private APTarget apLastTarget = null;
    private double apDropoutSeconds = 0.0;

    // ---- Subsystems ----
    private final SwerveSubsystem drivetrain;

    /** Constructor for the auto-align command factory. */
    public AutoAlignCommandFactory(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        thetaPoseController.enableContinuousInput(-Math.PI, Math.PI);
        // autopilotThetaController already has continuous input enabled by its builder.
    }


    // ---- TOP-LEVEL ALIGN COMMANDS ----


    /**
     * Primary auto-align: measures the reef goal from the Limelight and drives onto it with a single
     * velocity-seeded trapezoidal profile, continuously re-anchored to the observed tag. Bails out
     * cleanly (does nothing) if no trustworthy reef tag is seen, so it never drives on garbage.
     *
     * @param side which branch (LEFT/RIGHT) of the reef face to score on
     */
    public Command getAlignTeleop(ReefSide side) {
        return Commands.defer(
            () -> {
                Optional<Pose2d> goal = measureGoalFromVision(side);
                if (goal.isEmpty()) {
                    DriverStation.reportWarning("AutoAlign: no reef tag visible; not aligning.", false);
                    return Commands.none();
                }
                return runProfileToGoal(goal.get(), side); // re-anchors to the tag
            },
            Set.of(drivetrain));
    }

    /**
     * Odometry-only fallback to the surveyed field pose (no vision required, no re-anchoring). For
     * use when vision is unavailable. NOTE: this path carries the global field-calibration error the
     * vision path avoids, and the surveyed poses in {@link #getTarget} are not all verified -- prefer
     * {@link #getAlignTeleop}.
     */
    public Command getDriveToScore(ReefSide side, ReefIndex index) {
        return Commands.defer(
            () -> runProfileToGoal(getTarget(index, side), null), // null side -> no re-anchoring
            Set.of(drivetrain));
    }


    // ---- AUTOPILOT (TAG-BASED POSE) ALIGN ----
    //
    // Feeds Autopilot a "fake pose" built in a tag-anchored frame T that shares the gyro/odometry
    // ORIENTATION. The tag is pinned at the origin of T (an arbitrary anchor that cancels in
    // Autopilot's current->target relative math), so no global field POSITION is ever used:
    //   - translation: tx-ty vision (tag position relative to the robot), rotated into T by gyro yaw.
    //   - heading: gyro (odometry yaw, which is gyro-driven); translation of getPose() is NOT used.
    //   - perpendicular reference: the known reef-face angle (field geometry, not a surveyed position).
    // Because T shares odometry orientation, Autopilot's field-relative output velocity goes straight
    // into driveFieldOriented. Autopilot returns a heading SETPOINT, so a separate controller closes
    // the loop on it.


    /**
     * Autopilot-driven auto-align using a tag-based fake pose (translation from tx-ty, heading from
     * gyro, perpendicular reference from the known reef-face angle). Bails out cleanly if no reef tag
     * is visible. See the section comment above for the frame construction.
     *
     * @param side  which branch (LEFT/RIGHT) to score on
     * @param index which reef face (selects the perpendicular field-angle reference)
     */
    public Command getAlignAutopilot(ReefSide side, ReefIndex index) {
        return Commands.defer(
            () -> {
                if (bestReefCamera().isEmpty()) {
                    DriverStation.reportWarning("AutoAlign(AP): no reef tag visible; not aligning.", false);
                    return Commands.none();
                }
                // Perpendicular scoring heading (faces the tag) from field geometry, in T's frame
                // (T shares field/gyro orientation). Only the rotation of getTarget is used.
                final double scoringHeading = getTarget(index, side).getRotation().getRadians();
                final double lateralSign = (side == ReefSide.LEFT) ? -1.0 : 1.0;

                return new FunctionalCommand(
                    () -> {
                        autopilotThetaController.reset();
                        autopilotReached = false;
                        autopilotStopDebouncer = new Debouncer(kStopDebounceSeconds);
                        apLastCurrent = null;
                        apLastTarget = null;
                        apDropoutSeconds = 0.0;
                    },
                    () -> autopilotStep(scoringHeading, lateralSign),
                    (interrupted) -> drivetrain.drive(new ChassisSpeeds()),
                    () -> autopilotStopDebouncer.calculate(autopilotReached))
                    .withTimeout(kAlignTimeoutSeconds)
                    .finallyDo(() -> drivetrain.drive(new ChassisSpeeds()));
            },
            Set.of(drivetrain));
    }

    /**
     * Autopilot align HEAD-ON (no lateral offset) to a SPECIFIC AprilTag, {@code standoffMeters} out
     * from its face. The tag's facing direction and height are read from the 2026 field layout; the
     * drive itself stays tag-relative (tx-ty + gyro), so no global field POSITION is used to steer.
     * Does NOT touch the shared reef constants. REQUIRES a field-zeroed gyro.
     *
     * <p>Example: {@code getAlignHeadOnToTag(24, Units.feetToMeters(3.0))} -> 3 ft head-on to tag 24.
     *
     * @param tagId          the AprilTag to align to (must exist in the field layout)
     * @param standoffMeters robot-center distance out from the tag face
     */
    public Command getAlignHeadOnToTag(int tagId, double standoffMeters) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            DriverStation.reportWarning("AutoAlign(AP): tag " + tagId + " not in field layout.", false);
            return Commands.none();
        }
        // Robot head-on heading = face INTO the tag = the tag's outward normal + 180 deg (field frame).
        final double scoringHeading = tagPose.get().getRotation().toRotation2d()
            .rotateBy(Rotation2d.fromDegrees(180)).getRadians();
        final double tagHeight = tagPose.get().getZ(); // tag center height, for the tx-ty range math

        return Commands.defer(
            () -> {
                if (cameraSeeingTag(tagId).isEmpty()) {
                    DriverStation.reportWarning("AutoAlign(AP): tag " + tagId + " not visible; not aligning.", false);
                    return Commands.none();
                }
                return new FunctionalCommand(
                    () -> {
                        autopilotThetaController.reset();
                        autopilotReached = false;
                        autopilotStopDebouncer = new Debouncer(kStopDebounceSeconds);
                        apLastCurrent = null;
                        apLastTarget = null;
                        apDropoutSeconds = 0.0;
                    },
                    () -> autopilotStepToTag(tagId, scoringHeading, standoffMeters, 0.0, tagHeight),
                    (interrupted) -> drivetrain.drive(new ChassisSpeeds()),
                    () -> autopilotStopDebouncer.calculate(autopilotReached))
                    .withTimeout(kAlignTimeoutSeconds)
                    .finallyDo(() -> drivetrain.drive(new ChassisSpeeds()));
            },
            Set.of(drivetrain));
    }

    /** One Autopilot control tick for the REEF: best reef camera, shared standoff/lateral. */
    private void autopilotStep(double scoringHeading, double lateralSign) {
        autopilotReached = false;
        Rotation2d gyro = drivetrain.getPose().getRotation();
        ApStep fresh = null;
        Optional<String> camOpt = bestReefCamera();
        if (camOpt.isPresent()) {
            Optional<Translation2d> tagOpt = tagInRobotFromTxTy(camOpt.get());
            if (tagOpt.isPresent()) {
                fresh = computeApStep(autopilot, tagOpt.get(), gyro,
                    drivetrain.getRobotVelocity(), scoringHeading, lateralSign);
            }
        }
        driveAutopilot(gyro, fresh);
    }

    /** One Autopilot control tick for a SPECIFIC tag: explicit heading/standoff/lateral/tag height. */
    private void autopilotStepToTag(int tagId, double scoringHeading, double standoffMeters,
                                    double lateralMeters, double tagHeightMeters) {
        autopilotReached = false;
        Rotation2d gyro = drivetrain.getPose().getRotation();
        ApStep fresh = null;
        Optional<String> camOpt = cameraSeeingTag(tagId);
        if (camOpt.isPresent()) {
            Optional<Translation2d> tagOpt = tagInRobotFromTxTy(camOpt.get(), tagHeightMeters);
            if (tagOpt.isPresent()) {
                fresh = computeApStep(autopilot, tagOpt.get(), gyro,
                    drivetrain.getRobotVelocity(), scoringHeading, standoffMeters, lateralMeters);
            }
        }
        driveAutopilot(gyro, fresh);
    }

    /**
     * Shared Autopilot tail: hold-last-good on a brief vision dropout so a momentary occlusion coasts
     * through instead of braking (requirement 3), drive the field-relative result with a
     * heading-controller omega, and set the (debounced) end flag only on a FRESH fix.
     */
    private void driveAutopilot(Rotation2d gyro, ApStep freshStep) {
        APResult result;
        Pose2d current;
        APTarget target;
        boolean fresh = freshStep != null && apResultFinite(freshStep.result());
        if (fresh) {
            result = freshStep.result();
            current = freshStep.current();
            target = freshStep.target();
            apLastCurrent = current;
            apLastTarget = target;
            apDropoutSeconds = 0.0;
        } else {
            if (apLastCurrent == null || apDropoutSeconds >= kApHoldSeconds) {
                drivetrain.drive(new ChassisSpeeds());
                return;
            }
            apDropoutSeconds += kLoopDtSeconds;
            result = autopilot.calculate(apLastCurrent, drivetrain.getRobotVelocity(), apLastTarget);
            current = apLastCurrent;
            target = apLastTarget;
            if (!apResultFinite(result)) { drivetrain.drive(new ChassisSpeeds()); return; }
        }

        double vx = result.vx().in(MetersPerSecond);
        double vy = result.vy().in(MetersPerSecond);
        double omega = autopilotThetaController.calculate(
            gyro.getRadians(), result.targetAngle().getRadians());
        if (!Double.isFinite(omega)) { drivetrain.drive(new ChassisSpeeds()); return; }
        // T shares odometry orientation, so Autopilot's field-relative output drives directly.
        drivetrain.driveFieldOriented(new ChassisSpeeds(vx, vy, omega));

        if (fresh) {
            ChassisSpeeds measured = drivetrain.getRobotVelocity();
            boolean stopped =
                Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) < kTransVelStopThreshold
                && Math.abs(measured.omegaRadiansPerSecond) < kRotVelStopThreshold;
            autopilotReached = autopilot.atTarget(current, target) && stopped;
        }
    }

    /** True if the Autopilot result exists and its field-relative velocity components are finite. */
    private static boolean apResultFinite(APResult r) {
        return r != null
            && Double.isFinite(r.vx().in(MetersPerSecond))
            && Double.isFinite(r.vy().in(MetersPerSecond));
    }

    /**
     * Tag position relative to the robot, from tx-ty only (no PnP, no global pose). Range comes from
     * the vertical angle ty and the known camera/tag heights; bearing from the horizontal angle tx.
     *
     * <p>HARDWARE-VERIFICATION POINT: the camera extrinsics ({@link #cameraExtrinsics}) and the tx
     * sign must match the physical mount.
     */
    private Optional<Translation2d> tagInRobotFromTxTy(String cam) {
        return tagInRobotFromTxTy(cam, cameraExtrinsics(cam).tagHeightMeters);
    }

    /** As above, but with an explicit target tag-center height (tag height is per-tag, not per-camera). */
    private Optional<Translation2d> tagInRobotFromTxTy(String cam, double tagHeightMeters) {
        if (!LimelightHelpers.getTV(cam)) return Optional.empty();
        double tx = Math.toRadians(LimelightHelpers.getTX(cam));
        double ty = Math.toRadians(LimelightHelpers.getTY(cam));
        CameraExtrinsics ex = cameraExtrinsics(cam);

        double denom = Math.tan(ex.pitchRadians + ty);
        if (Math.abs(denom) < 1e-6) return Optional.empty(); // line of sight near-parallel to floor
        double range = (tagHeightMeters - ex.heightMeters) / denom; // horizontal ground distance
        if (!Double.isFinite(range) || range <= 0) return Optional.empty();

        // Polar in the camera frame: +x forward, +y left, so a right-of-crosshair target (tx>0) is -y.
        Translation2d tagInCam = new Translation2d(range, new Rotation2d(-tx));
        Translation2d tagInRobot = ex.offset.plus(tagInCam.rotateBy(ex.yaw));
        if (!Double.isFinite(tagInRobot.getX()) || !Double.isFinite(tagInRobot.getY())) {
            return Optional.empty();
        }
        return Optional.of(tagInRobot);
    }

    /** Camera mounting parameters used by the tx-ty range/bearing solve. TUNE per camera. */
    private record CameraExtrinsics(
        double heightMeters,     // lens height off the floor
        double pitchRadians,     // upward tilt of the camera
        double tagHeightMeters,  // height of the reef tag center
        Translation2d offset,    // camera position in the robot frame
        Rotation2d yaw) {}       // camera facing in the robot frame

    private CameraExtrinsics cameraExtrinsics(String cam) {
        // Placeholders -- replace with measured values per Limelight. Both cameras default to a
        // forward-facing, robot-centered mount so the code is well-defined before calibration.
        final double tagHeight = 0.305; // reef AprilTag center height (m) -- VERIFY
        switch (cam) {
            case "limelight-left":
                return new CameraExtrinsics(0.20, Math.toRadians(20), tagHeight,
                    new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
            case "limelight-right":
            default:
                return new CameraExtrinsics(0.20, Math.toRadians(20), tagHeight,
                    new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        }
    }


    // ---- ALIGN COMMAND CONSTRUCTION ----


    /**
     * Drives a single trapezoidal profile from the current pose to {@code initialGoal}. If
     * {@code reanchorSide} is non-null, the goal is continuously re-measured from vision and blended
     * in; if null, the goal is held fixed (odometry-only).
     */
    private Command runProfileToGoal(Pose2d initialGoal, ReefSide reanchorSide) {
        String limelightName =
            (reanchorSide != null) ? limelightFor(reanchorSide) : kCameras[0];
        return new FunctionalCommand(
            () -> {
                pathStart = drivetrain.getPose();
                activeGoal = initialGoal;
                seedProfiles(pathStart, activeGoal);
            },
            () -> {
                if (reanchorSide != null) {
                    reanchorGoal(reanchorSide);
                }
                followSetpoint(getNextSetpoint(makePath(pathStart, activeGoal)));
            },
            (interrupted) -> {},
            () -> false)
            .until(atGoal())
            .withTimeout(kAlignTimeoutSeconds)
            .finallyDo(() -> {
                drivetrain.drive(new ChassisSpeeds()); // guarantee a stop
                updateOdometryWithVision(limelightName);
            });
    }

    /**
     * End condition: within position and heading tolerance of the (live) goal AND the profile has
     * wound down to ~0 velocity, debounced so we report "done" exactly once and only at the end.
     */
    private BooleanSupplier atGoal() {
        Debouncer stopDebouncer = new Debouncer(kStopDebounceSeconds);
        return () -> {
            Pose2d cur = drivetrain.getPose();
            double posError = cur.getTranslation().getDistance(activeGoal.getTranslation());
            double headingError = Math.abs(MathUtil.angleModulus(
                cur.getRotation().getRadians() - activeGoal.getRotation().getRadians()));
            boolean profileStopped =
                Math.abs(translationalState.velocity) < kTransVelStopThreshold
                && Math.abs(rotationalState.velocity) < kRotVelStopThreshold;
            // Also require the robot to be physically (not just per the profile) at rest, so a
            // high-speed entry cannot latch "done" while the chassis is still coasting through.
            ChassisSpeeds measured = drivetrain.getRobotVelocity();
            boolean measuredStopped =
                Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) < kTransVelStopThreshold
                && Math.abs(measured.omegaRadiansPerSecond) < kRotVelStopThreshold;
            boolean atTarget =
                posError < kPosToleranceMeters
                && headingError < kHeadingToleranceRadians
                && profileStopped
                && measuredStopped;
            return stopDebouncer.calculate(atTarget);
        };
    }

    /**
     * Re-measures the goal from vision and low-pass blends it into {@link #activeGoal}. Rejects
     * outlier frames, holds the last good goal on tag loss, and freezes the goal once we are close
     * so the final stop does not chase vision noise.
     */
    private void reanchorGoal(ReefSide side) {
        // Freeze near the end for a stable single stop.
        if (drivetrain.getPose().getTranslation().getDistance(activeGoal.getTranslation())
                < kReanchorLockMeters) {
            return;
        }
        Optional<Pose2d> candidate = measureGoalFromVision(side);
        if (candidate.isEmpty()) return; // hold last good on dropout
        Pose2d c = candidate.get();
        // Reject a frame that would jerk the goal (bad solution / wrong tag).
        if (c.getTranslation().getDistance(activeGoal.getTranslation()) > kReanchorMaxJumpMeters) {
            return;
        }
        // Per-axis low-pass toward the fresh measurement. NOTE: use the component interpolators
        // (Translation2d/Rotation2d are true lerps); Pose2d.interpolate is a geodesic twist blend
        // that would cross-couple translation and heading, which we do not want for noise filtering.
        Translation2d blendedTranslation =
            activeGoal.getTranslation().interpolate(c.getTranslation(), kReanchorAlpha);
        Rotation2d blendedRotation =
            activeGoal.getRotation().interpolate(c.getRotation(), kReanchorAlpha);
        activeGoal = new Pose2d(blendedTranslation, blendedRotation);
    }


    // ---- VISION: GOAL MEASUREMENT & CAMERA SELECTION ----


    /**
     * Measures the scoring goal directly from the best Limelight and expresses it in the odometry
     * frame. Returns empty if no trustworthy reef tag is visible.
     *
     * <p>goalOdom = robotPoseOdom (+) transform( robotInTagFrame -> goalInTagFrame ). The tag's
     * absolute field coordinates are never used, so global field-calibration error does not enter.
     */
    public Optional<Pose2d> measureGoalFromVision(ReefSide side) {
        Optional<String> camOpt = bestReefCamera();
        if (camOpt.isEmpty()) return Optional.empty();
        String cam = camOpt.get();

        // Re-confirm the camera's primary tag is a reef tag immediately before reading its
        // target-space pose, so the pose we consume corresponds to a reef tag (botpose_targetspace
        // is relative to the camera's primary in-view tag).
        if (kFilterReefTags
            && !kReefTagIds.contains((int) Math.round(LimelightHelpers.getFiducialID(cam)))) {
            return Optional.empty();
        }

        Pose3d robotInTag3d = LimelightHelpers.getBotPose3d_TargetSpace(cam);
        // Empty/zero pose means no valid target data this frame.
        if (robotInTag3d.getTranslation().getNorm() < 1e-3) return Optional.empty();

        Pose2d robotInTag = robotPoseInTagFrame(robotInTag3d);
        if (!isFinite(robotInTag)) return Optional.empty();

        // Goal expressed in the planar tag frame: stand kStandoffMeters out from the tag face
        // (+x = out of tag), offset to the branch (+y = left), facing the tag (heading = pi).
        // LEFT branch uses -y, RIGHT uses +y (verify against the physical reef on hardware).
        double lateral = (side == ReefSide.LEFT ? -1.0 : 1.0) * kBranchLateralMeters;
        Pose2d goalInTag = new Pose2d(kStandoffMeters, lateral, new Rotation2d(Math.PI));

        // Transform from the robot to the goal, expressed in the robot's local frame...
        Transform2d robotToGoal = new Transform2d(robotInTag, goalInTag);
        // ...applied to the current odometry pose -> goal in the odometry frame.
        Pose2d goalOdom = drivetrain.getPose().transformBy(robotToGoal);

        if (!isFinite(goalOdom)) return Optional.empty();
        return Optional.of(goalOdom);
    }

    /**
     * Converts a Limelight target-space robot {@link Pose3d} into a planar tag frame:
     * x = distance out from the tag face, y = left, heading = robot yaw relative to the tag normal.
     *
     * <p>Limelight target space: +X right, +Y down, +Z out of the tag toward the viewer. Planar
     * frame: x = +Z (out of tag), y = -X (left). Heading is taken from the robot's forward axis
     * projected into that plane, which is robust to the full rotation (no Euler-angle ambiguity).
     */
    private Pose2d robotPoseInTagFrame(Pose3d robotInTag3d) {
        double x = robotInTag3d.getZ();   // distance out from the tag face
        double y = -robotInTag3d.getX();  // left

        // Robot's forward (+X body) axis expressed in the Limelight target frame.
        Translation3d forward = new Translation3d(1, 0, 0).rotateBy(robotInTag3d.getRotation());
        // Project into the planar frame (x = +Z, y = -X) and take its angle.
        double heading = Math.atan2(-forward.getX(), forward.getZ());

        return new Pose2d(x, y, new Rotation2d(heading));
    }

    /**
     * Picks the Limelight that currently sees a reef tag best (largest target area among frames with
     * acceptable ambiguity), giving the 360-degree, camera-independent measurement source. Returns
     * empty if none qualify.
     */
    public Optional<String> bestReefCamera() {
        String best = null;
        double bestTa = kMinTagArea; // must beat the minimum-area threshold to qualify
        for (String cam : kCameras) {
            if (!LimelightHelpers.getTV(cam)) continue;
            int primaryId = (int) Math.round(LimelightHelpers.getFiducialID(cam));
            if (kFilterReefTags && !kReefTagIds.contains(primaryId)) continue;
            if (primaryTagAmbiguity(cam, primaryId) > kMaxAmbiguity) continue;
            double ta = LimelightHelpers.getTA(cam);
            if (ta > bestTa) {
                bestTa = ta;
                best = cam;
            }
        }
        return Optional.ofNullable(best);
    }

    /** The Limelight whose PRIMARY in-view tag is exactly {@code tagId} (acceptable ambiguity), or empty. */
    public Optional<String> cameraSeeingTag(int tagId) {
        String best = null;
        double bestTa = kMinTagArea; // must beat the minimum-area threshold to qualify
        for (String cam : kCameras) {
            if (!LimelightHelpers.getTV(cam)) continue;
            if ((int) Math.round(LimelightHelpers.getFiducialID(cam)) != tagId) continue;
            if (primaryTagAmbiguity(cam, tagId) > kMaxAmbiguity) continue;
            double ta = LimelightHelpers.getTA(cam);
            if (ta > bestTa) {
                bestTa = ta;
                best = cam;
            }
        }
        return Optional.ofNullable(best);
    }

    /** Ambiguity of the camera's primary fiducial, or 1.0 (worst) if it cannot be found. */
    private double primaryTagAmbiguity(String cam, int primaryId) {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(cam);
        if (fiducials == null) return 1.0;
        for (RawFiducial f : fiducials) {
            if (f.id == primaryId) return f.ambiguity;
        }
        return 1.0;
    }


    // ---- HELPER METHODS ----


    private static boolean isFinite(Pose2d pose) {
        return Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians());
    }

    private static String limelightFor(ReefSide side) {
        return (side == ReefSide.LEFT) ? "limelight-right" : "limelight-left";
    }

    private static AutoAlignPath makePath(Pose2d start, Pose2d goal) {
        return new AutoAlignPath(start, goal, kMaxVelocity, kMaxAcceleration, kMaxOmega, kMaxAlpha);
    }

    public boolean isTagVisible(ReefSide side) {
        String limelightName = limelightFor(side);
        return LimelightHelpers.getTV(limelightName) && LimelightHelpers.getTA(limelightName) > 0.0;
    }

    /**
     * Updates odometry from the Limelight MegaTag pose estimate (auto only). Used as a one-shot
     * correction when an align finishes; does not run during teleop to avoid fighting the driver.
     */
    private void updateOdometryWithVision(String limelightName) {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (poseEstimate == null) return;

        Pose2d pose = poseEstimate.pose;
        if (pose == null || pose.equals(new Pose2d())) return;

        if (!DriverStation.isTeleop()) {
            drivetrain.resetOdometry(pose);
        }
    }

    /** Transforms red alliance poses to blue by reflecting around the center point of the field. */
    public Pose2d applyAllianceTransform(Pose2d pose) {
        return new Pose2d(
            pose.getX() - 2 * (pose.getX() - 8.75),
            pose.getY() - 2 * (pose.getY() - 4.0),
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    /**
     * Surveyed field-pose fallback for a given reef index/side (used only by {@link #getDriveToScore}
     * when no tag is visible). Carries the global field-calibration error the vision path avoids.
     */
    public Pose2d getTarget(ReefIndex index, ReefSide side) {
        DriverStation.Alliance alliance =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        int multiplier = (side == ReefSide.LEFT) ? -1 : 1;
        final double DISTANCE_FROM_TAG = kBranchLateralMeters;
        HashMap<ReefIndex, Pose2d> targetPoses = new HashMap<>();

        // All target poses are in meters.
        targetPoses.put(ReefIndex.BOTTOM_RIGHT, new Pose2d(13.477, 2.691, Rotation2d.fromDegrees(300))); // TODO verify
        targetPoses.put(ReefIndex.FAR_RIGHT, new Pose2d(14.423, 3.721, Rotation2d.fromDegrees(0)));
        targetPoses.put(ReefIndex.TOP_RIGHT, new Pose2d(14.006, 5.056, Rotation2d.fromDegrees(60)));
        targetPoses.put(ReefIndex.TOP_LEFT, new Pose2d(12.640, 5.361, Rotation2d.fromDegrees(120)));
        targetPoses.put(ReefIndex.FAR_LEFT, new Pose2d(11.693, 4.331, Rotation2d.fromDegrees(180))); // TODO verify
        targetPoses.put(ReefIndex.BOTTOM_LEFT, new Pose2d(12.111, 2.996, Rotation2d.fromDegrees(240)));

        Pose2d aprilTagPose = targetPoses.get(index);

        Pose2d targetPose = new Pose2d(
            aprilTagPose.getX() + (DISTANCE_FROM_TAG * Math.cos(aprilTagPose.getRotation().getRadians() + Math.PI / 2) * multiplier),
            aprilTagPose.getY() + (DISTANCE_FROM_TAG * Math.sin(aprilTagPose.getRotation().getRadians() + Math.PI / 2) * multiplier),
            aprilTagPose.getRotation());

        if (alliance == DriverStation.Alliance.Blue) {
            targetPose = applyAllianceTransform(targetPose);
        }
        return targetPose;
    }

    public Pose2d getPrescoreTarget(Pose2d targetPose) {
        return new Pose2d(
            targetPose.getX() + 0.8 * Math.cos(targetPose.getRotation().getRadians()),
            targetPose.getY() + 0.8 * Math.sin(targetPose.getRotation().getRadians()),
            targetPose.getRotation());
    }


    // ---- TRAJECTORY PROFILE (straight line, trapezoidal, velocity-seeded) ----


    /**
     * Seeds the translation and rotation profiles from the robot's CURRENT velocity so the align can
     * begin at any speed without first braking (requirement: start at any velocity).
     */
    private void seedProfiles(Pose2d start, Pose2d goal) {
        translationProfile = new TrapezoidProfile(new Constraints(kMaxVelocity, kMaxAcceleration));
        rotationProfile = new TrapezoidProfile(new Constraints(kMaxOmega, kMaxAlpha));

        // Clear integral windup / stale derivative so a subsequent align does not start with a
        // spurious correction that would fight the clean velocity seed.
        xPoseController.reset();
        yPoseController.reset();
        thetaPoseController.reset();

        Translation2d disp = goal.getTranslation().minus(start.getTranslation());
        double dispNorm = disp.getNorm();
        translationalState.position = dispNorm;
        if (dispNorm < 1e-3) {
            // Already at the goal: no initial velocity to project (avoids divide-by-zero).
            translationalState.velocity = 0;
        } else {
            // Project current field velocity onto the path direction. position is the remaining
            // distance (decreasing), so motion toward the goal is negative.
            ChassisSpeeds fv = drivetrain.getFieldVelocity();
            double seeded = MathUtil.clamp(
                (-1) * (fv.vxMetersPerSecond * disp.getX() + fv.vyMetersPerSecond * disp.getY()) / dispNorm,
                -kMaxVelocity,
                0);
            translationalState.velocity = Double.isFinite(seeded) ? seeded : 0;
        }

        targetTranslationalState.position = 0;
        targetTranslationalState.velocity = 0; // stop at the goal

        targetRotationalState.position = goal.getRotation().getRadians();
        rotationalState.position = drivetrain.getPose().getRotation().getRadians();
        double omega = drivetrain.getRobotVelocity().omegaRadiansPerSecond;
        rotationalState.velocity = Double.isFinite(omega) ? omega : 0;
    }

    /**
     * Drives the drivetrain to a field-relative {@link Setpoint}: feeds the profiled setpoint
     * velocity forward and corrects residual pose error with the pose PID controllers.
     */
    private void followSetpoint(Setpoint setpoint) {
        Pose2d pose = drivetrain.getPose();
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(
            setpoint.vx + xPoseController.calculate(pose.getX(), setpoint.x),
            setpoint.vy + yPoseController.calculate(pose.getY(), setpoint.y),
            setpoint.omega + thetaPoseController.calculate(pose.getRotation().getRadians(), setpoint.theta));
        drivetrain.driveFieldOriented(targetSpeeds);
    }

    /**
     * Advances the trapezoidal profiles one tick and returns the next field-relative setpoint. One
     * translation profile (arc length along the line) and one rotation profile share the timeline,
     * so translation lands once and heading arrives with it. Reads the goal fresh from
     * {@code path.endPose} each tick so continuous re-anchoring is supported.
     */
    public Setpoint getNextSetpoint(AutoAlignPath path) {
        // Translation: profile toward the goal. Re-base the profile's "remaining distance" on the
        // LIVE distance to the (possibly re-anchored) goal each tick, keeping the profiled velocity
        // for continuity. This is the standard "profile toward a moving setpoint" pattern; it
        // prevents the profile state from desyncing from the geometry when the goal moves.
        // Project robot->goal onto the approach line for an exact 1-D along-line remaining distance
        // (a lateral offset must not inflate it); clamp >= 0 in case the robot is past the goal.
        Translation2d toGoal = path.endPose.getTranslation().minus(drivetrain.getPose().getTranslation());
        Translation2d dir = path.getNormalizedDisplacement();
        double remaining = Math.max(0.0, toGoal.getX() * dir.getX() + toGoal.getY() * dir.getY());
        translationalState.position = remaining;
        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
        translationalState.velocity = translationSetpoint.velocity;

        // Rotation: unwrap goal and state near the current heading (continuous-input handling).
        double rotationalPose = drivetrain.getPose().getRotation().getRadians();
        double goalRotation = path.endPose.getRotation().getRadians();
        double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);
        targetRotationalState.position = rotationalPose + angularError;
        double setpointError = MathUtil.angleModulus(rotationalState.position - rotationalPose);
        rotationalState.position = rotationalPose + setpointError;

        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
        rotationalState.position = rotationalSetpoint.position;
        rotationalState.velocity = rotationalSetpoint.velocity;

        // Arc-length parametrization of the straight line from start to end. fraction is clamped to
        // [0, 1] so a re-anchored (closer) goal can never extrapolate the waypoint behind the robot.
        // fraction == 0 -> the goal (endPose); fraction == 1 -> the start.
        double dispNorm = path.getDisplacement().getNorm();
        double fraction = (dispNorm < 1e-6)
            ? 0.0
            : MathUtil.clamp(translationSetpoint.position / dispNorm, 0.0, 1.0);
        Translation2d interpolatedTranslation = path.endPose
            .getTranslation()
            .interpolate(path.startPose.getTranslation(), fraction);

        return new Setpoint(
            interpolatedTranslation.getX(),
            interpolatedTranslation.getY(),
            rotationalState.position,
            path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity,
            path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
            rotationalState.velocity);
    }
}
