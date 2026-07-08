package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;


public class Autoalign extends SequentialCommandGroup {
    
    // Autopilot configuration
    private static final APConstraints kConstraints = new APConstraints()
        .withVelocity(0.3)      // TESTING: very low top speed (m/s)
        .withAcceleration(0.3)  // TESTING: very low acceleration (m/s^2)
        .withJerk(1.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    // Camera + tag-frame goal. SAME frame getTagRelativeRobotPose() returns:
    //   x = meters out from the tag face, y = meters to the left, heading = pi means facing the tag.
    private static final String kCamera = "limelight-br";
    private static final double kStandoffMeters = 0.5; // TUNE: tag-face -> robot-center distance
    // Heading target in the CORRECTED tag frame (heading 0 = pointing out of the tag face, away from
    // it). The old frame was rotated 180 deg, so its empirically-found "0" is pi here. VERIFY on
    // robot: position the robot exactly as it should finish, read "Climb/Prescore/Current_HeadingDeg",
    // and set this heading to that value.
    private static final Pose2d kTargetPose = new Pose2d(kStandoffMeters, 0.0, new Rotation2d(Math.PI));
    // Entry angle = the DIRECTION OF MOTION at arrival (not which way the robot faces). The goal sits
    // in front of the tag (at +x standoff) and we approach from further out, so we arrive moving
    // TOWARD the tag = -x = pi.
    private static final Rotation2d kEntryAngle = new Rotation2d(Math.PI);

    // Tag-frame map for Glass/AdvantageScope: shows the tag, the target, and the robot. Everything
    // is in the TAG frame (tag at the origin), shifted by kVizOffset so it draws mid-canvas instead
    // of at the field corner (Field2d clips negative coordinates).
    private final Field2d m_field = new Field2d();
    private static final Translation2d kVizOffset = new Translation2d(8.0, 4.0);

    // PID Controllers (kept for backward compatibility and heading control)
    private PIDController rotationController;

    // Subsystems
    private SwerveSubsystem drivetrain;


    /** Constructor for our climb command */
    public Autoalign(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        // Heading controller for Autopilot's target-angle reference. (MUST be constructed before use.)
        this.rotationController = new PIDController(5.5, 0.15, 0.05);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Log initialization
        SmartDashboard.putString("Climb/Status", "Initialized");
        // Publish the tag-frame map once; per-tick code updates the poses on it.
        SmartDashboard.putData("Climb/Prescore/Field", m_field);
        m_field.getObject("tag").setPose(new Pose2d(kVizOffset, new Rotation2d()));
        m_field.getObject("target").setPose(
            new Pose2d(kTargetPose.getTranslation().plus(kVizOffset), kTargetPose.getRotation()));

        super.addCommands(
            getDriveToTarget()
        );
        super.addRequirements(drivetrain);
    }

    /** Update PID values from SmartDashboard (call this in execute if you want live tuning) */


    /** Drives to the tag-relative target using Autopilot (everything in the TAG frame). */
    public Command getDriveToTarget() {
        return Commands.run(() -> {
            // NO-TAG GUARD: if the Limelight isn't publishing a valid target-space pose, botpose is
            // all-zeros, so currentPose would be a FIXED garbage pose -> the robot spins forever with
            // no translation. Hold still until we actually see the tag.
            Pose3d raw = LimelightHelpers.getBotPose3d_TargetSpace(kCamera);
            double botposeNorm = raw.getTranslation().getNorm();
            SmartDashboard.putBoolean("Climb/Prescore/HasTarget", LimelightHelpers.getTV(kCamera));
            SmartDashboard.putNumber("Climb/Prescore/BotposeNorm", botposeNorm);
            SmartDashboard.putNumber("Climb/Prescore/Tid", LimelightHelpers.getFiducialID(kCamera));
            // Raw crosshair offsets (degrees) -- plot these to sanity-check the camera itself.
            SmartDashboard.putNumber("Climb/Prescore/tx", LimelightHelpers.getTX(kCamera));
            SmartDashboard.putNumber("Climb/Prescore/ty", LimelightHelpers.getTY(kCamera));
            if (botposeNorm < 1e-3) {
                drivetrain.drive(new ChassisSpeeds()); // no valid tag -> stop, don't spin on garbage
                SmartDashboard.putBoolean("Climb/Prescore/At_Target", false);
                return;
            }

            Pose2d currentPose = getTagRelativeRobotPose(kCamera);          // robot in tag frame

            // Update the tag-frame map: robot + target (tag stays pinned at kVizOffset).
            m_field.setRobotPose(
                new Pose2d(currentPose.getTranslation().plus(kVizOffset), currentPose.getRotation()));
            m_field.getObject("target").setPose(
                new Pose2d(kTargetPose.getTranslation().plus(kVizOffset), kTargetPose.getRotation()));

            ChassisSpeeds robotRelativeSpeeds = drivetrain.getRobotVelocity();

            APTarget target = new APTarget(kTargetPose).withEntryAngle(kEntryAngle);

            Autopilot.APResult output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, target);

            // Autopilot's vx/vy are in the pose frame (= tag frame here).
            double xVel = output.vx().in(edu.wpi.first.units.Units.MetersPerSecond);
            double yVel = output.vy().in(edu.wpi.first.units.Units.MetersPerSecond);
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                output.targetAngle().getRadians()
            );

            // Rotate Autopilot's TAG-frame vx/vy into the gyro's "field" frame using the per-tick
            // offset (gyro - visionHeading). NO field layout / global pose: the gyro term cancels
            // exactly inside driveFieldOriented, so this works with ANY gyro zero -- net effect is
            // "tag-frame velocity rotated into the robot frame by the vision heading", with the gyro
            // only stabilizing the command between vision updates (offset is constant while tracking,
            // even mid-rotation).
            Rotation2d gyro = drivetrain.getPose().getRotation();
            Rotation2d tagToField = gyro.minus(currentPose.getRotation());
            Translation2d velField = new Translation2d(xVel, yVel).rotateBy(tagToField);
            drivetrain.driveFieldOriented(new ChassisSpeeds(velField.getX(), velField.getY(), rotVel));
            SmartDashboard.putNumber("Climb/Prescore/GyroDeg", gyro.getDegrees());
            SmartDashboard.putNumber("Climb/Prescore/TagToFieldDeg", tagToField.getDegrees());
            SmartDashboard.putNumber("Climb/Prescore/Output_Field_X_Vel", velField.getX());
            SmartDashboard.putNumber("Climb/Prescore/Output_Field_Y_Vel", velField.getY());

            // ---- Essential debug logging ----
            // Vision health (the usual failure: no botpose_targetspace from the Limelight).
            SmartDashboard.putBoolean("Climb/Prescore/HasTarget", LimelightHelpers.getTV(kCamera));
            SmartDashboard.putNumber("Climb/Prescore/Tid", LimelightHelpers.getFiducialID(kCamera));
            SmartDashboard.putNumber("Climb/Prescore/BotposeNorm",
                LimelightHelpers.getBotPose3d_TargetSpace(kCamera).getTranslation().getNorm());
            // Robot pose in the tag frame (what we feed Autopilot).
            SmartDashboard.putNumber("Climb/Prescore/Current_X", currentPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Current_Y", currentPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Current_HeadingDeg", currentPose.getRotation().getDegrees());
            // Target (tag frame).
            SmartDashboard.putNumber("Climb/Prescore/Target_X", kTargetPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Target_Y", kTargetPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Target_HeadingDeg", kTargetPose.getRotation().getDegrees());
            // Errors + distance.
            SmartDashboard.putNumber("Climb/Prescore/Error_X", kTargetPose.getX() - currentPose.getX());
            SmartDashboard.putNumber("Climb/Prescore/Error_Y", kTargetPose.getY() - currentPose.getY());
            SmartDashboard.putNumber("Climb/Prescore/Error_Rot_Deg",
                Math.toDegrees(kTargetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));
            SmartDashboard.putNumber("Climb/Prescore/Distance",
                currentPose.getTranslation().getDistance(kTargetPose.getTranslation()));
            // Autopilot output (commanded, tag frame) + its heading reference.
            SmartDashboard.putNumber("Climb/Prescore/Output_X_Vel", xVel);
            SmartDashboard.putNumber("Climb/Prescore/Output_Y_Vel", yVel);
            SmartDashboard.putNumber("Climb/Prescore/Output_Rot_Vel", rotVel);
            SmartDashboard.putNumber("Climb/Prescore/TargetAngleDeg", output.targetAngle().getDegrees());
            SmartDashboard.putBoolean("Climb/Prescore/At_Target", kAutopilot.atTarget(currentPose, target));
        }, drivetrain).until(() -> {
            APTarget target = new APTarget(kTargetPose).withEntryAngle(kEntryAngle);
            return kAutopilot.atTarget(getTagRelativeRobotPose(kCamera), target);
        }).finallyDo(() -> drivetrain.drive(new ChassisSpeeds()));
    }

    private Pose2d getTagRelativeRobotPose(String limelightName) {
        Pose3d r = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
        Translation3d fwd = new Translation3d(1, 0, 0).rotateBy(r.getRotation());
        // MEASURED ON ROBOT (dashboard 2026-07-06): a robot IN FRONT of the tag reads NEGATIVE Z
        // (Current_X was -2.44 at ~2.4m out), i.e. this Limelight's target-space Z+ points INTO the
        // tag, X+ = viewer's right. Planar frame: x = out of tag face = -Z, y = X (right-handed),
        // heading from the projected forward vector. The old (r.getZ(), -r.getX()) remap was this
        // frame rotated 180 deg -> the (0.5, 0) target sat half a meter BEHIND the wall.
        return new Pose2d(-r.getZ(), r.getX(), new Rotation2d(Math.atan2(fwd.getX(), -fwd.getZ())));
    }
    }




