package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.utility.shooting.AlliancePass;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import frc.robot.utility.telemetry.RateLimitedTelemetry;

/**
 * Points the front of the chassis at the alliance hub while retaining driver-controlled
 * field-relative translation.
 */
public class AimAtHub extends Command {
  private final SwerveSubsystem drivebase;
  private final Supplier<ChassisSpeeds> driverSpeeds;
  private final Supplier<Translation2d> releasePosition;
  private boolean atTarget;
  private Translation2d translationLead = new Translation2d();
  private Translation2d fieldVelocity = new Translation2d();

  /** Supplied by the shot command before execute(); zero restores original stationary aiming. */
  public void setTranslationCompensation(Translation2d lead, Translation2d velocity) {
    translationLead = lead;
    fieldVelocity = velocity;
  }

  /** Actual heading error and measured angular speed must both be within tolerance. */
  public boolean isAtTarget() { return atTarget; }
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          DrivebaseConstants.HUB_AIM_KP,
          DrivebaseConstants.HUB_AIM_KI,
          DrivebaseConstants.HUB_AIM_KD,
          new TrapezoidProfile.Constraints(
              DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
              DrivebaseConstants.HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));

  public AimAtHub(SwerveSubsystem drivebase, Supplier<ChassisSpeeds> driverSpeeds) {
    this(drivebase, driverSpeeds, () -> drivebase.getPose().getTranslation());
  }

  public AimAtHub(SwerveSubsystem drivebase, Supplier<ChassisSpeeds> driverSpeeds,
      Supplier<Translation2d> releasePosition) {
    this.drivebase = drivebase;
    this.releasePosition = releasePosition;
    this.driverSpeeds = driverSpeeds;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(
        DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS,
        DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    atTarget = false;
    setTranslationCompensation(new Translation2d(), new Translation2d());
    headingController.reset(
        drivebase.getPose().getRotation().getRadians(),
        drivebase.getRobotVelocity().omegaRadiansPerSecond);
    RateLimitedTelemetry.putBoolean("Hub Aim/Active", true);
    RateLimitedTelemetry.recordOutput("HubAim/Active", true);
    RateLimitedTelemetry.recordOutput("AutoTurn/Active", true);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = driverSpeeds.get();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      atTarget = false;
      // Retain normal manual rotation if the Driver Station has not supplied an alliance.
      drivebase.driveFieldOriented(speeds);
      RateLimitedTelemetry.putBoolean("Hub Aim/Has Alliance", false);
      RateLimitedTelemetry.putBoolean("Hub Aim/At Target", false);
      RateLimitedTelemetry.recordOutput("HubAim/HasAlliance", false);
      RateLimitedTelemetry.recordOutput("HubAim/AtTarget", false);
      return;
    }

    Pose2d robotPose = drivebase.getPose();
    Translation2d hub =
        alliance.get() == Alliance.Blue
            ? FieldConstants.BLUE_HUB_CENTER
            : FieldConstants.RED_HUB_CENTER;
    Translation2d robotToHub = hub.minus(releasePosition.get()).minus(translationLead);
    boolean passing = DriverStation.isTeleopEnabled()
        && AlliancePass.outsideZone(alliance.get(), robotPose.getX());
    double targetHeading = passing ? AlliancePass.heading(alliance.get())
        : Math.atan2(robotToHub.getY(), robotToHub.getX());
    double headingError =
        MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
    double trackingOmega = passing ? 0 : trackingOmega(robotToHub, fieldVelocity);
    atTarget = passing
        ? Double.isFinite(headingError) && Math.abs(headingError) <= AlliancePass.ANGLE_TOLERANCE
            && Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond)
                <= DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC
        : isSettled(headingError, drivebase.getRobotVelocity().omegaRadiansPerSecond - trackingOmega);
    double feedbackOmega =
        headingController.calculate(robotPose.getRotation().getRadians(), targetHeading);
    double profileOmega = headingController.getSetpoint().velocity;
    // ProfiledPIDController returns feedback for the profiled position setpoint, but it does not
    // include the setpoint velocity. Add that velocity as feedforward so the chassis actually
    // follows the requested fast trapezoidal profile instead of slowly chasing it with PID alone.
    double omega = feedbackOmega + profileOmega + trackingOmega;

    if (atTarget) {
      omega = trackingOmega;
    }
    // Keep a final safety bound even if PID feedback briefly exceeds the profile velocity.
    omega =
        MathUtil.clamp(
            omega,
            -DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
            DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC);

    drivebase.driveFieldOriented(
        new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega));

    RateLimitedTelemetry.putBoolean("Hub Aim/Has Alliance", true);
    RateLimitedTelemetry.putNumber("Hub Aim/Target Heading Deg", Math.toDegrees(targetHeading));
    RateLimitedTelemetry.putNumber(
        "Hub Aim/Heading Error Deg",
        Math.toDegrees(headingError));
    RateLimitedTelemetry.putNumber(
        "Hub Aim/Profile Velocity Deg Per Sec",
        Math.toDegrees(headingController.getSetpoint().velocity));
    RateLimitedTelemetry.putBoolean("Hub Aim/At Target", atTarget);

    RateLimitedTelemetry.recordOutput("HubAim/AtTarget", atTarget);
    RateLimitedTelemetry.recordOutput("HubAim/HeadingErrorDeg", Math.toDegrees(headingError));

  }

  static double trackingOmega(Translation2d toTarget, Translation2d velocity) {
    double distanceSquared = toTarget.getX() * toTarget.getX() + toTarget.getY() * toTarget.getY();
    return distanceSquared < 1e-9 ? 0
        : (toTarget.getY() * velocity.getX() - toTarget.getX() * velocity.getY()) / distanceSquared;
  }

  static boolean isSettled(double headingError, double measuredOmega) {
    return Double.isFinite(headingError) && Double.isFinite(measuredOmega)
        && Math.abs(headingError) <= DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS
        && Math.abs(measuredOmega) <= DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC;
  }

  @Override
  public void end(boolean interrupted) {
    atTarget = false;
    setTranslationCompensation(new Translation2d(), new Translation2d());
    drivebase.driveFieldOriented(new ChassisSpeeds());
    RateLimitedTelemetry.putBoolean("Hub Aim/Active", false);
    RateLimitedTelemetry.putBoolean("Hub Aim/At Target", false);
    RateLimitedTelemetry.recordOutput("HubAim/Active", false);
    RateLimitedTelemetry.recordOutput("HubAim/AtTarget", false);
    RateLimitedTelemetry.recordOutput("HubAim/CommandedOmegaRadPerSec", 0.0);
    RateLimitedTelemetry.recordOutput("AutoTurn/Active", false);
    RateLimitedTelemetry.recordOutput("AutoTurn/CommandedOmegaRadPerSec", 0.0);
  }
}
