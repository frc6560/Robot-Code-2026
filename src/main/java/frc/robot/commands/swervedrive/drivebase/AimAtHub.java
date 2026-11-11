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

/**
 * Points the front of the chassis at the alliance hub while retaining driver-controlled
 * field-relative translation.
 */
public class AimAtHub extends Command {
  private final SwerveSubsystem drivebase;
  private final Supplier<ChassisSpeeds> driverSpeeds;
  private final Supplier<Translation2d> releasePosition;
  private final ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  private boolean atTarget;
  private double nextTelemetry;

  /** Heading and measured turn rate must be within the acquire/hold tolerances. */
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
    nextTelemetry = 0.0;
    headingController.reset(
        drivebase.getPose().getRotation().getRadians(),
        drivebase.getRobotVelocity().omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Hub Aim/Active", true);
    SmartDashboard.putBoolean("HubAim/Active", true);
    SmartDashboard.putBoolean("AutoTurn/Active", true);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = driverSpeeds.get();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      atTarget = false;
      // Retain normal manual rotation if the Driver Station has not supplied an alliance.
      drivebase.driveFieldOriented(speeds);
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      if (edu.wpi.first.wpilibj.RobotBase.isReal() && now < nextTelemetry) return;
      nextTelemetry = now + 0.5;
      SmartDashboard.putBoolean("Hub Aim/Has Alliance", false);
      SmartDashboard.putBoolean("Hub Aim/At Target", false);
      SmartDashboard.putBoolean("HubAim/HasAlliance", false);
      SmartDashboard.putBoolean("HubAim/AtTarget", false);
      return;
    }

    Pose2d robotPose = drivebase.getPose();
    Translation2d hub =
        alliance.get() == Alliance.Blue
            ? FieldConstants.BLUE_HUB_CENTER
            : FieldConstants.RED_HUB_CENTER;
    Translation2d release = releasePosition.get();
    boolean passing = DriverStation.isTeleopEnabled()
        && AlliancePass.outsideZone(alliance.get(), robotPose.getX());
    double targetHeading = passing ? AlliancePass.heading(alliance.get())
        : Math.atan2(hub.getY() - release.getY(), hub.getX() - release.getX());
    double headingError =
        MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
    double headingTolerance = passing ? AlliancePass.ANGLE_TOLERANCE
        : DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS;
    double measuredOmega = drivebase.getRobotVelocity().omegaRadiansPerSecond;
    atTarget = isSettled(headingError, measuredOmega, headingTolerance, atTarget);
    double omega;
    if (Double.isFinite(headingError) && Math.abs(headingError) <= headingTolerance) {
      // Brake inside the heading deadband even while waiting for the chassis to settle.
      // Reset the profile so a noisy turn-rate sample cannot resume an old turn trajectory.
      omega = 0.0;
      headingController.reset(robotPose.getRotation().getRadians(), 0.0);
    } else {
      double feedbackOmega =
          headingController.calculate(robotPose.getRotation().getRadians(), targetHeading);
      omega = feedbackOmega + headingController.getSetpoint().velocity;
    }
    // Keep a final safety bound even if PID feedback briefly exceeds the profile velocity.
    omega =
        MathUtil.clamp(
            omega,
            -DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
            DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC);

    commandedSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    commandedSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    commandedSpeeds.omegaRadiansPerSecond = omega;
    drivebase.driveFieldOriented(commandedSpeeds);

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (edu.wpi.first.wpilibj.RobotBase.isReal() && now < nextTelemetry) return;
    nextTelemetry = now + 0.5;
    SmartDashboard.putBoolean("Hub Aim/Has Alliance", true);
    SmartDashboard.putNumber("Hub Aim/Target Heading Deg", Math.toDegrees(targetHeading));
    SmartDashboard.putNumber(
        "Hub Aim/Heading Error Deg",
        Math.toDegrees(headingError));
    SmartDashboard.putNumber(
        "Hub Aim/Profile Velocity Deg Per Sec",
        Math.toDegrees(headingController.getSetpoint().velocity));
    SmartDashboard.putBoolean("Hub Aim/At Target", atTarget);

    SmartDashboard.putBoolean("HubAim/AtTarget", atTarget);
    SmartDashboard.putNumber("HubAim/HeadingErrorDeg", Math.toDegrees(headingError));
    SmartDashboard.putNumber("Hub Aim/Measured Turn Rate Deg Per Sec", Math.toDegrees(measuredOmega));
    SmartDashboard.putNumber("HubAim/MeasuredOmegaRadPerSec", measuredOmega);
    SmartDashboard.putNumber("HubAim/CommandedOmegaRadPerSec", omega);

  }

  static boolean isSettled(double headingError, double measuredOmega) {
    return isSettled(headingError, measuredOmega,
        DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS, false);
  }

  /** Acquire below 5 deg/s; release above 8 deg/s. Heading limits never widen. */
  public static boolean isSettled(double headingError, double measuredOmega,
      double headingTolerance, boolean wasAligned) {
    double velocityTolerance = wasAligned
        ? DrivebaseConstants.HUB_AIM_VELOCITY_RELEASE_TOLERANCE_RAD_PER_SEC
        : DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC;
    return Double.isFinite(headingError) && Double.isFinite(measuredOmega)
        && Math.abs(headingError) <= headingTolerance
        && Math.abs(measuredOmega) <= velocityTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    atTarget = false;
    commandedSpeeds.vxMetersPerSecond = 0;
    commandedSpeeds.vyMetersPerSecond = 0;
    commandedSpeeds.omegaRadiansPerSecond = 0;
    drivebase.driveFieldOriented(commandedSpeeds);
    SmartDashboard.putBoolean("Hub Aim/Active", false);
    SmartDashboard.putBoolean("Hub Aim/At Target", false);
    SmartDashboard.putBoolean("HubAim/Active", false);
    SmartDashboard.putBoolean("HubAim/AtTarget", false);
    SmartDashboard.putNumber("HubAim/CommandedOmegaRadPerSec", 0.0);
    SmartDashboard.putBoolean("AutoTurn/Active", false);
    SmartDashboard.putNumber("AutoTurn/CommandedOmegaRadPerSec", 0.0);
  }
}
