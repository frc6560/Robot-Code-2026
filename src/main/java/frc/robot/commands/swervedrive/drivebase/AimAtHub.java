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
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Points the front of the chassis at the alliance hub while retaining driver-controlled
 * field-relative translation.
 */
public class AimAtHub extends Command {
  private final SwerveSubsystem drivebase;
  private final Supplier<ChassisSpeeds> driverSpeeds;
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          DrivebaseConstants.HUB_AIM_KP,
          DrivebaseConstants.HUB_AIM_KI,
          DrivebaseConstants.HUB_AIM_KD,
          new TrapezoidProfile.Constraints(
              DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
              DrivebaseConstants.HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));

  public AimAtHub(SwerveSubsystem drivebase, Supplier<ChassisSpeeds> driverSpeeds) {
    this.drivebase = drivebase;
    this.driverSpeeds = driverSpeeds;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(
        DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS,
        DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    headingController.reset(
        drivebase.getPose().getRotation().getRadians(),
        drivebase.getRobotVelocity().omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Hub Aim/Active", true);
    Logger.recordOutput("HubAim/Active", true);
    Logger.recordOutput("AutoTurn/Active", true);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = driverSpeeds.get();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      // Retain normal manual rotation if the Driver Station has not supplied an alliance.
      drivebase.driveFieldOriented(speeds);
      SmartDashboard.putBoolean("Hub Aim/Has Alliance", false);
      SmartDashboard.putBoolean("Hub Aim/At Target", false);
      Logger.recordOutput("HubAim/HasAlliance", false);
      Logger.recordOutput("HubAim/AtTarget", false);
      return;
    }

    Pose2d robotPose = drivebase.getPose();
    Translation2d hub =
        alliance.get() == Alliance.Blue
            ? FieldConstants.BLUE_HUB_CENTER
            : FieldConstants.RED_HUB_CENTER;
    Translation2d robotToHub = hub.minus(robotPose.getTranslation());
    double targetHeading = Math.atan2(robotToHub.getY(), robotToHub.getX());
    double headingError =
        MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
    double feedbackOmega =
        headingController.calculate(robotPose.getRotation().getRadians(), targetHeading);
    double profileOmega = headingController.getSetpoint().velocity;
    // ProfiledPIDController returns feedback for the profiled position setpoint, but it does not
    // include the setpoint velocity. Add that velocity as feedforward so the chassis actually
    // follows the requested fast trapezoidal profile instead of slowly chasing it with PID alone.
    double omega = feedbackOmega + profileOmega;

    if (headingController.atGoal()) {
      omega = 0.0;
    }
    // Keep a final safety bound even if PID feedback briefly exceeds the profile velocity.
    omega =
        MathUtil.clamp(
            omega,
            -DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
            DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC);

    drivebase.driveFieldOriented(
        new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega));

    SmartDashboard.putBoolean("Hub Aim/Has Alliance", true);
    SmartDashboard.putNumber("Hub Aim/Target Heading Deg", Math.toDegrees(targetHeading));
    SmartDashboard.putNumber(
        "Hub Aim/Heading Error Deg",
        Math.toDegrees(headingError));
    SmartDashboard.putNumber(
        "Hub Aim/Profile Velocity Deg Per Sec",
        Math.toDegrees(headingController.getSetpoint().velocity));
    SmartDashboard.putBoolean("Hub Aim/At Target", headingController.atGoal());

    Logger.recordOutput("HubAim/HasAlliance", true);
    Logger.recordOutput("HubAim/AtTarget", headingController.atGoal());
    Logger.recordOutput("HubAim/TargetHeadingDeg", Math.toDegrees(targetHeading));
    Logger.recordOutput("HubAim/CurrentHeadingDeg", robotPose.getRotation().getDegrees());
    Logger.recordOutput("HubAim/HeadingErrorDeg", Math.toDegrees(headingError));
    Logger.recordOutput("HubAim/CommandedOmegaRadPerSec", omega);
    Logger.recordOutput("HubAim/PIDFeedbackRadPerSec", feedbackOmega);
    Logger.recordOutput("HubAim/ProfileFeedforwardRadPerSec", profileOmega);
    Logger.recordOutput(
        "HubAim/MeasuredOmegaRadPerSec", drivebase.getRobotVelocity().omegaRadiansPerSecond);
    Logger.recordOutput(
        "HubAim/ProfileSetpointPositionDeg",
        Math.toDegrees(headingController.getSetpoint().position));
    Logger.recordOutput(
        "HubAim/ProfileSetpointVelocityDegPerSec",
        Math.toDegrees(headingController.getSetpoint().velocity));
    Logger.recordOutput("HubAim/DistanceToHubMeters", robotToHub.getNorm());
    Logger.recordOutput("HubAim/CommandedFieldVxMetersPerSec", speeds.vxMetersPerSecond);
    Logger.recordOutput("HubAim/CommandedFieldVyMetersPerSec", speeds.vyMetersPerSecond);

    // Keep shooting auto-turn traces together and separate from BLine path-rotation telemetry.
    Logger.recordOutput("AutoTurn/TargetHeadingDeg", Math.toDegrees(targetHeading));
    Logger.recordOutput("AutoTurn/CurrentHeadingDeg", robotPose.getRotation().getDegrees());
    Logger.recordOutput("AutoTurn/HeadingErrorDeg", Math.toDegrees(headingError));
    Logger.recordOutput("AutoTurn/ProfilePositionDeg", Math.toDegrees(headingController.getSetpoint().position));
    Logger.recordOutput("AutoTurn/ProfileVelocityDegPerSec", Math.toDegrees(profileOmega));
    Logger.recordOutput("AutoTurn/PIDFeedbackRadPerSec", feedbackOmega);
    Logger.recordOutput("AutoTurn/CommandedOmegaRadPerSec", omega);
    Logger.recordOutput(
        "AutoTurn/MeasuredOmegaRadPerSec", drivebase.getRobotVelocity().omegaRadiansPerSecond);
    Logger.recordOutput(
        "AutoTurn/MaxVelocityDegPerSec",
        Math.toDegrees(DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC));
    Logger.recordOutput(
        "AutoTurn/MaxAccelerationDegPerSecSq",
        Math.toDegrees(DrivebaseConstants.HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Hub Aim/Active", false);
    SmartDashboard.putBoolean("Hub Aim/At Target", false);
    Logger.recordOutput("HubAim/Active", false);
    Logger.recordOutput("HubAim/AtTarget", false);
    Logger.recordOutput("HubAim/CommandedOmegaRadPerSec", 0.0);
    Logger.recordOutput("AutoTurn/Active", false);
    Logger.recordOutput("AutoTurn/CommandedOmegaRadPerSec", 0.0);
  }
}
