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
      return;
    }

    Pose2d robotPose = drivebase.getPose();
    Translation2d hub =
        alliance.get() == Alliance.Blue
            ? FieldConstants.BLUE_HUB_CENTER
            : FieldConstants.RED_HUB_CENTER;
    Translation2d robotToHub = hub.minus(robotPose.getTranslation());
    double targetHeading = Math.atan2(robotToHub.getY(), robotToHub.getX());
    double omega =
        headingController.calculate(robotPose.getRotation().getRadians(), targetHeading);

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
        Math.toDegrees(MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians())));
    SmartDashboard.putNumber(
        "Hub Aim/Profile Velocity Deg Per Sec",
        Math.toDegrees(headingController.getSetpoint().velocity));
    SmartDashboard.putBoolean("Hub Aim/At Target", headingController.atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Hub Aim/Active", false);
    SmartDashboard.putBoolean("Hub Aim/At Target", false);
  }
}
