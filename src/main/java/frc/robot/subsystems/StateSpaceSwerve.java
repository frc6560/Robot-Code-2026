package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Hardware swerve subsystem. Chassis kinematics create module references; each drive motor then
 * tracks its wheel-speed reference with an LQR/Kalman state-space loop in {@link StateSpaceModule}.
 */
public class StateSpaceSwerve extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(Constants.Drive.GYRO_CAN_ID, Constants.Drive.CAN_BUS);
  private final StateSpaceModule[] modules = new StateSpaceModule[] {
      new StateSpaceModule(Constants.Drive.FRONT_LEFT),
      new StateSpaceModule(Constants.Drive.FRONT_RIGHT),
      new StateSpaceModule(Constants.Drive.BACK_LEFT),
      new StateSpaceModule(Constants.Drive.BACK_RIGHT)
  };
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.Drive.FRONT_LEFT.location(), Constants.Drive.FRONT_RIGHT.location(),
      Constants.Drive.BACK_LEFT.location(), Constants.Drive.BACK_RIGHT.location());
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, getHeading(), getModulePositions(), new Pose2d());

  /** Commands field- or robot-relative chassis velocity. */
  public void drive(ChassisSpeeds requestedSpeeds, boolean fieldRelative) {
    ChassisSpeeds robotRelative = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(requestedSpeeds, getHeading())
        : requestedSpeeds;
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(robotRelative, Constants.Drive.LOOP_PERIOD_SECONDS));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(states[i]);
  }

  /** Turns all wheels inward/outward to resist pushing while the X button is held. */
  public void lockX() {
    for (int i = 0; i < modules.length; i++) {
      Rotation2d angle = Constants.Drive.MODULES[i].location().getAngle();
      modules[i].setDesiredState(new SwerveModuleState(0.0, angle));
    }
  }

  public void zeroGyro() {
    gyro.setYaw(0.0);
    poseEstimator.resetPosition(getHeading(), getModulePositions(), new Pose2d());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) positions[i] = modules[i].getPosition();
    return positions;
  }

  @Override
  public void periodic() {
    poseEstimator.update(getHeading(), getModulePositions());
    Logger.recordOutput("StateSpaceSwerve/Pose", getPose());
    Logger.recordOutput("StateSpaceSwerve/HeadingRadians", getHeading().getRadians());
    for (StateSpaceModule module : modules) module.log();
  }
}
