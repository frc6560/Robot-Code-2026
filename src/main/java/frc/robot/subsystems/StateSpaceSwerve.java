package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Hardware swerve subsystem. Chassis kinematics create module references; each drive motor then
 * tracks its wheel-speed reference with an LQR/Kalman state-space loop in {@link StateSpaceModule}.
 */
public class StateSpaceSwerve extends SubsystemBase {
  private static final String[] MODULE_DASHBOARD_NAMES = {
      "Front Left", "Front Right", "Back Left", "Back Right"
  };
  private static final int DASHBOARD_UPDATE_DIVISOR = 5;

  private final Pigeon2 gyro = new Pigeon2(
      Constants.Drive.GYRO_CAN_ID, new CANBus(Constants.Drive.GYRO_CAN_BUS));
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
  private final Field2d field = new Field2d();
  private final Mechanism2d moduleView = new Mechanism2d(3.0, 3.0);
  private final MechanismLigament2d[] measuredModuleIndicators =
      new MechanismLigament2d[modules.length];
  private final MechanismLigament2d[] desiredModuleIndicators =
      new MechanismLigament2d[modules.length];
  private int dashboardUpdateCounter;
  private long motionCaptureSample;

  public StateSpaceSwerve() {
    SmartDashboard.putData("Swerve/Field", field);
    for (int i = 0; i < modules.length; i++) {
      double x = Constants.Drive.MODULES[i].location().getX() > 0.0 ? 2.25 : 0.75;
      double y = Constants.Drive.MODULES[i].location().getY() > 0.0 ? 2.25 : 0.75;
      var root = moduleView.getRoot(MODULE_DASHBOARD_NAMES[i], x, y);
      desiredModuleIndicators[i] = root.append(new MechanismLigament2d(
          "Target", 0.55, 0.0, 4.0, new Color8Bit(Color.kOrange)));
      measuredModuleIndicators[i] = root.append(new MechanismLigament2d(
          "Measured", 0.55, 0.0, 8.0, new Color8Bit(Color.kBlue)));
    }
    SmartDashboard.putData("Swerve/Module View", moduleView);
  }

  /** Commands field- or robot-relative chassis velocity. */
  public void drive(ChassisSpeeds requestedSpeeds, boolean fieldRelative) {
    if (Math.hypot(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond)
            < Constants.Drive.IDLE_LINEAR_SPEED_EPSILON_MPS
        && Math.abs(requestedSpeeds.omegaRadiansPerSecond)
            < Constants.Drive.IDLE_ANGULAR_SPEED_EPSILON_RAD_PER_SEC) {
      for (StateSpaceModule module : modules) module.stop();
      return;
    }

    ChassisSpeeds robotRelative = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(requestedSpeeds, getHeading())
        : requestedSpeeds;
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(robotRelative, Constants.Drive.LOOP_PERIOD_SECONDS));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(states[i]);
  }

  /** Drives and records one synchronized controller/sensor sample for offline analysis. */
  public void driveWithControllerCapture(
      ChassisSpeeds requestedSpeeds, boolean fieldRelative,
      double rawLeftX, double rawLeftY, double rawRightX) {
    double timestampSeconds = Timer.getFPGATimestamp();
    drive(requestedSpeeds, fieldRelative);

    ChassisSpeeds robotRelative = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(requestedSpeeds, getHeading())
        : requestedSpeeds;
    String key = "MotionCapture";
    Logger.recordOutput(key + "/Sample", motionCaptureSample++);
    Logger.recordOutput(key + "/TimestampSeconds", timestampSeconds);
    Logger.recordOutput(key + "/Controller/RawLeftX", rawLeftX);
    Logger.recordOutput(key + "/Controller/RawLeftY", rawLeftY);
    Logger.recordOutput(key + "/Controller/RawRightX", rawRightX);
    Logger.recordOutput(key + "/Command/FieldVxMps", requestedSpeeds.vxMetersPerSecond);
    Logger.recordOutput(key + "/Command/FieldVyMps", requestedSpeeds.vyMetersPerSecond);
    Logger.recordOutput(key + "/Command/FieldTranslationAngleDegrees",
        Math.toDegrees(Math.atan2(
            requestedSpeeds.vyMetersPerSecond, requestedSpeeds.vxMetersPerSecond)));
    Logger.recordOutput(key + "/Command/OmegaRadPerSec", requestedSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput(key + "/Command/RobotVxMps", robotRelative.vxMetersPerSecond);
    Logger.recordOutput(key + "/Command/RobotVyMps", robotRelative.vyMetersPerSecond);
    Logger.recordOutput(key + "/Command/RobotTranslationAngleDegrees",
        Math.toDegrees(Math.atan2(
            robotRelative.vyMetersPerSecond, robotRelative.vxMetersPerSecond)));
    Logger.recordOutput(key + "/GyroHeadingDegrees", getHeading().getDegrees());
    for (StateSpaceModule module : modules) module.logMotionCapture(key + "/Modules");
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

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) states[i] = modules[i].getState();
    return states;
  }

  private SwerveModuleState[] getDesiredModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) states[i] = modules[i].getDesiredState();
    return states;
  }

  private SwerveModuleState[] getRequestedModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) states[i] = modules[i].getRequestedState();
    return states;
  }

  /**
   * Largest separation between any two module axes, treating angles 180 degrees apart as
   * equivalent. Checking every pair is important: modules at -5 and +5 degrees are 10 degrees
   * apart even if the first module used by an earlier reference-based calculation sat at zero.
   */
  private static double getAxisSpreadDegrees(SwerveModuleState[] states) {
    double spreadDegrees = 0.0;
    for (int first = 0; first < states.length; first++) {
      for (int second = first + 1; second < states.length; second++) {
        double differenceDegrees = Math.abs(MathUtil.inputModulus(
            states[second].angle.getDegrees() - states[first].angle.getDegrees(),
            -90.0, 90.0));
        spreadDegrees = Math.max(spreadDegrees, differenceDegrees);
      }
    }
    return spreadDegrees;
  }

  private static double getMaxTrackingErrorDegrees(
      SwerveModuleState[] measuredStates, SwerveModuleState[] desiredStates) {
    double maxErrorDegrees = 0.0;
    for (int i = 0; i < measuredStates.length; i++) {
      maxErrorDegrees = Math.max(maxErrorDegrees, Math.abs(
          desiredStates[i].angle.minus(measuredStates[i].angle).getDegrees()));
    }
    return maxErrorDegrees;
  }

  private void updateDashboard(
      SwerveModuleState[] measuredStates, SwerveModuleState[] desiredStates) {
    Pose2d pose = getPose();
    ChassisSpeeds measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);
    field.setRobotPose(pose);

    SmartDashboard.putNumber("Swerve/Robot/Heading (deg)", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Swerve/Robot/Speed (mps)",
        Math.hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond));
    SmartDashboard.putNumber(
        "Swerve/Robot/Turn Rate (radps)", measuredSpeeds.omegaRadiansPerSecond);

    for (int i = 0; i < modules.length; i++) {
      String key = "Swerve/Modules/" + MODULE_DASHBOARD_NAMES[i];
      double measuredLength = 0.25 + 0.55 * Math.min(
          1.0, Math.abs(measuredStates[i].speedMetersPerSecond)
              / Constants.Drive.MAX_SPEED_METERS_PER_SECOND);
      double desiredLength = 0.25 + 0.55 * Math.min(
          1.0, Math.abs(desiredStates[i].speedMetersPerSecond)
              / Constants.Drive.MAX_SPEED_METERS_PER_SECOND);
      measuredModuleIndicators[i].setAngle(measuredStates[i].angle);
      measuredModuleIndicators[i].setLength(measuredLength);
      desiredModuleIndicators[i].setAngle(desiredStates[i].angle);
      desiredModuleIndicators[i].setLength(desiredLength);

      SmartDashboard.putNumber(key + "/Angle (deg)", measuredStates[i].angle.getDegrees());
      SmartDashboard.putNumber(
          key + "/Raw Encoder (deg) - Calibration", modules[i].getRawEncoderDegrees());
      SmartDashboard.putNumber(
          key + "/Target Angle (deg)", desiredStates[i].angle.getDegrees());
      SmartDashboard.putNumber(key + "/Speed (mps)", measuredStates[i].speedMetersPerSecond);
      SmartDashboard.putNumber(
          key + "/Target Speed (mps)", desiredStates[i].speedMetersPerSecond);
      SmartDashboard.putBoolean(key + "/Configured", modules[i].isHardwareConfigured());
    }
  }

  @Override
  public void periodic() {
    poseEstimator.update(getHeading(), getModulePositions());
    SwerveModuleState[] measuredStates = getModuleStates();
    SwerveModuleState[] desiredStates = getDesiredModuleStates();
    SwerveModuleState[] requestedStates = getRequestedModuleStates();

    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/HeadingRadians", getHeading().getRadians());
    Logger.recordOutput("Swerve/ModuleStates/Measured", measuredStates);
    Logger.recordOutput("Swerve/ModuleStates/Desired", desiredStates);
    Logger.recordOutput("Swerve/Diagnostics/RequestedAxisSpreadDegrees",
        getAxisSpreadDegrees(requestedStates));
    Logger.recordOutput("Swerve/Diagnostics/DesiredAxisSpreadDegrees",
        getAxisSpreadDegrees(desiredStates));
    Logger.recordOutput("Swerve/Diagnostics/MeasuredAxisSpreadDegrees",
        getAxisSpreadDegrees(measuredStates));
    Logger.recordOutput("Swerve/Diagnostics/MaxSteerTrackingErrorDegrees",
        getMaxTrackingErrorDegrees(measuredStates, desiredStates));
    for (StateSpaceModule module : modules) module.log();

    if (++dashboardUpdateCounter >= DASHBOARD_UPDATE_DIVISOR) {
      dashboardUpdateCounter = 0;
      updateDashboard(measuredStates, desiredStates);
    }
  }
}
