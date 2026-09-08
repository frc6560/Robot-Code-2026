package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** One Kraken-drive/Kraken-steer/CANcoder module with an LQR wheel-velocity controller. */
class StateSpaceModule {
  private final Constants.ModuleConfig config;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut steerVoltageRequest = new VoltageOut(0.0);
  private final PositionVoltage steerPositionRequest = new PositionVoltage(0.0);
  private final LinearSystemLoop<N1, N1, N1> driveLoop;
  private SwerveModuleState requestedState = new SwerveModuleState();
  private SwerveModuleState desiredState = new SwerveModuleState();
  private boolean stopped = true;
  private boolean hardwareConfigured;
  private boolean optimizationReversed;

  StateSpaceModule(Constants.ModuleConfig config) {
    this.config = config;
    CANBus canBus = new CANBus(Constants.Drive.CAN_BUS);
    driveMotor = new TalonFX(config.driveMotorCanId(), canBus);
    steerMotor = new TalonFX(config.steerMotorCanId(), canBus);
    encoder = new CANcoder(config.encoderCanId(), canBus);
    configureHardware();

    LinearSystem<N1, N1, N1> drivePlant = LinearSystemId.identifyVelocitySystem(
        Constants.Drive.DRIVE_KV_VOLTS_PER_MPS, Constants.Drive.DRIVE_KA_VOLTS_PER_MPS_SQUARED);
    LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
        drivePlant, VecBuilder.fill(0.20), VecBuilder.fill(12.0), Constants.Drive.LOOP_PERIOD_SECONDS);
    KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(), drivePlant, VecBuilder.fill(0.30), VecBuilder.fill(0.15),
        Constants.Drive.LOOP_PERIOD_SECONDS);
    driveLoop = new LinearSystemLoop<>(drivePlant, controller, observer, 12.0,
        Constants.Drive.LOOP_PERIOD_SECONDS);
    driveLoop.reset(VecBuilder.fill(getDriveVelocityMetersPerSecond()));
  }

  private void configureHardware() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drive.DRIVE_CURRENT_LIMIT_AMPS;
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Drive.OPEN_LOOP_RAMP_SECONDS;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    StatusCode driveConfigStatus = applyWithRetry(
        () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = -config.encoderOffsetDegrees() / 360.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    StatusCode encoderConfigStatus = applyWithRetry(
        () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drive.STEER_CURRENT_LIMIT_AMPS;
    steerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Drive.OPEN_LOOP_RAMP_SECONDS;
    steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Drive.STEER_CLOSED_LOOP_RAMP_SECONDS;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Positive closed-loop error must physically increase the CCW-positive CANcoder reading.
    // On this robot the steering motor is mounted with the opposite polarity from the encoder.
    steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Close the loop directly on the calibrated absolute encoder. Avoid fused feedback until the
    // rotor-to-CANcoder phase relationship has been independently verified on this robot.
    steerConfig.Feedback.FeedbackRemoteSensorID = config.encoderCanId();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerConfig.Feedback.RotorToSensorRatio = Constants.Drive.STEER_GEAR_RATIO;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Slot0.kP = Constants.Drive.STEER_KP_VOLTS_PER_ROTATION;
    steerConfig.Slot0.kD = Constants.Drive.STEER_KD_VOLTS_PER_ROTATION_PER_SECOND;
    steerConfig.Slot0.kS = Constants.Drive.STEER_KS_VOLTS;
    steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    steerConfig.Voltage.PeakForwardVoltage = Constants.Drive.MAX_STEER_VOLTAGE;
    steerConfig.Voltage.PeakReverseVoltage = -Constants.Drive.MAX_STEER_VOLTAGE;
    StatusCode steerConfigStatus = applyWithRetry(
        () -> steerMotor.getConfigurator().apply(steerConfig, 0.25));

    hardwareConfigured = driveConfigStatus.isOK()
        && encoderConfigStatus.isOK()
        && steerConfigStatus.isOK();
    if (!hardwareConfigured) {
      DriverStation.reportError(
          "Swerve " + config.name() + " configuration failed: drive=" + driveConfigStatus
              + ", encoder=" + encoderConfigStatus + ", steer=" + steerConfigStatus,
          false);
    }
  }

  private static StatusCode applyWithRetry(Supplier<StatusCode> configAction) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int attempt = 0; attempt < 5; attempt++) {
      status = configAction.get();
      if (status.isOK()) break;
    }
    return status;
  }

  void setDesiredState(SwerveModuleState wantedState) {
    stopped = false;
    Rotation2d currentAngle = getAngle();
    requestedState = new SwerveModuleState(wantedState.speedMetersPerSecond, wantedState.angle);
    desiredState = new SwerveModuleState(requestedState.speedMetersPerSecond, requestedState.angle);
    // WPILib chooses the closest equivalent wheel state from the live measured angle. This must
    // be recomputed for every requested vector rather than remembered across unrelated commands.
    desiredState.optimize(currentAngle);
    optimizationReversed =
        Math.abs(desiredState.angle.minus(wantedState.angle).getDegrees()) > 90.0;
    desiredState.cosineScale(currentAngle);
    driveLoop.setNextR(VecBuilder.fill(desiredState.speedMetersPerSecond));
    driveLoop.correct(VecBuilder.fill(getDriveVelocityMetersPerSecond()));
    driveLoop.predict(Constants.Drive.LOOP_PERIOD_SECONDS);
    double driveVolts = driveLoop.getU(0);
    driveMotor.setControl(driveVoltageRequest.withOutput(MathUtil.clamp(driveVolts, -12.0, 12.0)));

    steerMotor.setControl(steerPositionRequest.withPosition(desiredState.angle.getRotations()));
  }

  /** Stops wheel drive without changing the module's steering angle. */
  void stop() {
    // Retain the last requested angles for diagnostics. Replacing them with each module's current
    // angle made physical settling error appear as a nonzero *desired* axis spread after release.
    requestedState = new SwerveModuleState(0.0, requestedState.angle);
    desiredState = new SwerveModuleState(0.0, desiredState.angle);
    driveMotor.setControl(driveVoltageRequest.withOutput(0.0));
    steerMotor.setControl(steerVoltageRequest.withOutput(0.0));
    if (!stopped) {
      driveLoop.reset(VecBuilder.fill(getDriveVelocityMetersPerSecond()));
      stopped = true;
    }
  }

  SwerveModulePosition getPosition() {
    double driveMotorRotations = driveMotor.getPosition().getValueAsDouble();
    double distanceMeters = driveMotorRotations / Constants.Drive.DRIVE_GEAR_RATIO
        * Constants.Drive.WHEEL_CIRCUMFERENCE_METERS;
    return new SwerveModulePosition(distanceMeters, getAngle());
  }

  SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAngle());
  }

  SwerveModuleState getDesiredState() {
    return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
  }

  SwerveModuleState getRequestedState() {
    return new SwerveModuleState(requestedState.speedMetersPerSecond, requestedState.angle);
  }

  boolean isHardwareConfigured() {
    return hardwareConfigured;
  }

  /** Raw absolute encoder angle before applying this module's calibration offset. */
  double getRawEncoderDegrees() {
    double calibratedRotations = encoder.getAbsolutePosition().getValueAsDouble();
    double rawRotations = MathUtil.inputModulus(
        calibratedRotations + config.encoderOffsetDegrees() / 360.0, 0.0, 1.0);
    return rawRotations * 360.0;
  }

  private double getDriveVelocityMetersPerSecond() {
    double motorRotationsPerSecond = driveMotor.getVelocity().getValueAsDouble();
    return motorRotationsPerSecond / Constants.Drive.DRIVE_GEAR_RATIO
        * Constants.Drive.WHEEL_CIRCUMFERENCE_METERS;
  }

  private Rotation2d getAngle() {
    double encoderRotations = encoder.getAbsolutePosition().getValueAsDouble();
    return Rotation2d.fromRotations(encoderRotations);
  }

  void logMotionCapture(String parentKey) {
    String key = parentKey + "/" + config.name();
    double calibratedEncoderRotations = encoder.getAbsolutePosition().getValueAsDouble();
    double rawEncoderRotations = getRawEncoderDegrees() / 360.0;
    Rotation2d calibratedAngle = Rotation2d.fromRotations(calibratedEncoderRotations);

    Logger.recordOutput(key + "/RawEncoderRotations", rawEncoderRotations);
    Logger.recordOutput(key + "/EncoderRotations", calibratedEncoderRotations);
    Logger.recordOutput(key + "/EncoderDegrees", calibratedAngle.getDegrees());
    Logger.recordOutput(key + "/SelectedFeedbackRotations",
        steerMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(key + "/SteerRotorRotations",
        steerMotor.getRotorPosition().getValueAsDouble());
    Logger.recordOutput(key + "/DesiredAngleDegrees", desiredState.angle.getDegrees());
    Logger.recordOutput(key + "/RequestedAngleDegrees", requestedState.angle.getDegrees());
    Logger.recordOutput(key + "/AngleErrorDegrees",
        desiredState.angle.minus(calibratedAngle).getDegrees());
    Logger.recordOutput(key + "/DesiredSpeedMps", desiredState.speedMetersPerSecond);
    Logger.recordOutput(key + "/MeasuredSpeedMps", getDriveVelocityMetersPerSecond());
    Logger.recordOutput(key + "/SteerAppliedVolts",
        steerMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(key + "/DriveAppliedVolts",
        driveMotor.getMotorVoltage().getValueAsDouble());
  }

  void log() {
    String key = "Swerve/Modules/" + config.name();
    double signedSteerErrorDegrees = desiredState.angle.minus(getAngle()).getDegrees();
    Logger.recordOutput(key + "/DesiredSpeedMps", desiredState.speedMetersPerSecond);
    Logger.recordOutput(key + "/MeasuredSpeedMps", getDriveVelocityMetersPerSecond());
    Logger.recordOutput(key + "/DesiredAngleRadians", desiredState.angle.getRadians());
    Logger.recordOutput(key + "/RequestedAngleRadians", requestedState.angle.getRadians());
    Logger.recordOutput(key + "/AngleRadians", getAngle().getRadians());
    Logger.recordOutput(key + "/SteerErrorDegrees", signedSteerErrorDegrees);
    Logger.recordOutput(key + "/AbsoluteSteerErrorDegrees",
        Math.abs(signedSteerErrorDegrees));
    Logger.recordOutput(key + "/SelectedFeedbackDegrees",
        steerMotor.getPosition().getValueAsDouble() * 360.0);
    Logger.recordOutput(key + "/DriveVolts", driveMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(key + "/SteerVolts", steerMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(key + "/HardwareConfigured", hardwareConfigured);
    Logger.recordOutput(key + "/OptimizationReversed", optimizationReversed);
  }
}
