package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** One Kraken-drive/Kraken-steer/CANcoder module with an LQR wheel-velocity controller. */
class StateSpaceModule {
  private final Constants.ModuleConfig config;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final PositionVoltage steerPositionRequest = new PositionVoltage(0.0);
  private final LinearSystemLoop<N1, N1, N1> driveLoop;
  private SwerveModuleState desiredState = new SwerveModuleState();

  StateSpaceModule(Constants.ModuleConfig config) {
    this.config = config;
    CANBus canBus = new CANBus(Constants.Drive.CAN_BUS);
    driveMotor = new TalonFX(config.driveMotorCanId(), canBus);
    steerMotor = new TalonFX(config.steerMotorCanId(), canBus);
    encoder = new CANcoder(config.encoderCanId(), canBus);
    configureHardware();
    // The steering controller uses the TalonFX integrated sensor. Align it to the CANcoder before
    // the first command so a 0-radian request is a real module angle, not merely rotor zero.
    steerMotor.setPosition(getAngle().getRotations() * Constants.Drive.STEER_GEAR_RATIO);

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
    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drive.STEER_CURRENT_LIMIT_AMPS;
    steerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Drive.OPEN_LOOP_RAMP_SECONDS;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Slot0.kP = 40.0 / Constants.Drive.STEER_GEAR_RATIO;
    steerConfig.Slot0.kD = 0.1 / Constants.Drive.STEER_GEAR_RATIO;
    steerMotor.getConfigurator().apply(steerConfig);
    encoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  void setDesiredState(SwerveModuleState wantedState) {
    desiredState = new SwerveModuleState(wantedState.speedMetersPerSecond, wantedState.angle);
    desiredState.optimize(getAngle());
    driveLoop.setNextR(VecBuilder.fill(desiredState.speedMetersPerSecond));
    driveLoop.correct(VecBuilder.fill(getDriveVelocityMetersPerSecond()));
    double driveVolts = driveLoop.getU(0);
    driveLoop.predict(Constants.Drive.LOOP_PERIOD_SECONDS);
    driveMotor.setControl(driveVoltageRequest.withOutput(MathUtil.clamp(driveVolts, -12.0, 12.0)));

    double targetMotorRotations = desiredState.angle.getRotations() * Constants.Drive.STEER_GEAR_RATIO;
    steerMotor.setControl(steerPositionRequest.withPosition(targetMotorRotations));
  }

  SwerveModulePosition getPosition() {
    double driveMotorRotations = driveMotor.getPosition().getValueAsDouble();
    double distanceMeters = driveMotorRotations / Constants.Drive.DRIVE_GEAR_RATIO
        * Constants.Drive.WHEEL_CIRCUMFERENCE_METERS;
    return new SwerveModulePosition(distanceMeters, getAngle());
  }

  private double getDriveVelocityMetersPerSecond() {
    double motorRotationsPerSecond = driveMotor.getVelocity().getValueAsDouble();
    return motorRotationsPerSecond / Constants.Drive.DRIVE_GEAR_RATIO
        * Constants.Drive.WHEEL_CIRCUMFERENCE_METERS;
  }

  private Rotation2d getAngle() {
    double encoderRotations = encoder.getAbsolutePosition().getValueAsDouble();
    return Rotation2d.fromRotations(encoderRotations - config.encoderOffsetDegrees() / 360.0);
  }

  void log() {
    String key = "StateSpaceSwerve/" + config.name();
    Logger.recordOutput(key + "/DesiredSpeedMps", desiredState.speedMetersPerSecond);
    Logger.recordOutput(key + "/MeasuredSpeedMps", getDriveVelocityMetersPerSecond());
    Logger.recordOutput(key + "/AngleRadians", getAngle().getRadians());
    Logger.recordOutput(key + "/DriveVolts", driveMotor.getMotorVoltage().getValueAsDouble());
  }
}
