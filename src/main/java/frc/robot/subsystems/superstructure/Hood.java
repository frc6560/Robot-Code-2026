package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.utility.Shooter.ShotCalculator;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;
  private final PositionVoltage positionControl;

  private double targetAngle = 0.0;

  public Hood(PoseSupplier poseSupplier) {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");
    positionControl = new PositionVoltage(0.0).withSlot(0);

    configureAbsoluteEncoder();
    configureMotor();

    // Seed motor position from CANcoder
    Timer.delay(0.25);
    hoodMotor.setPosition(getHoodAngle() / 360.0 * HoodConstants.HOOD_GEAR_RATIO);
    targetAngle = getHoodAngle();
  }

  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = HoodConstants.HOOD_ABSOLUTE_ENCODER_OFFSET;
    absoluteEncoder.getConfigurator().apply(config);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Motor spins HOOD_GEAR_RATIO / ABSOLUTE_HOOD_ENCODER_GEAR_RATIO times per CANcoder rotation
    config.Feedback.RotorToSensorRatio = HoodConstants.HOOD_GEAR_RATIO / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;

    hoodMotor.getConfigurator().apply(config);

    // PID
    Slot0Configs pidConfig = new Slot0Configs();
    pidConfig.kS = HoodConstants.kS;
    pidConfig.kV = HoodConstants.kV;
    pidConfig.kA = HoodConstants.kA;
    pidConfig.kP = HoodConstants.kP;
    pidConfig.kI = HoodConstants.kI;
    pidConfig.kD = HoodConstants.kD;

    hoodMotor.getConfigurator().apply(pidConfig);
  }

  public double getHoodAngle() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0 / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
  }

  public double getMotorPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public void setGoalFromCalculator(ShotCalculator calculator) {
    setGoal(calculator.getHoodAzimuth());
  }

  public void setGoal(double goalDeg) {
    goalDeg = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    targetAngle = goalDeg;
    setMotorPosition(goalDeg);
  }

  public void setMotorPosition(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    double targetMotorRotations = angleDegrees / 360.0 * HoodConstants.HOOD_GEAR_RATIO;
    positionControl.Position = targetMotorRotations;
    hoodMotor.setControl(positionControl);
  }

  public boolean atTarget() {
    return Math.abs(getHoodAngle() - targetAngle) < 1.0;
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Current Angle", getHoodAngle());
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Hood/CANcoder Raw", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Position", getMotorPosition());
  }

  public interface PoseSupplier {
    Pose2d getPose();
  }
}
