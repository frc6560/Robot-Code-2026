package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private double targetAngle = 0.0;
  private static final double ANGLE_TOLERANCE = 0.5;

  public Hood(PoseSupplier poseSupplier) {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");

    configureAbsoluteEncoder();
    configureMotor();

    Timer.delay(0.25);
    seedMotorEncoder();
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

    // PID and feedforward
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = HoodConstants.kS;
    slot0.kV = HoodConstants.kV;
    slot0.kA = HoodConstants.kA;
    slot0.kP = HoodConstants.kP;
    slot0.kI = HoodConstants.kI;
    slot0.kD = HoodConstants.kD;

    // Motion Magic (trapezoidal profile) - convert from deg/s to motor rotations/s
    MotionMagicConfigs mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = HoodConstants.kMaxV * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    mm.MotionMagicAcceleration = HoodConstants.kMaxA * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    mm.MotionMagicJerk = 0; // 0 = trapezoidal (no jerk limit)

    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodMotor.getConfigurator().apply(config);
  }

  /** Seeds the motor encoder from the CANcoder absolute position. */
  private void seedMotorEncoder() {
    double cancoderRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    double hoodRotations = cancoderRotations / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
    double motorRotations = hoodRotations * HoodConstants.HOOD_GEAR_RATIO;
    hoodMotor.setPosition(motorRotations);
  }

  public double getHoodAngle() {
    double motorRotations = hoodMotor.getPosition().getValueAsDouble();
    return motorRotations * 360.0 / HoodConstants.HOOD_GEAR_RATIO;
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
  }

  public void setMotorPosition(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    targetAngle = angleDegrees;
  }

  public boolean atTarget() {
    return Math.abs(getHoodAngle() - targetAngle) < ANGLE_TOLERANCE;
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // Convert target angle to motor rotations
    double targetMotorRotations = targetAngle * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    hoodMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));

    double currentAngle = getHoodAngle();
    SmartDashboard.putNumber("Hood/Current Angle", currentAngle);
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Hood/CANcoder Raw", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Hood/Error", targetAngle - currentAngle);
  }

  public interface PoseSupplier {
    Pose2d getPose();
  }
}