package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
  private final DutyCycleOut dutyCycleControl;

  private double targetAngle = 0.0;
  private static final double DUTY_CYCLE_SPEED = 0.3;
  private static final double ANGLE_TOLERANCE = 0.5;

  public Hood(PoseSupplier poseSupplier) {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");
    dutyCycleControl = new DutyCycleOut(0.0);

    configureAbsoluteEncoder();
    configureMotor();

    Timer.delay(0.25);
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

    hoodMotor.getConfigurator().apply(config);
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
    // Open loop control
    double currentAngle = getHoodAngle();
    double error = targetAngle - currentAngle;

    if (Math.abs(error) > ANGLE_TOLERANCE) {
      // Move toward target
      double speed = error > 0 ? DUTY_CYCLE_SPEED : -DUTY_CYCLE_SPEED;
      hoodMotor.setControl(dutyCycleControl.withOutput(speed));
    } else {
      // At target, stop
      hoodMotor.setControl(dutyCycleControl.withOutput(0.0));
    }

    SmartDashboard.putNumber("Hood/Current Angle", currentAngle);
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Hood/CANcoder Raw", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Hood/Error", error);
  }

  public interface PoseSupplier {
    Pose2d getPose();
  }
}
