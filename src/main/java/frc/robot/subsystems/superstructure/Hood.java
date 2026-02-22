package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  private double targetAngle = HoodConstants.HOOD_MIN_ANGLE;
  private static final double ANGLE_TOLERANCE = 0.5;

  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);


  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");

    configureAbsoluteEncoder();
    configureMotor();

    seedMotorEncoder();
  }

  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = HoodConstants.HOOD_ABSOLUTE_ENCODER_OFFSET;
    absoluteEncoder.getConfigurator().apply(config);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = config.Slot0;
        slot0.kS = HoodConstants.kS;
        slot0.kV = HoodConstants.kV;
        slot0.kA = HoodConstants.kA;
        slot0.kP = HoodConstants.kP;
        slot0.kI = HoodConstants.kI;
        slot0.kD = HoodConstants.kD;

    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;

    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = HoodConstants.kMaxV * HoodConstants.HOOD_GEAR_RATIO / 360.0; // deg/s -> rot/s
        motionMagicConfigs.MotionMagicAcceleration = HoodConstants.kMaxA * HoodConstants.HOOD_GEAR_RATIO / 360.0;   // deg/s^2 -> rot/s^2
        motionMagicConfigs.MotionMagicJerk = 0;

    hoodMotor.getConfigurator().apply(config);
  }

  private void seedMotorEncoder() {
    Timer.delay(0.25); // Wait for CANcoder to stabilize

    double raw = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); 
    double cancoderRotations = raw - Math.floor(raw); // mod 0 to 1
    double hoodRotations = cancoderRotations / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
    double motorRotations = hoodRotations * HoodConstants.HOOD_GEAR_RATIO;
    hoodMotor.setPosition(motorRotations);
  }

  public double getHoodAngle() {
    double motorPosition = hoodMotor.getPosition().getValueAsDouble();
    return motorPosition / HoodConstants.HOOD_GEAR_RATIO * 360.0; // Convert motor rotations to degrees
  }

  /** Set target goal to a specific angle in degrees
   * @param goalDeg Desired hood angle in SHOT degrees (will be clamped to min/max)
   */
  public void setGoal(double goalDeg) {
    goalDeg = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    goalDeg -= 15.0;
    targetAngle = goalDeg;
  }

  public boolean atTarget() {
    return Math.abs(getHoodAngle() - targetAngle) < ANGLE_TOLERANCE;
  }

  public void stop() {
    
    hoodMotor.stopMotor();
  }

  /** Sets the hood motor to follow the target angle with CTRE's motion magic.*/
  public void setControl(){
    double targetMotorRotations = targetAngle * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    hoodMotor.setControl(m_motionMagicRequest.withPosition(targetMotorRotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Current Angle", getHoodAngle() + 15.0);
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putNumber("Hood/Encoder Rotations", absoluteEncoder.getAbsolutePosition().getValueAsDouble() - Math.floor(absoluteEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Error", getHoodAngle() - targetAngle);

    setControl();
  }
}
