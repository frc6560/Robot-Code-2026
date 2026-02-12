package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.utility.Shooter.ShotCalculator;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  // Control
  private final PositionVoltage positionControl;
  private final TrapezoidProfile.Constraints hoodConstraints;
  private final TrapezoidProfile hoodTrapezoidProfile;
  
  // State for Motion Profile
  private TrapezoidProfile.State hoodSetpointState = new TrapezoidProfile.State();
  private TrapezoidProfile.State hoodGoalState = new TrapezoidProfile.State();
  private double targetAngle = 0.0;

  public Hood(PoseSupplier poseSupplier) {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");

    positionControl = new PositionVoltage(0.0).withSlot(0);

    // Profile Constraints
    hoodConstraints = new TrapezoidProfile.Constraints(
      HoodConstants.kMaxV,
      HoodConstants.kMaxA
    );
    hoodTrapezoidProfile = new TrapezoidProfile(hoodConstraints);

    configureAbsoluteEncoder();
    configureMotor();

    // Seed motor position from CANcoder
    Timer.delay(0.25);
    hoodMotor.setPosition(getCurrentAngle() / 360.0);
    resetProfileToCurrent();
  }

  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = HoodConstants.HOOD_ABSOLUTE_ENCODER_OFFSET;
    absoluteEncoder.getConfigurator().apply(config);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kS = HoodConstants.kS;
    config.Slot0.kV = HoodConstants.kV;
    config.Slot0.kA = HoodConstants.kA;
    config.Slot0.kP = HoodConstants.kP;
    config.Slot0.kI = HoodConstants.kI;
    config.Slot0.kD = HoodConstants.kD;

    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio from motor to CANcoder
    config.Feedback.RotorToSensorRatio = HoodConstants.HOOD_GEAR_RATIO;

    // Soft limits (0 to 37 degrees converted to rotations)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.HOOD_MAX_ANGLE / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.HOOD_MIN_ANGLE / 360.0;

    hoodMotor.getConfigurator().apply(config);
  }

  public void setGoalFromCalculator(ShotCalculator calculator) {
    setGoal(calculator.getHoodAzimuth());
  }


  public void setGoal(double goalDeg) {
    goalDeg = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    targetAngle = goalDeg;
    hoodGoalState = new TrapezoidProfile.State(targetAngle / 360.0, 0);
  }

  public void resetProfileToCurrent() {
    double currentRotations = getCurrentAngle() / 360.0;
    hoodSetpointState = new TrapezoidProfile.State(currentRotations, 0);
    hoodGoalState = new TrapezoidProfile.State(currentRotations, 0);
    targetAngle = currentRotations * 360.0;
  }

  public void runControlLoop() {
    hoodSetpointState = hoodTrapezoidProfile.calculate(0.02, hoodSetpointState, hoodGoalState);
    positionControl.Position = hoodSetpointState.position;
    positionControl.Velocity = hoodSetpointState.velocity;
    hoodMotor.setControl(positionControl);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  public double getCurrentAngle() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0 / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
  }

  public double getMotorPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget() {
    return Math.abs(getCurrentAngle() - targetAngle) < 1.0; 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Current Angle", getCurrentAngle());
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
  }

  public interface PoseSupplier {
    Pose2d getPose();
  }
}