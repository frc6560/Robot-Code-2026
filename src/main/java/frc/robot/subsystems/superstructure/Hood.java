package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Measure;
//import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  private double targetAngle = 11.0;
  private static final double ANGLE_TOLERANCE = 0.5;

  // --- CONTROL REQUESTS ---
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut m_sysIdControl = new VoltageOut(0);

  // --- SYSID ROUTINE ---
  private final SysIdRoutine sysIdRoutine;

  public Hood(PoseSupplier poseSupplier) {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");

    // Configure SysId
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), // Safe Ramp Rate (0.5V/s)
            Volts.of(4.0),             // Safe Step Voltage (4V)
            null,
            (state) -> SignalLogger.writeString("SysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> hoodMotor.setControl(m_sysIdControl.withOutput(voltage.in(Volts))),
            null,
            this
        )
    );

    configureAbsoluteEncoder();
    configureMotor();
    
    // Ideally move this to an initialize method or accept the delay on boot
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

    // 1. PID & FF
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = HoodConstants.kS;
    slot0.kV = HoodConstants.kV;
    slot0.kA = HoodConstants.kA;
    slot0.kP = HoodConstants.kP;
    slot0.kI = HoodConstants.kI;
    slot0.kD = HoodConstants.kD;
    slot0.kG = HoodConstants.kG;

    // 2. Output
    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // 3. Motion Magic
    MotionMagicConfigs mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = HoodConstants.kMaxV * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    mm.MotionMagicAcceleration = HoodConstants.kMaxA * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    mm.MotionMagicJerk = 0;

    // 4. SOFT LIMITS (Essential for SysId Safety)
    SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = HoodConstants.HOOD_MAX_ANGLE * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    softLimits.ReverseSoftLimitThreshold = HoodConstants.HOOD_MIN_ANGLE * HoodConstants.HOOD_GEAR_RATIO / 360.0;

    hoodMotor.getConfigurator().apply(config);
  }

  private void seedMotorEncoder() {
    Timer.delay(0.25);
    double cancoderRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    double hoodRotations = cancoderRotations / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
    double motorRotations = hoodRotations * HoodConstants.HOOD_GEAR_RATIO;
    hoodMotor.setPosition(motorRotations);
  }

  public double getHoodAngle() {
    return hoodMotor.getPosition().getValueAsDouble() / HoodConstants.HOOD_GEAR_RATIO * 360.0;
  }

  public void setGoal(double goalDeg) {
    targetAngle = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
  }

  // --- NEW: EXPLICIT CONTROL METHOD ---
  // Call this from HoodCommand.execute(), NOT periodic()
  public void runControlLoop() {
    double targetMotorRotations = targetAngle * HoodConstants.HOOD_GEAR_RATIO / 360.0;
    hoodMotor.setControl(m_motionMagicRequest.withPosition(targetMotorRotations));
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  // --- SYSID ---
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // Logging Only!
    SmartDashboard.putNumber("Hood/Angle", getHoodAngle());
    SmartDashboard.putNumber("Hood/Target", targetAngle);
    SmartDashboard.putNumber("Hood/Volts", hoodMotor.getMotorVoltage().getValueAsDouble());
    
    // DELETED: setControl() is GONE from here.
  }
  
  public interface PoseSupplier { Pose2d getPose(); }
}