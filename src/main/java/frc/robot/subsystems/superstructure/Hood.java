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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.utility.Shooter.ShotCalculator;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  // Visualization
  private final Mechanism2d hoodMech;
  private final MechanismLigament2d hoodArm;

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

    // --- CRITICAL FIX: SEEDING ---
    // Wait briefly for CANcoder to boot up
    Timer.delay(0.25); 
    
    // 1. Read the real angle (Rotations) from the Absolute Encoder
    double absPositionRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    
    // 2. Tell the Motor "This is your current position"
    // (This syncs the internal soft limits to reality)
    hoodMotor.setPosition(absPositionRotations);

    // 3. Reset the motion profile so it doesn't jump
    resetProfileToCurrent();

    // Visualization
    hoodMech = new Mechanism2d(200, 200);
    MechanismRoot2d hoodRoot = hoodMech.getRoot("Hood Root", 100, 100);
    hoodArm = hoodRoot.append(new MechanismLigament2d("Hood Arm", 50, 0));
    hoodArm.setColor(new Color8Bit(255, 165, 0));
    SmartDashboard.putData("Hood Mechanism", hoodMech);
  }

  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0; // TODO: Calibrate this if 0 is not 0
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

    // --- GEARING ---
    // Combining the ratios into one "SensorToMechanismRatio" is usually safer,
    // but if this math worked for you before, we keep it.
    // Ideally: SensorToMechanismRatio = TotalGearRatio
    config.Feedback.SensorToMechanismRatio = 2 * (HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO);
    config.Feedback.RotorToSensorRatio = (44.0/9.0); 

    // Soft limits (0 to 37 degrees converted to Rotations)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 37.0 / 360.0; 
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0 / 360.0; 

    hoodMotor.getConfigurator().apply(config);
  }

  public void setGoalFromCalculator(ShotCalculator calculator) {
    setGoal(calculator.getHoodAzimuth());
  }

  public void manualUp() {
    setGoal(targetAngle + 1); // Nudge up 0.5 deg
  }

  public void manualDown() {
    setGoal(targetAngle - 1); // Nudge down 0.5 deg
  }

  public void setGoal(double goalDeg) {
    // Clamp the target so we never ask the profile to go outside safe limits
    targetAngle = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    hoodGoalState = new TrapezoidProfile.State(targetAngle / 360.0, 0);
  }

  /**
   * SAFETY FIX: Syncs software profile to the real hood angle.
   * Call this in Command.initialize()
   */
  public void resetProfileToCurrent() {
    double currentRotations = getCurrentAngle() / 360.0;
    
    // Reset both states to where we are NOW
    hoodSetpointState = new TrapezoidProfile.State(currentRotations, 0);
    hoodGoalState = new TrapezoidProfile.State(currentRotations, 0);
    
    // Update targetAngle so manual nudge works correctly from here
    targetAngle = currentRotations * 360.0;
  }

  /**
   * Call this in Command.execute()
   */
  public void runControlLoop() {
    // 1. Calculate next step in the profile
    hoodSetpointState = hoodTrapezoidProfile.calculate(0.02, hoodSetpointState, hoodGoalState);

    // 2. Send to motor
    positionControl.Position = hoodSetpointState.position;
    positionControl.Velocity = hoodSetpointState.velocity;
    hoodMotor.setControl(positionControl);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  public double getCurrentAngle() {
    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public double getAbsoluteAngle() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
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
    
    hoodArm.setAngle(getCurrentAngle());
  }

  public interface PoseSupplier {
    Pose2d getPose();
  }
}