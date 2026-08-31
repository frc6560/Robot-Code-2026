package frc.robot.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX implementation for one X44 extension motor and one X60 roller motor. */
public final class TalonFXIntakeStateSpaceIO implements IntakeStateSpaceIO {
  public static final int EXTENSION_MOTOR_CAN_ID = 15;
  public static final int ROLLER_MOTOR_CAN_ID = 16;
  public static final String CAN_BUS = "rio";
  private static final double PINION_DIAMETER_INCHES = 1.751;
  private static final double EXTENSION_GEAR_RATIO = 64.0 / 14.0;
  private static final double METERS_PER_MOTOR_ROTATION =
      Math.PI * PINION_DIAMETER_INCHES * 0.0254 / EXTENSION_GEAR_RATIO;
  private static final double MAX_EXTENSION_ROTATIONS =
      IntakeStateSpaceController.MAX_EXTENSION_METERS / METERS_PER_MOTOR_ROTATION;

  private final TalonFX extensionMotor = new TalonFX(EXTENSION_MOTOR_CAN_ID, CAN_BUS);
  private final TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_CAN_ID, CAN_BUS);
  private final StatusSignal<Angle> extensionPosition = extensionMotor.getPosition();
  private final StatusSignal<AngularVelocity> extensionVelocity = extensionMotor.getVelocity();
  private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
  private final StatusSignal<Voltage> extensionVoltage = extensionMotor.getMotorVoltage();
  private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();

  public TalonFXIntakeStateSpaceIO() {
    configureExtensionMotor();
    configureRollerMotor();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, extensionPosition, extensionVelocity,
        rollerVelocity, extensionVoltage, rollerVoltage);
    extensionMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
  }

  private void configureExtensionMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_EXTENSION_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    extensionMotor.getConfigurator().apply(config);
  }

  private void configureRollerMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerMotor.getConfigurator().apply(config);
  }

  @Override public void updateInputs() { BaseStatusSignal.refreshAll(extensionPosition, extensionVelocity, rollerVelocity, extensionVoltage, rollerVoltage); }
  @Override public double getExtensionMeters() { return extensionPosition.getValueAsDouble() * METERS_PER_MOTOR_ROTATION; }
  @Override public double getExtensionVelocityMetersPerSecond() { return extensionVelocity.getValueAsDouble() * METERS_PER_MOTOR_ROTATION; }
  @Override public double getRollerVelocityRps() { return rollerVelocity.getValueAsDouble(); }
  @Override public void setDeploymentVoltage(double volts) { extensionMotor.setVoltage(clamp(volts)); }
  @Override public void setRollerVoltage(double volts) { rollerMotor.setVoltage(clamp(volts)); }
  @Override public void stop() { extensionMotor.stopMotor(); rollerMotor.stopMotor(); }
  @Override public void setCoastMode(boolean coast) {
    extensionMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void zeroExtensionEncoder() { extensionMotor.setPosition(0.0); }
  private static double clamp(double volts) { return Math.max(-6.0, Math.min(6.0, volts)); }
}
