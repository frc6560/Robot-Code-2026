package frc.robot.intake;

public interface IntakeStateSpaceIO {
  default void updateInputs() {}
  default double getExtensionMeters() { return 0.0; }
  default double getExtensionVelocityMetersPerSecond() { return 0.0; }
  default double getRollerVelocityRps() { return 0.0; }
  default void setDeploymentVoltage(double volts) {}
  default void setRollerVoltage(double volts) {}
  default void stop() {
    setDeploymentVoltage(0.0);
    setRollerVoltage(0.0);
  }
  default void setCoastMode(boolean coast) {}
}
