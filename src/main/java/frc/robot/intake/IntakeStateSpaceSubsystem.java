package frc.robot.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Command-ready intake subsystem; all motor outputs remain capped at half voltage. */
public final class IntakeStateSpaceSubsystem extends SubsystemBase {
  private final IntakeStateSpaceIO io;
  private final IntakeStateSpaceController controller = new IntakeStateSpaceController();
  private IntakeStateSpaceController.Mode requestedMode = IntakeStateSpaceController.Mode.RETRACTED;
  private IntakeStateSpaceController.Output lastOutput;
  private boolean pitCoastMode = false;

  public IntakeStateSpaceSubsystem(IntakeStateSpaceIO io) { this.io = io; }

  public void setMode(IntakeStateSpaceController.Mode mode) {
    requestedMode = mode;
    controller.setMode(mode);
  }

  public void activate() { setMode(IntakeStateSpaceController.Mode.EXTENDED_SPINNING); }
  public void deactivate() { setMode(IntakeStateSpaceController.Mode.EXTENDED); }
  public void activateOuttake() { setMode(IntakeStateSpaceController.Mode.OUTTAKE); }
  public boolean isActive() { return requestedMode == IntakeStateSpaceController.Mode.EXTENDED_SPINNING; }
  public IntakeStateSpaceController.Mode getMode() { return requestedMode; }
  public IntakeStateSpaceController.Output getLastOutput() { return lastOutput; }

  public void setPitCoastMode(boolean enabled) {
    pitCoastMode = enabled;
    if (enabled) {
      requestedMode = IntakeStateSpaceController.Mode.RETRACTED;
      controller.setMode(requestedMode);
      io.stop();
    }
    io.setCoastMode(enabled);
  }

  @Override
  public void periodic() {
    io.updateInputs();
    double extension = Math.max(IntakeStateSpaceController.MIN_EXTENSION_METERS,
        Math.min(IntakeStateSpaceController.MAX_EXTENSION_METERS, io.getExtensionMeters()));
    if (pitCoastMode) {
      io.stop();
      recordTelemetry(null, extension);
      return;
    }
    lastOutput = controller.calculate(extension, io.getExtensionVelocityMetersPerSecond(),
        io.getRollerVelocityRps(), 0.02);
    io.setDeploymentVoltage(lastOutput.deploymentVolts());
    io.setRollerVoltage(lastOutput.rollerVolts());
    recordTelemetry(lastOutput, extension);
  }

  private void recordTelemetry(IntakeStateSpaceController.Output output, double extension) {
    Logger.recordOutput("Intake/StateSpace/Mode", requestedMode.toString());
    Logger.recordOutput("Intake/StateSpace/ExtensionMeters", extension);
    Logger.recordOutput("Intake/StateSpace/ExtensionVelocityMetersPerSecond", io.getExtensionVelocityMetersPerSecond());
    Logger.recordOutput("Intake/StateSpace/RollerVelocityRps", io.getRollerVelocityRps());
    Logger.recordOutput("Intake/StateSpace/PitCoastMode", pitCoastMode);
    if (output != null) {
      Logger.recordOutput("Intake/StateSpace/ReferenceExtensionMeters", output.referenceExtensionMeters());
      Logger.recordOutput("Intake/StateSpace/ReferenceVelocityMetersPerSecond", output.referenceVelocityMetersPerSecond());
      Logger.recordOutput("Intake/StateSpace/ReferenceRollerRps", output.referenceRollerRps());
      Logger.recordOutput("Intake/StateSpace/DeploymentVoltage", output.deploymentVolts());
      Logger.recordOutput("Intake/StateSpace/RollerVoltage", output.rollerVolts());
      Logger.recordOutput("Intake/StateSpace/ExtensionErrorMeters", output.referenceExtensionMeters() - extension);
      Logger.recordOutput("Intake/StateSpace/RollerErrorRps", output.referenceRollerRps() - io.getRollerVelocityRps());
      Logger.recordOutput("Intake/StateSpace/VoltageLimited",
          Math.abs(output.deploymentVolts()) >= IntakeStateSpaceController.HALF_SPEED_VOLTAGE_LIMIT
              || Math.abs(output.rollerVolts()) >= IntakeStateSpaceController.HALF_SPEED_VOLTAGE_LIMIT);
    }
  }
}
