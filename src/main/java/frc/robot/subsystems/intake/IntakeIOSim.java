package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

/** Simple normalized travel model with current rises at the two mechanical end stops. */
public class IntakeIOSim implements IntakeIO {
    private double position = .5;
    private double volts;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double proposedPosition = position + volts / 12.0 * .02;
        boolean atStop = volts != 0 && (proposedPosition <= 0 || proposedPosition >= 1);
        position = MathUtil.clamp(proposedPosition, 0, 1);
        inputs.motorConnected = true;
        inputs.motorCurrentValid = true;
        inputs.motorAppliedVolts = volts;
        inputs.motorStatorCurrentAmps = volts == 0 ? 0 : atStop ? 35 : 5;
        inputs.motorSupplyCurrentAmps = inputs.motorStatorCurrentAmps * Math.abs(volts) / 12;
        inputs.motorTempCelsius = 25;
        inputs.motorConfigurationStatus = "Simulation";
    }

    @Override public void setMotorVoltage(double volts) { this.volts = MathUtil.clamp(volts, -12, 12); }
    @Override public void stop() { volts = 0; }
}
