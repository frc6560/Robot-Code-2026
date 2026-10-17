package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public boolean motorConnected = false;
        public boolean motorCurrentValid = false;
        public double motorStatorCurrentAmps = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorTempCelsius = 0.0;
        public String motorConfigurationStatus = "Not configured";
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    /** Positive voltage extends; negative voltage retracts. Zero must stop. */
    default void setMotorVoltage(double volts) {}
    default void stop() {}
    default void setCoastMode(boolean coast) {}
}
