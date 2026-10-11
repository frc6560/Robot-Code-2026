package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double deployPositionRotations = 0.0;
        public double deployVelocityRPS = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
        public double deployTempCelsius = 0.0;

        public double rollerVelocityRPS = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerRPM(double rpm) {}

    default void setDeployPosition(double motorRotations) {}

    default void stop() {}

    /** Select pit coast mode; intake rollers normally remain in coast mode as well. */
    default void setCoastMode(boolean coast) {}
}
