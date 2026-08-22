package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double leftVelocityRPS = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;

        public double rightVelocityRPS = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerRPM(double rpm) {}

    default void stop() {}

    /** Select pit coast mode; intake rollers normally remain in coast mode as well. */
    default void setCoastMode(boolean coast) {}
}
