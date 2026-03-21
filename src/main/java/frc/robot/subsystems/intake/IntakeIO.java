package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double extendIntakePositionRotations = 0.0;
        public double extendIntakeVelocityRPS = 0.0;
        public double extendIntakeAppliedVolts = 0.0;
        public double extendIntakeCurrentAmps = 0.0;
        public double extendIntakeTempCelsius = 0.0;

        public double spinVelocityRPS = 0.0;
        public double spinAppliedVolts = 0.0;
        public double spinCurrentAmps = 0.0;
        public double spinTempCelsius = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setExtendIntakePercent(double percent) {}

    default void setExtendIntakePosition(double rotations) {}

    default void stopExtendIntake() {}

    default void setSpinPercent(double percent) {}

    default void resetExtendIntakePosition() {}
}
