package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double extendPositionRotations = 0.0;
        public double extendVelocityRPS = 0.0;
        public double extendAppliedVolts = 0.0;
        public double extendCurrentAmps = 0.0;
        public double extendTempCelsius = 0.0;

        public double spinVelocityRPS = 0.0;
        public double spinAppliedVolts = 0.0;
        public double spinCurrentAmps = 0.0;
        public double spinTempCelsius = 0.0;

        public boolean retractLimitSwitch = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setExtendPercent(double percent) {}

    default void setSpinPercent(double percent) {}

    default void resetExtendPosition() {}

    default void setSpringyCurrentLimits(boolean springy) {}
}
