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

    /** Updates the set of loggable inputs */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Set the extend motor percent output */
    default void setExtendPercent(double percent) {}

    /** Set the spin motor percent output */
    default void setSpinPercent(double percent) {}

    /** Reset the extend motor position to zero */
    default void resetExtendPosition() {}

    /** Apply springy current limits (lower limits for springy mode) */
    default void setSpringyCurrentLimits(boolean springy) {}
}
