package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double motorPositionRotations = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;

        public double absoluteEncoderPositionRotations = 0.0;
        public double hoodAngleDegrees = 0.0;
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(HoodIOInputs inputs) {}

    /** Set the target hood angle in degrees (motor rotations will be calculated internally) */
    default void setTargetAngle(double angleDegrees) {}

    /** Stop the hood motor */
    default void stop() {}

    /** Set voltage directly (for SysId) */
    default void setVoltage(double volts) {}
}
