package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double motorPositionRotations = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;

        public double absoluteEncoderPositionRotations = 0.0;
        public double turretAngleDegrees = 0.0;
        public double turretVelocityDegreesPerSec = 0.0;
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(TurretIOInputs inputs) {}

    /** Set the target turret angle in degrees (motor rotations will be calculated internally) */
    default void setTargetAngle(double angleDegrees) {}

    /** Set the target turret angle with velocity feedforward for tracking moving targets */
    default void setTargetAngleWithVelocity(double angleDegrees, double velocityDegreesPerSec) {}

    /** Stop the turret motor */
    default void stop() {}

    /** Select coast mode for manual movement, or restore the normal brake mode. */
    default void setCoastMode(boolean coast) {}

    /** Re-seed the motor encoder from the absolute encoder */
    default void seedMotorEncoder() {}

    /** Set voltage directly (for SysId) */
    default void setVoltage(double volts) {}

    /** Gets kG FF data. */
    default boolean getFF() { return false; }
}
