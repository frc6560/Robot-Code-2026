package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leaderPositionRotations = 0.0;
        public double leaderVelocityRPS = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double leaderTempCelsius = 0.0;

        public double followerVelocityRPS = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double followerTempCelsius = 0.0;
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Set the flywheel velocity in rotations per second (mechanism side) */
    default void setVelocityRPS(double rps) {}

    /** Set voltage directly (for SysId) */
    default void setVoltage(double volts) {}

    /** Coast the motors (neutral output) */
    default void stop() {}
}
