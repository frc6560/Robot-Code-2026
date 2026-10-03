package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean leaderConnected = false;
        public boolean followerConnected = false;

        /** Flywheel (mechanism side) velocity. */
        public double velocityRadsPerSec = 0.0;

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

    public static enum ShooterIOOutputMode { COAST, VELOCITY, VOLTAGE }

    /**
     * The full command, shipped every cycle. Nothing is latched in the IO, so whatever the
     * subsystem decides this loop is exactly what the motors get.
     */
    public static class ShooterIOOutputs {
        public ShooterIOOutputMode mode = ShooterIOOutputMode.COAST;
        /** VELOCITY: closed-loop setpoint, mechanism rad/s. */
        public double velocityRadsPerSec = 0.0;
        /** VELOCITY: arbitrary feedforward added on top of the Talon PID, volts. */
        public double feedforwardVolts = 0.0;
        /** VOLTAGE: open-loop output, volts. */
        public double voltage = 0.0;
        /** Talon slot 0 gains, V per motor rot/s. Reapplied only when they change. */
        public double kP = 0.0;
        public double kD = 0.0;
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Applies the outputs computed this cycle. */
    default void applyOutputs(ShooterIOOutputs outputs) {}
}
