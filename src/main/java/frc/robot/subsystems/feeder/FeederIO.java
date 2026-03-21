package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double panPositionRotations = 0.0;
        public double panVelocityRPS = 0.0;
        public double panAppliedVolts = 0.0;
        public double panCurrentAmps = 0.0;
        public double panTempCelsius = 0.0;

        public double pusherPositionRotations = 0.0;
        public double pusherVelocityRPS = 0.0;
        public double pusherAppliedVolts = 0.0;
        public double pusherCurrentAmps = 0.0;
        public double pusherTempCelsius = 0.0;

        public double leftPusherAssistVelocityRPS = 0.0;
        public double leftPusherAssistAppliedVolts = 0.0;
        public double leftPusherAssistCurrentAmps = 0.0;
        public double leftPusherAssistTempCelsius = 0.0;

        public double rightPusherAssistVelocityRPS = 0.0;
        public double rightPusherAssistAppliedVolts = 0.0;
        public double rightPusherAssistCurrentAmps = 0.0;
        public double rightPusherAssistTempCelsius = 0.0;
    }

    default void updateInputs(FeederIOInputs inputs) {}

    default void setPanRPM(double rpm) {}

    default void setPusherRPM(double rpm) {}

    default void stop() {}
}
