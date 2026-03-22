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
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(FeederIOInputs inputs) {}

    /** Set the pan motor velocity in RPM (mechanism side) */
    default void setPanRPM(double rpm) {}

    /** Set the pusher motor velocity in RPM (mechanism side) */
    default void setPusherRPM(double rpm) {}

    /** Sets the wall motor velocity in RPM */
    default void setWallRPM(double rpm) {}

    /** Sets the floor motor velocity in RPM */
    default void setFloorRPM(double rpm) {}

    /** Stop both motors */
    default void stop() {}
}
