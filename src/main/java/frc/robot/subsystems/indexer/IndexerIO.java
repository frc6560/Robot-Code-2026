package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean sensorTriggered = false;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setSpeed(double speed) {}

    default void stop() {}
}
