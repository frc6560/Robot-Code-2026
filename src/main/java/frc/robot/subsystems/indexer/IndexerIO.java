package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double floorPositionRotations = 0.0;
        public double floorVelocityRPS = 0.0;
        public double floorAppliedVolts = 0.0;
        public double floorLeaderCurrentAmps = 0.0;
        public double floorFollowerCurrentAmps = 0.0;

        public double towerPositionRotations = 0.0;
        public double towerVelocityRPS = 0.0;
        public double towerAppliedVolts = 0.0;
        public double towerCurrentAmps = 0.0;

        public double towerSensorDistanceMeters = 0.0;
        public boolean hasGamePiece = false;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setFloorDutyCycle(double dutyCycle) {}

    default void setTowerDutyCycle(double dutyCycle) {}

    default void stop() {}
}
