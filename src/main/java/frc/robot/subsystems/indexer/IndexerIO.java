package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double floorRPM = 0.0;
        public double tunnelRPM = 0.0;
        public double floorStatorLimitAmps = 40.0;
        public double tunnelStatorLimitAmps = 40.0;
        public boolean floorCurrentLimitOK = true;
        public boolean tunnelCurrentLimitOK = true;
        public double tunnelCurrentAmps = 0.0;
        public String tunnelConfigurationStatus = "Not configured";
        public String tunnelControlStatus = "No request";
        public boolean tunnelConnected = false;
        public boolean tunnelConfigured = false;
        public boolean floorConnected = false;
        public boolean floorSecondaryConnected = false;
        public double floorSecondaryRPM = 0.0;
        public double floorSecondaryAppliedVolts = 0.0;
        public double floorSecondaryCurrentAmps = 0.0;
        public double floorAppliedVolts = 0.0;
        public double floorCurrentAmps = 0.0;
        public String floorConfigurationStatus = "Not configured";
        public String floorControlStatus = "No request";
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    /** Mechanism RPM for both opposed floor motors. Zero stops both. */
    default void setFloorRPM(double rpm) {}

    default void setTunnelRPM(double rpm) {}

    default void stop() {}
}
