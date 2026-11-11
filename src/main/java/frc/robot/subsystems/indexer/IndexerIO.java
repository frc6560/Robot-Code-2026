package frc.robot.subsystems.indexer;


public interface IndexerIO {
    public static class IndexerIOInputs {
        public double floorPositionRotations, floorSecondaryPositionRotations, tunnelPositionRotations;
        public double appliedVolts = 0.0;
        public double floorRPM = 0.0;
        public double tunnelRPM = 0.0;
        public double floorStatorLimitAmps = 40.0;
        public double tunnelStatorLimitAmps = 40.0;
        public boolean floorCurrentLimitOK = true;
        public boolean tunnelCurrentLimitOK = true;
        public double tunnelCurrentAmps = 0.0;
        public boolean tunnelConnected = false;
        public boolean tunnelConfigured = false;
        public boolean floorConnected = false;
        public boolean floorSecondaryConnected = false;
        public double floorSecondaryRPM = 0.0;
        public double floorSecondaryAppliedVolts = 0.0;
        public double floorSecondaryCurrentAmps = 0.0;
        public double floorAppliedVolts = 0.0;
        public double floorCurrentAmps = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    /** Mechanism RPM for both opposed floor motors. Zero stops both. */
    default void setFloorRPM(double rpm) {}

    default void setTunnelRPM(double rpm) {}

    default void stop() {}
}
